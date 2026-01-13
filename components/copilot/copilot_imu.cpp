#include "copilot_imu.h"

#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "copilot_ui.h"

static const char *TAG = "copilot_imu";

// NVS namespace and keys for calibration data
#define NVS_NAMESPACE "copilot_imu"
#define NVS_KEY_GYRO_BIAS "gyro_bias"
#define NVS_KEY_CAL_VALID "cal_valid"

#ifdef CONFIG_COPILOT_MOTION_SOURCE_INTERNAL

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "qmi8658.h"
#include "bsp/esp32_s3_touch_amoled_1_75.h"

// ============================================================================
// Calibration state
// ============================================================================
static volatile bool s_calibrating = false;
static volatile bool s_cal_request = false;
static float s_stored_gyro_bias = 0.0f;
static bool s_bias_loaded = false;

// ============================================================================
// NVS helpers for calibration storage
// ============================================================================

static bool nvs_load_gyro_bias(float* bias) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "NVS namespace not found (first boot?)");
        return false;
    }

    uint8_t valid = 0;
    err = nvs_get_u8(handle, NVS_KEY_CAL_VALID, &valid);
    if (err != ESP_OK || valid != 1) {
        nvs_close(handle);
        ESP_LOGI(TAG, "No valid calibration in NVS");
        return false;
    }

    // NVS doesn't support float directly, use blob
    size_t size = sizeof(float);
    err = nvs_get_blob(handle, NVS_KEY_GYRO_BIAS, bias, &size);
    nvs_close(handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Loaded gyro bias from NVS: %.2f dps", *bias);
        return true;
    }
    return false;
}

static bool nvs_save_gyro_bias(float bias) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %d", err);
        return false;
    }

    err = nvs_set_blob(handle, NVS_KEY_GYRO_BIAS, &bias, sizeof(float));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save gyro bias: %d", err);
        nvs_close(handle);
        return false;
    }

    err = nvs_set_u8(handle, NVS_KEY_CAL_VALID, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save cal_valid: %d", err);
        nvs_close(handle);
        return false;
    }

    err = nvs_commit(handle);
    nvs_close(handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Saved gyro bias to NVS: %.2f dps", bias);
        return true;
    }
    return false;
}

// ============================================================================
// Mahony AHRS Algorithm
// Ported from: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
// ============================================================================

class MahonyAHRS {
public:
    // Quaternion of sensor frame relative to auxiliary frame
    float q0, q1, q2, q3;

    MahonyAHRS(float sampleFreq = 100.0f, float kp = 0.5f, float ki = 0.0f)
        : q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f),
          twoKp(2.0f * kp), twoKi(2.0f * ki),
          integralFBx(0.0f), integralFBy(0.0f), integralFBz(0.0f),
          invSampleFreq(1.0f / sampleFreq) {}

    void setSampleFreq(float freq) {
        invSampleFreq = 1.0f / freq;
    }

    void setGains(float kp, float ki) {
        twoKp = 2.0f * kp;
        twoKi = 2.0f * ki;
    }

    void reset() {
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    // IMU algorithm update (no magnetometer)
    // gx, gy, gz: gyroscope in rad/s
    // ax, ay, az: accelerometer (any unit, will be normalized)
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
        float recipNorm;
        float halfvx, halfvy, halfvz;
        float halfex, halfey, halfez;
        float qa, qb, qc;

        // Compute feedback only if accelerometer measurement valid
        if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Estimated direction of gravity
            halfvx = q1 * q3 - q0 * q2;
            halfvy = q0 * q1 + q2 * q3;
            halfvz = q0 * q0 - 0.5f + q3 * q3;

            // Error is sum of cross product between estimated and measured direction of gravity
            halfex = (ay * halfvz - az * halfvy);
            halfey = (az * halfvx - ax * halfvz);
            halfez = (ax * halfvy - ay * halfvx);

            // Compute and apply integral feedback if enabled
            if (twoKi > 0.0f) {
                integralFBx += twoKi * halfex * invSampleFreq;
                integralFBy += twoKi * halfey * invSampleFreq;
                integralFBz += twoKi * halfez * invSampleFreq;
                gx += integralFBx;
                gy += integralFBy;
                gz += integralFBz;
            } else {
                integralFBx = 0.0f;
                integralFBy = 0.0f;
                integralFBz = 0.0f;
            }

            // Apply proportional feedback
            gx += twoKp * halfex;
            gy += twoKp * halfey;
            gz += twoKp * halfez;
        }

        // Integrate rate of change of quaternion
        gx *= (0.5f * invSampleFreq);
        gy *= (0.5f * invSampleFreq);
        gz *= (0.5f * invSampleFreq);
        qa = q0;
        qb = q1;
        qc = q2;
        q0 += (-qb * gx - qc * gy - q3 * gz);
        q1 += (qa * gx + qc * gz - q3 * gy);
        q2 += (qa * gy - qb * gz + q3 * gx);
        q3 += (qa * gz + qb * gy - qc * gx);

        // Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }

    // Convert quaternion to Euler angles (ZYX convention) in radians
    void toEuler(float& roll, float& pitch, float& yaw) const {
        // Roll (X-axis rotation)
        float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
        float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
        roll = atan2f(sinr_cosp, cosr_cosp);

        // Pitch (Y-axis rotation)
        float sinp = 2.0f * (q0 * q2 - q3 * q1);
        if (fabsf(sinp) >= 1.0f) {
            pitch = copysignf((float)M_PI / 2.0f, sinp);
        } else {
            pitch = asinf(sinp);
        }

        // Yaw (Z-axis rotation)
        float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
        float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
        yaw = atan2f(siny_cosp, cosy_cosp);
    }

private:
    float twoKp;        // 2 * proportional gain
    float twoKi;        // 2 * integral gain
    float integralFBx, integralFBy, integralFBz;  // integral error terms
    float invSampleFreq;

    // Fast inverse square-root using union for type punning (C++ safe)
    static float invSqrt(float x) {
        union {
            float f;
            int32_t i;
        } conv;
        float halfx = 0.5f * x;
        conv.f = x;
        conv.i = 0x5f3759df - (conv.i >> 1);
        conv.f = conv.f * (1.5f - (halfx * conv.f * conv.f));
        return conv.f;
    }
};

// ============================================================================
// Coordinate system mapping from Kconfig
// ============================================================================

static void mapIMUToVehicle(const qmi8658_data_t& imu, float& vx, float& vy, float& vz, bool is_accel) {
    float ix = is_accel ? imu.accelX : imu.gyroX;
    float iy = is_accel ? imu.accelY : imu.gyroY;
    float iz = is_accel ? imu.accelZ : imu.gyroZ;

    // Map Vehicle X (front)
#if defined(CONFIG_COPILOT_IMU_VEHICLE_X_IS_IMU_X)
    vx = ix;
#elif defined(CONFIG_COPILOT_IMU_VEHICLE_X_IS_IMU_X_NEG)
    vx = -ix;
#elif defined(CONFIG_COPILOT_IMU_VEHICLE_X_IS_IMU_Y)
    vx = iy;
#elif defined(CONFIG_COPILOT_IMU_VEHICLE_X_IS_IMU_Y_NEG)
    vx = -iy;
#elif defined(CONFIG_COPILOT_IMU_VEHICLE_X_IS_IMU_Z)
    vx = iz;
#elif defined(CONFIG_COPILOT_IMU_VEHICLE_X_IS_IMU_Z_NEG)
    vx = -iz;
#else
    vx = iz;  // Default: IMU Z -> Vehicle X
#endif

    // Map Vehicle Y (left)
#if defined(CONFIG_COPILOT_IMU_VEHICLE_Y_IS_IMU_X)
    vy = ix;
#elif defined(CONFIG_COPILOT_IMU_VEHICLE_Y_IS_IMU_X_NEG)
    vy = -ix;
#elif defined(CONFIG_COPILOT_IMU_VEHICLE_Y_IS_IMU_Y)
    vy = iy;
#elif defined(CONFIG_COPILOT_IMU_VEHICLE_Y_IS_IMU_Y_NEG)
    vy = -iy;
#elif defined(CONFIG_COPILOT_IMU_VEHICLE_Y_IS_IMU_Z)
    vy = iz;
#elif defined(CONFIG_COPILOT_IMU_VEHICLE_Y_IS_IMU_Z_NEG)
    vy = -iz;
#else
    vy = -iy;  // Default: -IMU Y -> Vehicle Y
#endif

    // Map Vehicle Z (up)
#if defined(CONFIG_COPILOT_IMU_VEHICLE_Z_IS_IMU_X)
    vz = ix;
#elif defined(CONFIG_COPILOT_IMU_VEHICLE_Z_IS_IMU_X_NEG)
    vz = -ix;
#elif defined(CONFIG_COPILOT_IMU_VEHICLE_Z_IS_IMU_Y)
    vz = iy;
#elif defined(CONFIG_COPILOT_IMU_VEHICLE_Z_IS_IMU_Y_NEG)
    vz = -iy;
#elif defined(CONFIG_COPILOT_IMU_VEHICLE_Z_IS_IMU_Z)
    vz = iz;
#elif defined(CONFIG_COPILOT_IMU_VEHICLE_Z_IS_IMU_Z_NEG)
    vz = -iz;
#else
    vz = ix;  // Default: IMU X -> Vehicle Z
#endif
}

// ============================================================================
// IMU driver and state
// ============================================================================

static qmi8658_dev_t s_qmi = {};
static bool s_imu_ready = false;
static TaskHandle_t s_imu_task = nullptr;

// Mahony AHRS filter
static MahonyAHRS* s_ahrs = nullptr;
static volatile bool s_reset_requested = false;

// Button interrupt handler
#if CONFIG_COPILOT_IMU_RESET_GPIO >= 0
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    s_reset_requested = true;
}
#endif

static bool copilot_imu_hw_init(void) {
    // Get existing I2C bus handle from BSP
    i2c_master_bus_handle_t i2c_bus = bsp_i2c_get_handle();
    if (i2c_bus == nullptr) {
        ESP_LOGE(TAG, "Failed to get I2C bus handle from BSP");
        return false;
    }

    ESP_LOGI(TAG, "Using shared I2C bus from BSP");

    // Initialize QMI8658 sensor
    esp_err_t ret = qmi8658_init(&s_qmi, i2c_bus, CONFIG_COPILOT_IMU_I2C_ADDR);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "QMI8658 init failed: %s", esp_err_to_name(ret));
        return false;
    }

    // Configure accelerometer: 4G range, 125Hz ODR
    qmi8658_set_accel_range(&s_qmi, QMI8658_ACCEL_RANGE_4G);
    qmi8658_set_accel_odr(&s_qmi, QMI8658_ACCEL_ODR_125HZ);

    // Configure gyroscope: 512 DPS range, 125Hz ODR (increased range for better tracking)
    qmi8658_set_gyro_range(&s_qmi, QMI8658_GYRO_RANGE_512DPS);
    qmi8658_set_gyro_odr(&s_qmi, QMI8658_GYRO_ODR_125HZ);

    // Enable both sensors
    qmi8658_enable_sensors(&s_qmi, QMI8658_ENABLE_ACCEL | QMI8658_ENABLE_GYRO);

    ESP_LOGI(TAG, "QMI8658 configured: ACC 4G@125Hz, GYRO 512DPS@125Hz, addr=0x%02X",
             CONFIG_COPILOT_IMU_I2C_ADDR);

    // Setup reset button
#if CONFIG_COPILOT_IMU_RESET_GPIO >= 0
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << CONFIG_COPILOT_IMU_RESET_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;  // Trigger on button press (active low)
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)CONFIG_COPILOT_IMU_RESET_GPIO, gpio_isr_handler, nullptr);

    ESP_LOGI(TAG, "Reset button configured on GPIO%d", CONFIG_COPILOT_IMU_RESET_GPIO);
#endif

    return true;
}

/**
 * IMU task with Mahony AHRS for quaternion estimation
 *
 * Animation behavior (in vehicle frame):
 *   - Forward acceleration → face drifts UP (positive ay)
 *   - Backward acceleration → face drifts DOWN (negative ay)
 *   - Right turn (or right accel) → face drifts RIGHT (positive ax)
 *   - Left turn (or left accel) → face drifts LEFT (negative ax)
 *   - Yaw rotation → face rotates
 */
static void copilot_imu_task(void *arg) {
    (void)arg;

    // Calculate sample frequency from update interval
    const float sampleFreq = 1000.0f / (float)CONFIG_COPILOT_IMU_UPDATE_INTERVAL_MS;

    // Mahony AHRS gains:
    // Kp = proportional gain (higher = faster convergence, more noise)
    // Ki = integral gain (counters gyro bias drift)
    const float kp = 1.0f;   // Proportional gain
    const float ki = 0.05f;  // Small integral gain for bias correction

    s_ahrs = new MahonyAHRS(sampleFreq, kp, ki);

    ESP_LOGI(TAG, "IMU task started: Mahony AHRS, freq=%.1fHz, Kp=%.2f, Ki=%.2f",
             sampleFreq, kp, ki);

    for (;;) {
        // Check for reset request
        if (s_reset_requested) {
            s_reset_requested = false;
            s_ahrs->reset();
            ESP_LOGI(TAG, "AHRS reset to identity");
        }

        bool ready = false;
        if (qmi8658_is_data_ready(&s_qmi, &ready) != ESP_OK || !ready) {
            vTaskDelay(pdMS_TO_TICKS(CONFIG_COPILOT_IMU_UPDATE_INTERVAL_MS));
            continue;
        }

        qmi8658_data_t imu_data = {};
        if (qmi8658_read_sensor_data(&s_qmi, &imu_data) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(CONFIG_COPILOT_IMU_UPDATE_INTERVAL_MS));
            continue;
        }

        // Map IMU data to vehicle coordinate frame
        // QMI8658 outputs: acceleration in mg, gyroscope in dps
        float accel_x, accel_y, accel_z;  // Vehicle frame, mg (milli-g)
        float gyro_x, gyro_y, gyro_z;     // Vehicle frame, dps (degrees/second)

        mapIMUToVehicle(imu_data, accel_x, accel_y, accel_z, true);
        mapIMUToVehicle(imu_data, gyro_x, gyro_y, gyro_z, false);

        // ====================================================================
        // Calculate animation values
        // ====================================================================

        float ax, ay, yaw_anim;
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

#if CONFIG_COPILOT_IMU_FREEZE_QUATERNION
        // ================================================================
        // Freeze Quaternion Mode: Use raw accel/gyro directly
        // ================================================================
        //
        // COORDINATE SYSTEMS:
        //
        // 1. IMU Sensor Frame (QMI8658 datasheet):
        //    - Converted to Vehicle frame by mapIMUToVehicle() above
        //
        // 2. Vehicle Frame (SAE J670, used for ALL processing):
        //    - X = forward (positive toward front of vehicle)
        //    - Y = left (positive toward driver's left)
        //    - Z = up (positive toward sky)
        //    - Yaw = rotation around Z (positive = CCW from above = turn left)
        //    - Pitch = rotation around Y (positive = nose up)
        //    - Roll = rotation around X (positive = right side down)
        //
        // 3. Screen Frame (output only, converted at the end):
        //    - ax positive → face shifts RIGHT
        //    - ay positive → face shifts DOWN
        //
        // VEHICLE → SCREEN MAPPING (applied at output):
        //    - Turn right (negative yaw rate) → face shifts left (negative ax)
        //    - Turn left (positive yaw rate) → face shifts right (positive ax)
        //    - Accelerate forward (positive accel_x) → face lags back = down (positive ay)
        //    - Decelerate/brake (negative accel_x) → face moves forward = up (negative ay)
        //
        // ================================================================

        // ---- Vehicle frame inputs (from mapIMUToVehicle) ----
        // NOTE: Based on actual IMU installation observation:
        //   - Movement along physical Y axis affects up/down animation
        //   - Rotation around physical Y axis affects left/right animation
        // This suggests IMU Y axis is oriented as vehicle's vertical (up) axis.
        //
        // Observed behavior mapping:
        //   - Y+ movement → face down → need positive accel
        //   - Y-axis CW rotation → face left → need negative yaw (turn right)
        //
        // Sign adjustments based on mapIMUToVehicle default (vehicle_y = -imu_y):
        //   - veh_fwd_accel = -accel_y to get positive when Y+ movement
        //   - veh_yaw_rate = -gyro_y to get negative when CW rotation
        float veh_yaw_rate = -gyro_y;         // dps, Y-axis rotation: CW→negative (turn right)
        float veh_fwd_accel = -accel_y / 1000.0f;  // Convert mg to g, Y+→positive

        // ---- Accelerometer DC blocking (high-pass filter) ----
        // Remove static gravity component, respond only to dynamic acceleration
        // Uses a slow-adapting baseline that tracks the average (gravity) value
        static float accel_baseline = 0.0f;
        static bool accel_baseline_init = false;

        if (!accel_baseline_init) {
            // Initialize baseline to first reading
            accel_baseline = veh_fwd_accel;
            accel_baseline_init = true;
        }

        // Slow adaptation to track gravity (time constant ~10 seconds)
        // alpha = dt / (tau + dt), where tau = 10s, dt = 25ms → alpha ≈ 0.0025
        const float baseline_alpha = 0.003f;
        accel_baseline = accel_baseline + baseline_alpha * (veh_fwd_accel - accel_baseline);

        // Dynamic acceleration = raw - baseline (removes gravity)
        float veh_fwd_accel_filtered = veh_fwd_accel - accel_baseline;

        // ---- Gyroscope bias compensation (NVS-stored calibration) ----
        // Calibration state machine
        static float cal_accumulator = 0.0f;
        static uint32_t cal_sample_count = 0;
        static const uint32_t CAL_SAMPLES = 120;  // ~3 seconds at 25ms

        // Check for calibration request
        if (s_cal_request && !s_calibrating) {
            s_calibrating = true;
            s_cal_request = false;
            cal_accumulator = 0.0f;
            cal_sample_count = 0;
            ESP_LOGI(TAG, "Starting gyro calibration... Keep device STATIONARY!");
        }

        // Calibration in progress
        if (s_calibrating) {
            cal_accumulator += veh_yaw_rate;
            cal_sample_count++;

            if (cal_sample_count >= CAL_SAMPLES) {
                // Calibration complete
                float new_bias = cal_accumulator / (float)CAL_SAMPLES;
                s_stored_gyro_bias = new_bias;
                s_bias_loaded = true;
                s_calibrating = false;

                // Save to NVS
                if (nvs_save_gyro_bias(new_bias)) {
                    ESP_LOGI(TAG, "Calibration complete! Bias=%.2f dps saved to NVS", new_bias);
                } else {
                    ESP_LOGW(TAG, "Calibration complete (%.2f dps) but NVS save failed", new_bias);
                }
            } else if (cal_sample_count % 40 == 0) {
                // Progress update every ~1 second
                ESP_LOGI(TAG, "Calibrating... %lu/%lu samples",
                         (unsigned long)cal_sample_count, (unsigned long)CAL_SAMPLES);
            }
        }

        // Use stored bias (from NVS or calibration)
        float gyro_bias = s_bias_loaded ? s_stored_gyro_bias : 0.0f;

        // Subtract bias from raw reading (still in vehicle frame)
        float veh_yaw_rate_corrected = veh_yaw_rate - gyro_bias;

        // ---- Schmitt trigger (hysteresis deadzone) ----
        // Prevents jitter at low values, face stays centered until significant motion
        // All processing uses vehicle frame coordinates
        static bool active_yaw = false;
        static bool active_accel = false;

        // Higher thresholds for yaw (gyros are noisier than accelerometers)
        const float thresh_high_yaw = 12.0f;  // dps to activate
        const float thresh_low_yaw = 6.0f;    // dps to deactivate
        const float thresh_high_acc = 0.08f;  // g to activate
        const float thresh_low_acc = 0.04f;   // g to deactivate

        float abs_yaw = fabsf(veh_yaw_rate_corrected);
        float abs_acc = fabsf(veh_fwd_accel_filtered);

        // Schmitt trigger state transitions
        if (!active_yaw && abs_yaw > thresh_high_yaw) active_yaw = true;
        if (active_yaw && abs_yaw < thresh_low_yaw) active_yaw = false;
        if (!active_accel && abs_acc > thresh_high_acc) active_accel = true;
        if (active_accel && abs_acc < thresh_low_acc) active_accel = false;

        // Apply deadzone: zero output when inactive (still in vehicle frame)
        float veh_yaw_gated = active_yaw ? veh_yaw_rate_corrected : 0.0f;
        float veh_accel_gated = active_accel ? veh_fwd_accel_filtered : 0.0f;

        // ---- Normalize to animation range (vehicle frame) ----
        // Yaw rate: 30 dps = full deflection
        // Acceleration: 0.3g = full deflection
        float norm_yaw = veh_yaw_gated / 30.0f;     // Normalized yaw rate [-1, 1]
        float norm_accel = veh_accel_gated / 0.3f;  // Normalized acceleration [-1, 1]

        // ---- Bounded dynamic auto-gain with sliding window ----
        static float peak_yaw = 0.3f;
        static float peak_accel = 0.3f;

        // Peak bounds (prevents over/under amplification)
        const float peak_min = 0.2f;   // Minimum peak (max sensitivity limit)
        const float peak_max = 2.0f;   // Maximum peak (min sensitivity limit)

        // Peak decay (adaptive window)
        const float decay_rate = 0.997f;
        peak_yaw *= decay_rate;
        peak_accel *= decay_rate;

        // Update peaks
        float abs_norm_yaw = fabsf(norm_yaw);
        float abs_norm_accel = fabsf(norm_accel);
        if (abs_norm_yaw > peak_yaw) peak_yaw = abs_norm_yaw;
        if (abs_norm_accel > peak_accel) peak_accel = abs_norm_accel;

        // Clamp peaks to bounds
        peak_yaw = fmaxf(peak_min, fminf(peak_max, peak_yaw));
        peak_accel = fmaxf(peak_min, fminf(peak_max, peak_accel));

        // ---- Sensitivity and gain from Kconfig ----
#if defined(CONFIG_COPILOT_MOTION_SENSITIVITY_LOW)
        const float base_sens = 1.5f;
        const float gain_min = 1.0f;
        const float gain_max = 10.0f;
#elif defined(CONFIG_COPILOT_MOTION_SENSITIVITY_MEDIUM)
        const float base_sens = 1.0f;
        const float gain_min = 0.8f;
        const float gain_max = 6.0f;
#else
        const float base_sens = 0.6f;
        const float gain_min = 0.5f;
        const float gain_max = 3.0f;
#endif

        // Calculate bounded gain
        float gain_yaw = fmaxf(gain_min, fminf(gain_max, 1.0f / peak_yaw));
        float gain_accel = fmaxf(gain_min, fminf(gain_max, 1.0f / peak_accel));

        // Target animation values (still in vehicle frame, normalized)
        float target_yaw = norm_yaw * base_sens * gain_yaw;
        float target_accel = norm_accel * base_sens * gain_accel;

        // Clamp targets
        target_yaw = fmaxf(-1.0f, fminf(1.0f, target_yaw));
        target_accel = fmaxf(-1.0f, fminf(1.0f, target_accel));

        // ---- Smoothing (fast attack, fast release for responsive animation) ----
        static float smooth_yaw = 0.0f;
        static float smooth_accel = 0.0f;

        // Fast response in both directions - animation follows motion closely
        const float alpha_attack = 0.4f;   // Fast response to new motion
        const float alpha_release = 0.5f;  // Fast return to center (no inertia lag)

        float alpha_yaw = (fabsf(target_yaw) > fabsf(smooth_yaw)) ? alpha_attack : alpha_release;
        float alpha_accel = (fabsf(target_accel) > fabsf(smooth_accel)) ? alpha_attack : alpha_release;

        smooth_yaw = smooth_yaw + alpha_yaw * (target_yaw - smooth_yaw);
        smooth_accel = smooth_accel + alpha_accel * (target_accel - smooth_accel);

        // Snap to zero when inactive (immediate return to center)
        if (!active_yaw) smooth_yaw *= 0.7f;      // Faster decay when inactive
        if (!active_accel) smooth_accel *= 0.7f;
        if (fabsf(smooth_yaw) < 0.02f) smooth_yaw = 0.0f;
        if (fabsf(smooth_accel) < 0.02f) smooth_accel = 0.0f;

        // ================================================================
        // VEHICLE → SCREEN COORDINATE TRANSFORMATION (output stage)
        // ================================================================
        // Vehicle frame (SAE):
        //   - Yaw positive = turn left (CCW from above)
        //   - Accel positive = forward
        //
        // Screen frame:
        //   - ax positive = face shifts RIGHT
        //   - ay positive = face shifts DOWN
        //
        // Transformation:
        //   - Turn left (yaw+) → face shifts RIGHT (ax+) → ax = +yaw
        //   - Accelerate forward (accel+) → inertia → face lags DOWN (ay+) → ay = +accel
        ax = smooth_yaw;    // Turn left → face right
        ay = smooth_accel;  // Accelerate forward → face down (inertia)
        yaw_anim = 0.0f;    // Not used in freeze mode

        // Debug logging
        static uint32_t last_debug_ms = 0;
        if (now_ms - last_debug_ms >= 500) {
            last_debug_ms = now_ms;
            // Show coordinate system data flow (using Y-axis based on IMU installation)
            ESP_LOGI(TAG, "IMU sensor: gy=%.1f (dps) ay=%.1f (mg)",
                     imu_data.gyroY, imu_data.accelY);
            ESP_LOGI(TAG, "Yaw(Y-rot): raw=%.1f -> bias=%.1f -> %.1fdps [%s]",
                     veh_yaw_rate, gyro_bias, veh_yaw_rate_corrected,
                     s_calibrating ? "CAL" : (s_bias_loaded ? "OK" : "NO_CAL"));
            ESP_LOGI(TAG, "Accel(Y): raw=%.2fg base=%.2fg -> %.3fg [%c]",
                     veh_fwd_accel, accel_baseline, veh_fwd_accel_filtered,
                     active_accel ? 'A' : '_');
            int16_t shift_x = (int16_t)(ax * CONFIG_COPILOT_MOTION_MAX_SHIFT_PX);
            int16_t shift_y = (int16_t)(ay * CONFIG_COPILOT_MOTION_MAX_SHIFT_PX);
            ESP_LOGI(TAG, "Screen: yaw[%c]->ax accel[%c]->ay -> shift(%d,%d)px",
                     active_yaw ? 'A' : '_', active_accel ? 'A' : '_', shift_x, shift_y);
        }

#else  // !CONFIG_COPILOT_IMU_FREEZE_QUATERNION
        // ================================================================
        // Quaternion Mode: Use Mahony AHRS for orientation
        // ================================================================

        // Convert gyroscope from dps to rad/s for AHRS
        const float DEG_TO_RAD = 0.01745329f;
        float gx = gyro_x * DEG_TO_RAD;
        float gy = gyro_y * DEG_TO_RAD;
        float gz = gyro_z * DEG_TO_RAD;

        // Update Mahony AHRS
        s_ahrs->updateIMU(gx, gy, gz, accel_x, accel_y, accel_z);

        // Extract Euler angles
        float roll, pitch, yaw;
        s_ahrs->toEuler(roll, pitch, yaw);

        // Orientation-based animation
        float tilt_lateral = sinf(roll);
        float tilt_forward = sinf(pitch);
        float accel_lateral = -accel_y / 1000.0f;
        float accel_forward = accel_x / 1000.0f;

        // Combine tilt and acceleration
        float raw_ax = tilt_lateral + accel_lateral * 0.5f;
        float raw_ay = tilt_forward + accel_forward * 0.5f;
        yaw_anim = yaw * 57.2957795f;

        // Dynamic auto-gain
        static float peak_ax = 0.1f;
        static float peak_ay = 0.1f;

        const float decay_rate = 0.995f;
        peak_ax *= decay_rate;
        peak_ay *= decay_rate;

        float abs_ax = fabsf(raw_ax);
        float abs_ay = fabsf(raw_ay);
        if (abs_ax > peak_ax) peak_ax = abs_ax;
        if (abs_ay > peak_ay) peak_ay = abs_ay;

        if (peak_ax < 0.05f) peak_ax = 0.05f;
        if (peak_ay < 0.05f) peak_ay = 0.05f;

#if defined(CONFIG_COPILOT_MOTION_SENSITIVITY_LOW)
        const float base_sens = 1.5f;
        const float max_gain = 20.0f;
#elif defined(CONFIG_COPILOT_MOTION_SENSITIVITY_MEDIUM)
        const float base_sens = 1.0f;
        const float max_gain = 10.0f;
#else
        const float base_sens = 0.7f;
        const float max_gain = 5.0f;
#endif

        float gain_x = fminf(1.0f / peak_ax, max_gain);
        float gain_y = fminf(1.0f / peak_ay, max_gain);

        ax = raw_ax * base_sens * gain_x;
        ay = raw_ay * base_sens * gain_y;

        ax = fmaxf(-1.0f, fminf(1.0f, ax));
        ay = fmaxf(-1.0f, fminf(1.0f, ay));
        yaw_anim = fmaxf(-45.0f, fminf(45.0f, yaw_anim));

        // Debug logging
        static uint32_t last_debug_ms = 0;
        if (now_ms - last_debug_ms >= 500) {
            last_debug_ms = now_ms;
            ESP_LOGI(TAG, "IMU: acc(%.0f,%.0f,%.0f) gyro(%.0f,%.0f,%.0f)",
                     imu_data.accelX, imu_data.accelY, imu_data.accelZ,
                     imu_data.gyroX, imu_data.gyroY, imu_data.gyroZ);
            ESP_LOGI(TAG, "Euler: roll=%.1f pitch=%.1f yaw=%.1f",
                     roll * 57.2957795f, pitch * 57.2957795f, yaw_anim);
            ESP_LOGI(TAG, "Raw: tilt(%.2f,%.2f) accel(%.2f,%.2f)",
                     tilt_lateral, tilt_forward, accel_lateral, accel_forward);
            int16_t shift_x = (int16_t)(ax * CONFIG_COPILOT_MOTION_MAX_SHIFT_PX);
            int16_t shift_y = (int16_t)(ay * CONFIG_COPILOT_MOTION_MAX_SHIFT_PX);
            ESP_LOGI(TAG, "Output: ax=%.2f ay=%.2f -> shift(%d,%d)px",
                     ax, ay, shift_x, shift_y);
        }
#endif  // CONFIG_COPILOT_IMU_FREEZE_QUATERNION

        // ====================================================================
        // Update UI (motion only, no expression trigger)
        // ====================================================================

        copilot_motion_t motion = {};
        motion.ax = FP_FROM_FLOAT(ax);
        motion.ay = FP_FROM_FLOAT(ay);
        motion.yaw_deg = FP_FROM_FLOAT(yaw_anim);
        motion.speed = FP_ONE;

        // Use motion-only function to avoid triggering uneasy expression
        copilot_ui_set_motion_only_async(&motion);

        vTaskDelay(pdMS_TO_TICKS(CONFIG_COPILOT_IMU_UPDATE_INTERVAL_MS));
    }
}

void copilot_imu_init(void) {
#if CONFIG_COPILOT_IMU_FREEZE_QUATERNION
    ESP_LOGI(TAG, "Initializing QMI8658 IMU (Freeze Quaternion mode)");
    ESP_LOGI(TAG, "  Yaw rate -> left/right shift, Fwd accel -> up/down shift");

    // Load gyro bias calibration from NVS
    if (nvs_load_gyro_bias(&s_stored_gyro_bias)) {
        s_bias_loaded = true;
        ESP_LOGI(TAG, "  Using stored gyro bias: %.2f dps", s_stored_gyro_bias);
    } else {
        ESP_LOGW(TAG, "  No gyro calibration found. Run calibration for accurate centering.");
    }
#else
    ESP_LOGI(TAG, "Initializing QMI8658 IMU with Mahony AHRS");
#endif

    if (!copilot_imu_hw_init()) {
        ESP_LOGE(TAG, "IMU hardware init failed");
        return;
    }

    // Create IMU reading task
    BaseType_t ret = xTaskCreate(
        copilot_imu_task,
        "copilot_imu",
        4096,  // Stack for AHRS calculations
        nullptr,
        2,
        &s_imu_task
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMU task");
        return;
    }

    s_imu_ready = true;
    ESP_LOGI(TAG, "QMI8658 IMU initialized successfully with Mahony AHRS");
}

bool copilot_imu_is_ready(void) {
    return s_imu_ready;
}

void copilot_imu_reset_quaternion(void) {
    s_reset_requested = true;
}

void copilot_imu_get_quaternion(float* qw, float* qx, float* qy, float* qz) {
    if (s_ahrs) {
        if (qw) *qw = s_ahrs->q0;
        if (qx) *qx = s_ahrs->q1;
        if (qy) *qy = s_ahrs->q2;
        if (qz) *qz = s_ahrs->q3;
    } else {
        if (qw) *qw = 1.0f;
        if (qx) *qx = 0.0f;
        if (qy) *qy = 0.0f;
        if (qz) *qz = 0.0f;
    }
}

bool copilot_imu_start_calibration(void) {
    if (!s_imu_ready) {
        ESP_LOGW(TAG, "Cannot start calibration: IMU not ready");
        return false;
    }
    if (s_calibrating) {
        ESP_LOGW(TAG, "Calibration already in progress");
        return false;
    }

    ESP_LOGI(TAG, "Gyro calibration requested. Keep device STATIONARY!");
    s_cal_request = true;
    return true;
}

bool copilot_imu_is_calibrating(void) {
    return s_calibrating;
}

float copilot_imu_get_gyro_bias(void) {
    return s_bias_loaded ? s_stored_gyro_bias : 0.0f;
}

#else  // CONFIG_COPILOT_MOTION_SOURCE_INTERNAL not defined

void copilot_imu_init(void) {
    // No-op when internal IMU not enabled
}

bool copilot_imu_is_ready(void) {
    return false;
}

void copilot_imu_reset_quaternion(void) {
    // No-op
}

void copilot_imu_get_quaternion(float* qw, float* qx, float* qy, float* qz) {
    if (qw) *qw = 1.0f;
    if (qx) *qx = 0.0f;
    if (qy) *qy = 0.0f;
    if (qz) *qz = 0.0f;
}

bool copilot_imu_start_calibration(void) {
    return false;
}

bool copilot_imu_is_calibrating(void) {
    return false;
}

float copilot_imu_get_gyro_bias(void) {
    return 0.0f;
}

#endif  // CONFIG_COPILOT_MOTION_SOURCE_INTERNAL
