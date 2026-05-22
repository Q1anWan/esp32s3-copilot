#include "copilot_servo.h"

#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "servo";

// GPIO pins
#define SERVO_PITCH_GPIO CONFIG_COPILOT_SERVO_PITCH_GPIO
#define SERVO_YAW_GPIO   CONFIG_COPILOT_SERVO_YAW_GPIO

// LEDC config
#define SERVO_LEDC_MODE   LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_TIMER  LEDC_TIMER_0
#define SERVO_PITCH_CH    LEDC_CHANNEL_2
#define SERVO_YAW_CH      LEDC_CHANNEL_3
#define SERVO_FREQ_HZ     50
#define SERVO_DUTY_RES    LEDC_TIMER_14_BIT
#define SERVO_DUTY_MAX    ((1 << 14) - 1)  // 16383
#define SERVO_PERIOD_US   20000

// Smoothing
#define SERVO_SMOOTH_ALPHA 0.30f
#define SERVO_TASK_PERIOD_MS 20
#define SERVO_START_DELAY_MS 5000
#define SERVO_TASK_STACK  4096
#define SERVO_TASK_PRIO   (CONFIG_COPILOT_SERVO_TASK_PRIORITY)
#define SERVO_TASK_CORE   (CONFIG_COPILOT_SERVO_TASK_CORE)

// Default calibration — 180° servo (500us=-90°, 1500us=0°, 2500us=+90°)
// Mechanical range (±90°) is separate from soft limits (±8°/±25°)
#define DEF_MECH_ANGLE_MIN  -90.0f
#define DEF_MECH_ANGLE_MAX   90.0f
#define DEF_PITCH_SOFT_MIN  -15.0f
#define DEF_PITCH_SOFT_MAX   15.0f
#define DEF_YAW_SOFT_MIN    -45.0f
#define DEF_YAW_SOFT_MAX     45.0f
#define DEF_PULSE_MIN   500   // -90° mechanical
#define DEF_PULSE_MAX   2500  // +90° mechanical
#define DEF_PULSE_CTR   1500  //   0° mechanical

static copilot_servo_calib_t s_calib;
static volatile float  s_target_pitch;
static volatile float  s_target_yaw;
static volatile bool   s_calibration_mode;
static volatile bool   s_manual;
static volatile bool   s_running;

static float s_cur_pitch;
static float s_cur_yaw;
static uint16_t s_cur_pitch_us;
static uint16_t s_cur_yaw_us;
static StaticTask_t s_servo_task_tcb;
static StackType_t s_servo_task_stack[SERVO_TASK_STACK];

// Convert angle (degrees from center) to pulse width (microseconds)
static uint16_t angle_to_pulse(const copilot_servo_ch_calib_t *ch, float angle_deg) {
    if (angle_deg <= ch->angle_min) return ch->pulse_min_us;
    if (angle_deg >= ch->angle_max) return ch->pulse_max_us;
    float t = (angle_deg - ch->angle_min) / (ch->angle_max - ch->angle_min);
    return (uint16_t)((float)ch->pulse_min_us +
                      t * (float)(ch->pulse_max_us - ch->pulse_min_us));
}

// Convert pulse width to LEDC duty
static uint32_t pulse_to_duty(uint16_t pulse_us) {
    if (pulse_us > SERVO_PERIOD_US) pulse_us = SERVO_PERIOD_US;
    return (uint32_t)pulse_us * (SERVO_DUTY_MAX + 1) / SERVO_PERIOD_US;
}

static void update_pwm(uint16_t pitch_us, uint16_t yaw_us) {
    ledc_set_duty(SERVO_LEDC_MODE, SERVO_PITCH_CH, pulse_to_duty(pitch_us));
    ledc_update_duty(SERVO_LEDC_MODE, SERVO_PITCH_CH);
    ledc_set_duty(SERVO_LEDC_MODE, SERVO_YAW_CH, pulse_to_duty(yaw_us));
    ledc_update_duty(SERVO_LEDC_MODE, SERVO_YAW_CH);
}

static void servo_task(void *arg) {
    (void)arg;
    s_running = true;
    ESP_LOGI(TAG, "Servo task started (pitch=GPIO%d, yaw=GPIO%d, %dHz)",
             SERVO_PITCH_GPIO, SERVO_YAW_GPIO, SERVO_FREQ_HZ);

    s_cur_pitch_us = 0;
    s_cur_yaw_us = 0;
    vTaskDelay(pdMS_TO_TICKS(SERVO_START_DELAY_MS));

    while (true) {
        float target_p = s_target_pitch;
        float target_y = s_target_yaw;
        bool  calib    = s_calibration_mode;

        if (calib) {
            // Hold center
            s_cur_pitch = 0.0f;
            s_cur_yaw   = 0.0f;
            uint16_t ctr_pulse = s_calib.pitch.pulse_center_us;
            if (ctr_pulse == 0) ctr_pulse = DEF_PULSE_CTR;
            s_cur_pitch_us = ctr_pulse;
            s_cur_yaw_us   = s_calib.yaw.pulse_center_us;
            if (s_cur_yaw_us == 0) s_cur_yaw_us = DEF_PULSE_CTR;
            update_pwm(s_cur_pitch_us, s_cur_yaw_us);
        } else {
            // Clamp targets to soft limits
            if (target_p < s_calib.pitch.soft_limit_min) target_p = s_calib.pitch.soft_limit_min;
            if (target_p > s_calib.pitch.soft_limit_max) target_p = s_calib.pitch.soft_limit_max;
            if (target_y < s_calib.yaw.soft_limit_min)   target_y = s_calib.yaw.soft_limit_min;
            if (target_y > s_calib.yaw.soft_limit_max)   target_y = s_calib.yaw.soft_limit_max;

            // Smooth interpolation
            s_cur_pitch += (target_p - s_cur_pitch) * SERVO_SMOOTH_ALPHA;
            s_cur_yaw   += (target_y - s_cur_yaw)   * SERVO_SMOOTH_ALPHA;

            // Convert to pulses
            s_cur_pitch_us = angle_to_pulse(&s_calib.pitch, s_cur_pitch);
            s_cur_yaw_us   = angle_to_pulse(&s_calib.yaw,   s_cur_yaw);
            update_pwm(s_cur_pitch_us, s_cur_yaw_us);
        }

        vTaskDelay(pdMS_TO_TICKS(SERVO_TASK_PERIOD_MS));
    }
}

// --- Public API ---

void copilot_servo_init(void) {
    // Default calibration
    copilot_servo_reset_calib_to_defaults();
    s_target_pitch = 0.0f;
    s_target_yaw   = 0.0f;
    s_calibration_mode = false;

    // Reset GPIOs before LEDC takes over
    gpio_num_t pitch_gpio = (gpio_num_t)SERVO_PITCH_GPIO;
    gpio_num_t yaw_gpio   = (gpio_num_t)SERVO_YAW_GPIO;
    gpio_reset_pin(pitch_gpio);
    gpio_reset_pin(yaw_gpio);
    gpio_set_direction(pitch_gpio, GPIO_MODE_OUTPUT);
    gpio_set_direction(yaw_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(pitch_gpio, 0);
    gpio_set_level(yaw_gpio, 0);

    // LEDC timer
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = SERVO_LEDC_MODE,
        .duty_resolution = SERVO_DUTY_RES,
        .timer_num       = SERVO_LEDC_TIMER,
        .freq_hz         = SERVO_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    // Pitch channel (GPIO17)
    ledc_channel_config_t ch_pitch = {
        .gpio_num   = SERVO_PITCH_GPIO,
        .speed_mode = SERVO_LEDC_MODE,
        .channel    = SERVO_PITCH_CH,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = SERVO_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
        .flags      = { .output_invert = 0 },
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_pitch));

    // Yaw channel (GPIO18)
    ledc_channel_config_t ch_yaw = {
        .gpio_num   = SERVO_YAW_GPIO,
        .speed_mode = SERVO_LEDC_MODE,
        .channel    = SERVO_YAW_CH,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = SERVO_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
        .flags      = { .output_invert = 0 },
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_yaw));

    ESP_LOGI(TAG, "PWM initialized: timer=%d freq=%dHz duty_res=%d bits | "
             "Pitch CH%d GPIO%d | Yaw CH%d GPIO%d | soft-start delay=%dms",
             SERVO_LEDC_TIMER, SERVO_FREQ_HZ, SERVO_DUTY_RES,
             SERVO_PITCH_CH, SERVO_PITCH_GPIO,
             SERVO_YAW_CH, SERVO_YAW_GPIO,
             SERVO_START_DELAY_MS);

    // Start servo task
    TaskHandle_t handle = xTaskCreateStaticPinnedToCore(
        servo_task, "servo", SERVO_TASK_STACK, NULL,
        SERVO_TASK_PRIO, s_servo_task_stack, &s_servo_task_tcb, SERVO_TASK_CORE);
    if (handle == nullptr) {
        ESP_LOGE(TAG, "Failed to create servo task");
    }
}

void copilot_servo_set_target(float pitch_deg, float yaw_deg) {
    s_target_pitch = pitch_deg;
    s_target_yaw   = yaw_deg;
}

void copilot_servo_set_calibration(bool enable) {
    s_calibration_mode = enable;
    if (enable) {
        s_manual = false;  // calibration clears manual override
    }
}

bool copilot_servo_get_calibration(void) {
    return s_calibration_mode;
}

void copilot_servo_set_manual(bool enable) {
    s_manual = enable;
}

bool copilot_servo_get_manual(void) {
    return s_manual;
}

void copilot_servo_set_calib(const copilot_servo_calib_t *calib) {
    if (calib) {
        s_calib = *calib;
    }
}

void copilot_servo_get_calib(copilot_servo_calib_t *calib) {
    if (calib) {
        *calib = s_calib;
    }
}

void copilot_servo_reset_calib_to_defaults(void) {
    // Pitch: mechanical ±90°, soft limits ±8°
    s_calib.pitch.angle_min       = DEF_MECH_ANGLE_MIN;
    s_calib.pitch.angle_max       = DEF_MECH_ANGLE_MAX;
    s_calib.pitch.soft_limit_min  = DEF_PITCH_SOFT_MIN;
    s_calib.pitch.soft_limit_max  = DEF_PITCH_SOFT_MAX;
    s_calib.pitch.pulse_min_us    = DEF_PULSE_MIN;
    s_calib.pitch.pulse_max_us    = DEF_PULSE_MAX;
    s_calib.pitch.pulse_center_us = DEF_PULSE_CTR;
    // Yaw: mechanical ±90°, soft limits ±25°
    s_calib.yaw.angle_min         = DEF_MECH_ANGLE_MIN;
    s_calib.yaw.angle_max         = DEF_MECH_ANGLE_MAX;
    s_calib.yaw.soft_limit_min    = DEF_YAW_SOFT_MIN;
    s_calib.yaw.soft_limit_max    = DEF_YAW_SOFT_MAX;
    s_calib.yaw.pulse_min_us      = DEF_PULSE_MIN;
    s_calib.yaw.pulse_max_us      = DEF_PULSE_MAX;
    s_calib.yaw.pulse_center_us   = DEF_PULSE_CTR;
}

void copilot_servo_get_pulse_us(uint16_t *pitch_us, uint16_t *yaw_us) {
    if (pitch_us) *pitch_us = s_cur_pitch_us;
    if (yaw_us)  *yaw_us   = s_cur_yaw_us;
}

void copilot_servo_get_current_angle(float *pitch_deg, float *yaw_deg) {
    if (pitch_deg) *pitch_deg = s_cur_pitch;
    if (yaw_deg)  *yaw_deg   = s_cur_yaw;
}
