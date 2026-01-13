#ifndef COPILOT_IMU_H
#define COPILOT_IMU_H

#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize IMU driver and start reading task
 *
 * Only active when CONFIG_COPILOT_MOTION_SOURCE_INTERNAL is enabled.
 * Reads IMU data at CONFIG_COPILOT_IMU_UPDATE_INTERVAL_MS interval
 * and updates face motion accordingly.
 */
void copilot_imu_init(void);

/**
 * @brief Check if IMU is available and initialized
 * @return true if IMU is ready
 */
bool copilot_imu_is_ready(void);

/**
 * @brief Reset quaternion to identity (zero orientation)
 *
 * Call this when the device is at rest to reset gyro drift.
 * Can be triggered by pressing the reset button (CONFIG_COPILOT_IMU_RESET_GPIO).
 */
void copilot_imu_reset_quaternion(void);

/**
 * @brief Get current quaternion orientation
 * @param qw Pointer to store W component (can be NULL)
 * @param qx Pointer to store X component (can be NULL)
 * @param qy Pointer to store Y component (can be NULL)
 * @param qz Pointer to store Z component (can be NULL)
 */
void copilot_imu_get_quaternion(float* qw, float* qx, float* qy, float* qz);

/**
 * @brief Start gyroscope zero-bias calibration
 *
 * The device MUST be stationary during calibration.
 * Calibration takes about 3 seconds, collecting samples and computing bias.
 * Result is stored in NVS and loaded automatically on next boot.
 *
 * @return true if calibration started, false if already in progress or IMU not ready
 */
bool copilot_imu_start_calibration(void);

/**
 * @brief Check if calibration is in progress
 * @return true if calibration is running
 */
bool copilot_imu_is_calibrating(void);

/**
 * @brief Get current gyro bias value
 * @return Gyroscope Z-axis bias in dps
 */
float copilot_imu_get_gyro_bias(void);

#ifdef __cplusplus
}
#endif

#endif
