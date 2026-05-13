#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Servo calibration parameters
typedef struct {
    float angle_min;     // mechanical min angle (degrees from center, < 0, e.g. -90° for 180° servo)
    float angle_max;     // mechanical max angle (degrees from center, > 0, e.g. +90° for 180° servo)
    float soft_limit_min; // software target clamp min
    float soft_limit_max; // software target clamp max
    uint16_t pulse_min_us; // pulse width at angle_min (e.g. 500)
    uint16_t pulse_max_us; // pulse width at angle_max (e.g. 2500)
    uint16_t pulse_center_us; // pulse width at 0 degrees (e.g. 1500)
} copilot_servo_ch_calib_t;

typedef struct {
    copilot_servo_ch_calib_t pitch;
    copilot_servo_ch_calib_t yaw;
} copilot_servo_calib_t;

// Init hardware (LEDC on GPIO17/18) and start servo task
void copilot_servo_init(void);

// Set target angles in degrees (front-left-up frame):
//   pitch > 0 -> head up,   servo CCW
//   yaw   > 0 -> head left, servo CCW
// Called from any thread; servo task reads with atomic/volatile.
void copilot_servo_set_target(float pitch_deg, float yaw_deg);

// Calibration mode: servos hold center position, target updates ignored
void copilot_servo_set_calibration(bool enable);
bool copilot_servo_get_calibration(void);

// Runtime calibration overrides (non-persistent)
void copilot_servo_set_calib(const copilot_servo_calib_t *calib);
void copilot_servo_get_calib(copilot_servo_calib_t *calib);
void copilot_servo_reset_calib_to_defaults(void);

// Get raw current pulse widths (for debug/status)
void copilot_servo_get_pulse_us(uint16_t *pitch_us, uint16_t *yaw_us);

// Manual override: when true, UI animation won't drive servo targets.
// MQTT servo commands implicitly enable manual mode; calibration disables it.
void copilot_servo_set_manual(bool enable);
bool copilot_servo_get_manual(void);

// Get smoothed current angles
void copilot_servo_get_current_angle(float *pitch_deg, float *yaw_deg);

#ifdef __cplusplus
}
#endif
