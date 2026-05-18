#ifndef COPILOT_UI_H
#define COPILOT_UI_H

#include <stdbool.h>
#include <stdint.h>
#include "lvgl.h"
#include "copilot_face_data.h"

#ifdef __cplusplus
extern "C" {
#endif

// Fixed-point Q8.8 format for ESP32-C6 (no FPU)
// Range: -128.0 to +127.996, resolution: 1/256 = 0.00390625
#define FP_SHIFT 8
#define FP_ONE   (1 << FP_SHIFT)  // 256 = 1.0
#define FP_HALF  (FP_ONE >> 1)    // 128 = 0.5
#define FP_FROM_FLOAT(f) ((int16_t)((f) * FP_ONE))
#define FP_TO_INT(fp) ((fp) >> FP_SHIFT)
#define FP_MUL(a, b) (((int32_t)(a) * (b)) >> FP_SHIFT)

typedef struct {
    int16_t ax;      // Q8.8 fixed-point, range -1.0 to +1.0
    int16_t ay;      // Q8.8 fixed-point
    int16_t yaw_deg; // Q8.8 fixed-point, degrees
    int16_t speed;   // Q8.8 fixed-point
} copilot_motion_t;

typedef enum {
    COPILOT_SCREEN_STATE_NEUTRAL_IDLE = 0,
    COPILOT_SCREEN_STATE_PRE_MESSAGE_ORIENT,
    COPILOT_SCREEN_STATE_SPEAKING,
    COPILOT_SCREEN_STATE_RETURN_NEUTRAL,
    COPILOT_SCREEN_STATE_SILENT_NEUTRAL,
    COPILOT_SCREEN_STATE_DEBUG,
} copilot_screen_state_t;

typedef enum {
    COPILOT_SCREEN_ORIENT_FRONT = 0,
    COPILOT_SCREEN_ORIENT_LEFT,
    COPILOT_SCREEN_ORIENT_RIGHT,
    COPILOT_SCREEN_ORIENT_CENTER = COPILOT_SCREEN_ORIENT_FRONT,
} copilot_screen_orientation_t;

typedef struct {
    copilot_screen_state_t state;
    copilot_screen_orientation_t orientation;
    uint32_t duration_ms;
    const char *message_id;
    bool calibration_content_present;
    bool visual_semantic_content_present;
} copilot_screen_event_t;

void copilot_ui_init(lv_obj_t *root);
bool copilot_ui_is_ready(void);

void copilot_ui_set_screen_event(const copilot_screen_event_t *event);
void copilot_ui_set_expression(copilot_expr_t expr, uint32_t duration_ms);
void copilot_ui_set_motion(const copilot_motion_t *motion);
void copilot_ui_ring_show(bool on);
void copilot_ui_on_touch(uint16_t x, uint16_t y);

void copilot_ui_set_screen_event_async(const copilot_screen_event_t *event);
void copilot_ui_set_expression_async(copilot_expr_t expr, uint32_t duration_ms);
void copilot_ui_set_motion_async(const copilot_motion_t *motion);
void copilot_ui_set_motion_only_async(const copilot_motion_t *motion);  // Motion only, no expression trigger
void copilot_ui_ring_show_async(bool on);

#ifdef __cplusplus
}
#endif

#endif
