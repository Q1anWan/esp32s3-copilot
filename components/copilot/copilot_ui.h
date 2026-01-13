#ifndef COPILOT_UI_H
#define COPILOT_UI_H

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

void copilot_ui_init(lv_obj_t *root);
bool copilot_ui_is_ready(void);

void copilot_ui_set_expression(copilot_expr_t expr, uint32_t duration_ms);
void copilot_ui_set_motion(const copilot_motion_t *motion);
void copilot_ui_ring_show(bool on);
void copilot_ui_on_touch(uint16_t x, uint16_t y);

void copilot_ui_set_expression_async(copilot_expr_t expr, uint32_t duration_ms);
void copilot_ui_set_motion_async(const copilot_motion_t *motion);
void copilot_ui_set_motion_only_async(const copilot_motion_t *motion);  // Motion only, no expression trigger
void copilot_ui_ring_show_async(bool on);

#ifdef __cplusplus
}
#endif

#endif
