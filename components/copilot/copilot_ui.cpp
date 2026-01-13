#include "copilot_ui.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "sdkconfig.h"
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "copilot_perf.h"

#define LCD_H_RES BSP_LCD_H_RES
#define LCD_V_RES BSP_LCD_V_RES

static const char *TAG = "copilot_ui";

#define UI_OFFSET_X (-10)
#define UI_OFFSET_Y (-10)
#define FACE_INNER_OFFSET_X (0)
#define FACE_INNER_OFFSET_Y (0)

// Fixed-point constants for motion (avoid float on ESP32-C6)
#define MOTION_SHIFT_X_SIGN (-1)
#define MOTION_SHIFT_Y_SIGN (1)
#define MOTION_YAW_SIGN     (-1)

#define FACE_SIZE ((LCD_H_RES * 76) / 100)
#define FACE_X (((LCD_H_RES - FACE_SIZE) / 2) + UI_OFFSET_X)
#define FACE_Y (((LCD_V_RES - FACE_SIZE) / 2) + UI_OFFSET_Y)

#define EYE_SIZE ((FACE_SIZE * 28) / 100)
#define MOUTH_SIZE ((FACE_SIZE * 52) / 100)
#define EYE_SPACING ((FACE_SIZE * 20) / 100)

#define EYE_CENTER_Y ((FACE_SIZE * 35) / 100)
#define MOUTH_CENTER_Y ((FACE_SIZE * 62) / 100)

#define EYE_TOTAL_WIDTH ((EYE_SIZE * 2) + EYE_SPACING)
#define EYE_X_L ((FACE_SIZE - EYE_TOTAL_WIDTH) / 2)
#define EYE_X_R (EYE_X_L + EYE_SIZE + EYE_SPACING)
#define MOUTH_X ((FACE_SIZE - MOUTH_SIZE) / 2)

#define RING_SIZE ((LCD_H_RES * 94) / 100)
#define RING_X (((LCD_H_RES - RING_SIZE) / 2) + UI_OFFSET_X)
#define RING_Y (((LCD_V_RES - RING_SIZE) / 2) + UI_OFFSET_Y)
#define RING_SEGMENTS 60
#define RING_POINTS (RING_SEGMENTS + 1)

#define EYE_LINE_WIDTH 6
#define MOUTH_LINE_WIDTH 7
#define RING_OUTER_WIDTH 14
#define RING_INNER_WIDTH 7

// Animation timing - optimized for smoothness
#define ANIM_TIMER_MS 25          // 40fps for smooth animation
#define BLINK_CLOSE_MS 100        // Slightly slower close for cute effect
#define BLINK_OPEN_MS 140         // Slower open for natural feel
#define BLINK_INTERVAL_MIN 2000   // More frequent blinking
#define BLINK_INTERVAL_JITTER 2000
#define BLINK_EYE_START 255
#define BLINK_EYE_END 285
#define TOUCH_FLASH_MS 180

// Breathing animation for liveliness
#define BREATH_CYCLE_MS 3000      // 3 second breathing cycle
#define BREATH_AMPLITUDE 3        // Pixels of vertical movement

struct copilot_ui_state_t {
    lv_obj_t *root;
    lv_obj_t *face_root;
    lv_obj_t *ring_outer;
    lv_obj_t *ring_inner;
    lv_obj_t *eye_l;
    lv_obj_t *eye_r;
    lv_obj_t *mouth;

    face_arc_keyframe_t expr_from;
    face_arc_keyframe_t expr_to;
    face_arc_keyframe_t expr_cur;
    copilot_expr_t expr_current;
    uint32_t expr_start_ms;
    uint32_t expr_duration_ms;
    bool expr_animating;
    uint32_t expr_change_id;

    copilot_motion_t motion_target;
    copilot_motion_t motion_current;
    int16_t motion_roll;  // Q8.8 fixed-point
    uint32_t uneasy_until_ms;
    uint32_t uneasy_cooldown_ms;
    copilot_expr_t uneasy_restore_expr;
    uint32_t uneasy_restore_id;

    lv_timer_t *anim_timer;
    bool ring_visible;
    bool ready;
    uint32_t last_touch_ms;

    uint32_t next_blink_ms;
    uint32_t blink_phase_start;
    uint8_t blink_phase;

    bool touch_pending;
    uint32_t touch_flash_until_ms;
    bool touch_flash_active;
    bool touch_flash_owns_ring;

    bool face_applied;
    int16_t last_eye_start;
    int16_t last_eye_end;
    int16_t last_eye_rotation;
    int16_t last_mouth_start;
    int16_t last_mouth_end;
    int16_t last_mouth_rotation;
    int16_t last_eye_l_x;
    int16_t last_eye_r_x;
    int16_t last_eye_l_y;
    int16_t last_eye_r_y;
    int16_t last_mouth_x;
    int16_t last_mouth_y;
    bool last_eye_rounded;
    bool last_mouth_rounded;

    // Breathing animation state
    uint32_t breath_start_ms;
    int16_t breath_offset;      // Current breath offset in pixels
};

static copilot_ui_state_t s_ui = {};

// Ease-in-out cubic function for smooth animations
// Input: t in [0, t_max], Output: eased value in [0, t_max]
static uint32_t copilot_ease_in_out(uint32_t t, uint32_t t_max) {
    if (t_max == 0) return t_max;
    if (t >= t_max) return t_max;

    // Normalize to 0-256 range for fixed-point math
    uint32_t x = (t * 256) / t_max;
    uint32_t result;

    if (x < 128) {
        // Ease-in: 4 * x^3 (scaled)
        result = (4 * x * x * x) >> 16;
    } else {
        // Ease-out: 1 - 4 * (1-x)^3
        uint32_t inv = 256 - x;
        result = 256 - ((4 * inv * inv * inv) >> 16);
    }

    return (result * t_max) >> 8;
}

// Sine approximation for breathing animation (input: 0-360 degrees, output: -256 to 256)
static int16_t copilot_sin_approx(int16_t deg) {
    // Normalize to 0-360
    while (deg < 0) deg += 360;
    while (deg >= 360) deg -= 360;

    // Use symmetry: sin(180+x) = -sin(x)
    bool neg = false;
    if (deg >= 180) {
        deg -= 180;
        neg = true;
    }
    // sin(180-x) = sin(x)
    if (deg > 90) {
        deg = 180 - deg;
    }

    // Quadratic approximation for 0-90 degrees
    // sin(x) ≈ x * (180-x) * 4 / 32400 (scaled to 256)
    int32_t x = deg;
    int32_t result = (x * (90 - x) * 256) / (90 * 45);

    return neg ? (int16_t)(-result) : (int16_t)result;
}
static lv_point_precise_t s_ring_outer_pts[RING_POINTS];
static lv_point_precise_t s_ring_inner_pts[RING_POINTS];

static int16_t copilot_lerp_i16(int16_t a, int16_t b, uint32_t t, uint32_t t_max) {
    if (t_max == 0) {
        return b;
    }
    int32_t diff = (int32_t)b - (int32_t)a;
    return (int16_t)(a + (diff * (int32_t)t) / (int32_t)t_max);
}

static void copilot_touch_trigger(int16_t x, int16_t y);

static void copilot_build_ring_points(lv_point_precise_t *pts, int16_t radius) {
    if (!pts || radius <= 0) {
        return;
    }
    const int16_t cx = (int16_t)(RING_SIZE / 2);
    const int16_t cy = (int16_t)(RING_SIZE / 2);
    for (int i = 0; i <= RING_SEGMENTS; ++i) {
        int16_t angle = (int16_t)((360 * i) / RING_SEGMENTS);
        int32_t cos_v = lv_trigo_cos(angle);
        int32_t sin_v = lv_trigo_sin(angle);
        int32_t x = (int32_t)cx + (((int32_t)radius * cos_v) >> LV_TRIGO_SHIFT);
        int32_t y = (int32_t)cy + (((int32_t)radius * sin_v) >> LV_TRIGO_SHIFT);
        pts[i].x = (lv_coord_t)x;
        pts[i].y = (lv_coord_t)y;
    }
}

static int16_t copilot_wrap_angle(int32_t angle) {
    angle %= 360;
    if (angle < 0) {
        angle += 360;
    }
    return (int16_t)angle;
}

static int16_t copilot_wrap_angle_full(int32_t angle) {
    if (angle == 360) {
        return 360;
    }
    return copilot_wrap_angle(angle);
}

static int16_t copilot_arc_span(int16_t start, int16_t end) {
    if (start == 0 && end == 360) {
        return 360;
    }
    int16_t span = (int16_t)(end - start);
    if (span < 0) {
        span = (int16_t)(span + 360);
    }
    return span;
}

static int16_t copilot_lerp_angle(int16_t a, int16_t b, uint32_t t, uint32_t t_max) {
    if (t_max == 0) {
        return copilot_wrap_angle(b);
    }
    int16_t diff = (int16_t)(b - a);
    if (diff > 180) {
        diff = (int16_t)(diff - 360);
    } else if (diff < -180) {
        diff = (int16_t)(diff + 360);
    }
    int32_t value = (int32_t)a + ((int32_t)diff * (int32_t)t) / (int32_t)t_max;
    return copilot_wrap_angle(value);
}

static void copilot_lerp_frame(face_arc_keyframe_t *out, const face_arc_keyframe_t *a, const face_arc_keyframe_t *b,
                               uint32_t t, uint32_t t_max) {
    if (!out || !a || !b) {
        return;
    }
    int16_t eye_span_a = copilot_arc_span(a->eye_start, a->eye_end);
    int16_t eye_span_b = copilot_arc_span(b->eye_start, b->eye_end);
    int16_t eye_center_a = copilot_wrap_angle((int32_t)a->eye_start + eye_span_a / 2);
    int16_t eye_center_b = copilot_wrap_angle((int32_t)b->eye_start + eye_span_b / 2);

    int16_t eye_span = copilot_lerp_i16(eye_span_a, eye_span_b, t, t_max);
    int16_t eye_center = 0;
    if (eye_span_b >= 359) {
        eye_center = eye_center_a;
    } else if (eye_span_a >= 359) {
        eye_center = eye_center_b;
    } else {
        eye_center = copilot_lerp_angle(eye_center_a, eye_center_b, t, t_max);
    }

    if (eye_span >= 359) {
        out->eye_start = 0;
        out->eye_end = 360;
    } else {
        int16_t start = (int16_t)(eye_center - (eye_span / 2));
        out->eye_start = copilot_wrap_angle(start);
        out->eye_end = copilot_wrap_angle(start + eye_span);
    }

    out->eye_rotation = copilot_lerp_angle(a->eye_rotation, b->eye_rotation, t, t_max);
    out->eye_offset_y = copilot_lerp_i16(a->eye_offset_y, b->eye_offset_y, t, t_max);

    int16_t mouth_span_a = copilot_arc_span(a->mouth_start, a->mouth_end);
    int16_t mouth_span_b = copilot_arc_span(b->mouth_start, b->mouth_end);
    int16_t mouth_center_a = copilot_wrap_angle((int32_t)a->mouth_start + mouth_span_a / 2);
    int16_t mouth_center_b = copilot_wrap_angle((int32_t)b->mouth_start + mouth_span_b / 2);

    int16_t mouth_span = copilot_lerp_i16(mouth_span_a, mouth_span_b, t, t_max);
    int16_t mouth_center = 0;
    if (mouth_span_b >= 359) {
        mouth_center = mouth_center_a;
    } else if (mouth_span_a >= 359) {
        mouth_center = mouth_center_b;
    } else {
        mouth_center = copilot_lerp_angle(mouth_center_a, mouth_center_b, t, t_max);
    }

    if (mouth_span >= 359) {
        out->mouth_start = 0;
        out->mouth_end = 360;
    } else {
        int16_t start = (int16_t)(mouth_center - (mouth_span / 2));
        out->mouth_start = copilot_wrap_angle(start);
        out->mouth_end = copilot_wrap_angle(start + mouth_span);
    }

    out->mouth_rotation = copilot_lerp_angle(a->mouth_rotation, b->mouth_rotation, t, t_max);
    out->mouth_offset_y = copilot_lerp_i16(a->mouth_offset_y, b->mouth_offset_y, t, t_max);
}

static void copilot_apply_arc(lv_obj_t *arc, int16_t start, int16_t end, int16_t rotation) {
    if (!arc) {
        return;
    }
    start = copilot_wrap_angle_full(start);
    end = copilot_wrap_angle_full(end);
    lv_arc_set_angles(arc, (uint16_t)start, (uint16_t)end);
    lv_arc_set_rotation(arc, (uint16_t)copilot_wrap_angle(rotation));
}

// blink_ratio is Q8.8 fixed-point (0=open, FP_ONE=closed)
static void copilot_apply_face(const face_arc_keyframe_t *frame, int16_t blink_ratio) {
    if (!frame) {
        return;
    }

    int16_t eye_start = frame->eye_start;
    int16_t eye_end = frame->eye_end;
    if (blink_ratio > 0) {
        // Fixed-point interpolation: (1-ratio)*start + ratio*target
        // = start + ratio*(target - start)
        int32_t diff_start = BLINK_EYE_START - eye_start;
        int32_t diff_end = BLINK_EYE_END - eye_end;
        eye_start = eye_start + (int16_t)((diff_start * blink_ratio) >> FP_SHIFT);
        eye_end = eye_end + (int16_t)((diff_end * blink_ratio) >> FP_SHIFT);
    }

    int16_t eye_span = copilot_arc_span(eye_start, eye_end);
    bool eye_rounded = eye_span < 350;

    int16_t mouth_start = frame->mouth_start;
    int16_t mouth_end = frame->mouth_end;
    int16_t mouth_span = copilot_arc_span(mouth_start, mouth_end);
    bool mouth_rounded = mouth_span < 350;

    bool force = !s_ui.face_applied;
    if (force || eye_rounded != s_ui.last_eye_rounded) {
        lv_obj_set_style_arc_rounded(s_ui.eye_l, eye_rounded, LV_PART_INDICATOR | LV_STATE_DEFAULT);
        lv_obj_set_style_arc_rounded(s_ui.eye_r, eye_rounded, LV_PART_INDICATOR | LV_STATE_DEFAULT);
        s_ui.last_eye_rounded = eye_rounded;
    }
    if (force || mouth_rounded != s_ui.last_mouth_rounded) {
        lv_obj_set_style_arc_rounded(s_ui.mouth, mouth_rounded, LV_PART_INDICATOR | LV_STATE_DEFAULT);
        s_ui.last_mouth_rounded = mouth_rounded;
    }

    // roll_offset = motion_roll * 6, motion_roll is Q8.8, result in pixels
    int16_t roll_offset = (int16_t)((s_ui.motion_roll * 6) >> FP_SHIFT);

    // Add breathing animation offset for liveliness
    int16_t breath = s_ui.breath_offset;

    int16_t eye_center_y = (int16_t)EYE_CENTER_Y + frame->eye_offset_y + roll_offset + breath + FACE_INNER_OFFSET_Y;
    int16_t mouth_center_y = (int16_t)MOUTH_CENTER_Y + frame->mouth_offset_y + (roll_offset / 2) + (breath / 2) + FACE_INNER_OFFSET_Y;

    int16_t eye_l_x = (int16_t)(EYE_X_L + FACE_INNER_OFFSET_X);
    int16_t eye_r_x = (int16_t)(EYE_X_R + FACE_INNER_OFFSET_X);
    int16_t eye_l_y = (int16_t)(eye_center_y - (EYE_SIZE / 2));
    int16_t eye_r_y = (int16_t)(eye_center_y - (EYE_SIZE / 2));
    int16_t mouth_x = (int16_t)(MOUTH_X + FACE_INNER_OFFSET_X);
    int16_t mouth_y = (int16_t)(mouth_center_y - (MOUTH_SIZE / 2));

    if (force || eye_l_x != s_ui.last_eye_l_x || eye_l_y != s_ui.last_eye_l_y) {
        lv_obj_set_pos(s_ui.eye_l, eye_l_x, eye_l_y);
        s_ui.last_eye_l_x = eye_l_x;
        s_ui.last_eye_l_y = eye_l_y;
    }
    if (force || eye_r_x != s_ui.last_eye_r_x || eye_r_y != s_ui.last_eye_r_y) {
        lv_obj_set_pos(s_ui.eye_r, eye_r_x, eye_r_y);
        s_ui.last_eye_r_x = eye_r_x;
        s_ui.last_eye_r_y = eye_r_y;
    }
    if (force || mouth_x != s_ui.last_mouth_x || mouth_y != s_ui.last_mouth_y) {
        lv_obj_set_pos(s_ui.mouth, mouth_x, mouth_y);
        s_ui.last_mouth_x = mouth_x;
        s_ui.last_mouth_y = mouth_y;
    }

    if (force || eye_start != s_ui.last_eye_start || eye_end != s_ui.last_eye_end ||
        frame->eye_rotation != s_ui.last_eye_rotation) {
        copilot_apply_arc(s_ui.eye_l, eye_start, eye_end, frame->eye_rotation);
        copilot_apply_arc(s_ui.eye_r, eye_start, eye_end, frame->eye_rotation);
        s_ui.last_eye_start = eye_start;
        s_ui.last_eye_end = eye_end;
        s_ui.last_eye_rotation = frame->eye_rotation;
    }

    if (force || frame->mouth_start != s_ui.last_mouth_start || frame->mouth_end != s_ui.last_mouth_end ||
        frame->mouth_rotation != s_ui.last_mouth_rotation) {
        copilot_apply_arc(s_ui.mouth, frame->mouth_start, frame->mouth_end, frame->mouth_rotation);
        s_ui.last_mouth_start = frame->mouth_start;
        s_ui.last_mouth_end = frame->mouth_end;
        s_ui.last_mouth_rotation = frame->mouth_rotation;
    }

    s_ui.face_applied = true;
}

static void copilot_arc_style(lv_obj_t *arc, lv_color_t color, uint8_t width) {
    lv_obj_set_style_arc_color(arc, color, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(arc, width, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(arc, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(arc, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(arc, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_rounded(arc, true, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
}

static void copilot_ring_set_opa(lv_obj_t *line, lv_opa_t opa) {
    lv_obj_set_style_line_opa(line, opa, LV_PART_MAIN | LV_STATE_DEFAULT);
}

static void copilot_apply_motion_transform(void) {
#if CONFIG_COPILOT_USE_FLOAT_MOTION
    float ax = (float)s_ui.motion_current.ax / (float)FP_ONE;
    float ay = (float)s_ui.motion_current.ay / (float)FP_ONE;
    float yaw = (float)s_ui.motion_current.yaw_deg / (float)FP_ONE;

    if (ax > 1.0f) ax = 1.0f;
    if (ax < -1.0f) ax = -1.0f;
    if (ay > 1.0f) ay = 1.0f;
    if (ay < -1.0f) ay = -1.0f;

    float max_shift = (float)CONFIG_COPILOT_MOTION_MAX_SHIFT_PX;
    int16_t shift_x = (int16_t)(ay * max_shift * (float)MOTION_SHIFT_X_SIGN);
    int16_t shift_y = (int16_t)(ax * max_shift * (float)MOTION_SHIFT_Y_SIGN);

    float max_angle = (float)CONFIG_COPILOT_MOTION_MAX_ANGLE_DEG;
    if (max_angle < 1.0f) {
        max_angle = 1.0f;
    }
    if (yaw > max_angle) yaw = max_angle;
    if (yaw < -max_angle) yaw = -max_angle;

    if (s_ui.uneasy_until_ms && lv_tick_get() < s_ui.uneasy_until_ms) {
        float shake = ((lv_tick_get() / 90) % 2) ? 3.0f : -3.0f;
        yaw += shake;
    }

    float yaw_norm = (yaw * (float)MOTION_YAW_SIGN) / max_angle;
    if (yaw_norm > 1.0f) yaw_norm = 1.0f;
    if (yaw_norm < -1.0f) yaw_norm = -1.0f;
    s_ui.motion_roll = (int16_t)(yaw_norm * (float)FP_ONE);

    lv_obj_set_pos(s_ui.face_root, FACE_X + shift_x, FACE_Y + shift_y);
#else
    int16_t ax = s_ui.motion_current.ax;
    int16_t ay = s_ui.motion_current.ay;
    int16_t yaw = s_ui.motion_current.yaw_deg;

    // Clamp to -1.0 to +1.0 (Q8.8: -256 to +256)
    if (ax > FP_ONE) ax = FP_ONE;
    if (ax < -FP_ONE) ax = -FP_ONE;
    if (ay > FP_ONE) ay = FP_ONE;
    if (ay < -FP_ONE) ay = -FP_ONE;

    // max_shift in pixels, convert to Q8.8 for calculation
    int16_t max_shift = CONFIG_COPILOT_MOTION_MAX_SHIFT_PX;
    // shift = ay * max_shift (ay is Q8.8, result in pixels)
    int16_t shift_x = (int16_t)(((int32_t)ay * max_shift * MOTION_SHIFT_X_SIGN) >> FP_SHIFT);
    int16_t shift_y = (int16_t)(((int32_t)ax * max_shift * MOTION_SHIFT_Y_SIGN) >> FP_SHIFT);

    int16_t max_angle = CONFIG_COPILOT_MOTION_MAX_ANGLE_DEG;
    if (max_angle < 1) max_angle = 1;
    // yaw is in Q8.8 degrees, clamp to max_angle
    int16_t max_angle_fp = max_angle << FP_SHIFT;
    if (yaw > max_angle_fp) yaw = max_angle_fp;
    if (yaw < -max_angle_fp) yaw = -max_angle_fp;

    if (s_ui.uneasy_until_ms && lv_tick_get() < s_ui.uneasy_until_ms) {
        int16_t shake = ((lv_tick_get() / 90) % 2) ? (3 << FP_SHIFT) : (-3 << FP_SHIFT);
        yaw += shake;
    }

    // motion_roll = (yaw * MOTION_YAW_SIGN) / max_angle, result is Q8.8
    // = (yaw * MOTION_YAW_SIGN * FP_ONE) / max_angle_fp
    s_ui.motion_roll = (int16_t)(((int32_t)yaw * MOTION_YAW_SIGN * FP_ONE) / max_angle_fp);
    if (s_ui.motion_roll > FP_ONE) s_ui.motion_roll = FP_ONE;
    if (s_ui.motion_roll < -FP_ONE) s_ui.motion_roll = -FP_ONE;

    lv_obj_set_pos(s_ui.face_root, FACE_X + shift_x, FACE_Y + shift_y);
#endif
}

// Fixed-point smoothing alpha = 0.18 ≈ 46/256
#define MOTION_ALPHA 46

// Returns true if motion changed enough to warrant redraw
static bool copilot_update_motion(void) {
    int16_t old_ax = s_ui.motion_current.ax;
    int16_t old_ay = s_ui.motion_current.ay;
    int16_t old_yaw = s_ui.motion_current.yaw_deg;
    
    // Exponential smoothing: current += (target - current) * alpha
    // Using integer arithmetic: current += ((target - current) * ALPHA) >> 8
    s_ui.motion_current.ax += (int16_t)(((int32_t)(s_ui.motion_target.ax - s_ui.motion_current.ax) * MOTION_ALPHA) >> FP_SHIFT);
    s_ui.motion_current.ay += (int16_t)(((int32_t)(s_ui.motion_target.ay - s_ui.motion_current.ay) * MOTION_ALPHA) >> FP_SHIFT);
    s_ui.motion_current.yaw_deg += (int16_t)(((int32_t)(s_ui.motion_target.yaw_deg - s_ui.motion_current.yaw_deg) * MOTION_ALPHA) >> FP_SHIFT);
    s_ui.motion_current.speed += (int16_t)(((int32_t)(s_ui.motion_target.speed - s_ui.motion_current.speed) * MOTION_ALPHA) >> FP_SHIFT);
    
    // Check if changed significantly (threshold ~1% of range)
    int16_t dx = s_ui.motion_current.ax - old_ax;
    int16_t dy = s_ui.motion_current.ay - old_ay;
    int16_t dyaw = s_ui.motion_current.yaw_deg - old_yaw;
    if (dx < 0) dx = -dx;
    if (dy < 0) dy = -dy;
    if (dyaw < 0) dyaw = -dyaw;
    
    bool changed = (dx > 2 || dy > 2 || dyaw > 2);
    if (changed) {
        copilot_apply_motion_transform();
    }
    return changed;
}

static void copilot_trigger_uneasy(void) {
    uint32_t now = lv_tick_get();
    if (now < s_ui.uneasy_cooldown_ms) {
        return;
    }

    s_ui.uneasy_cooldown_ms = now + 1200;
    s_ui.uneasy_until_ms = now + 900;
    s_ui.uneasy_restore_expr = s_ui.expr_current;
    s_ui.uneasy_restore_id = s_ui.expr_change_id + 1;

    copilot_ui_set_expression(COPILOT_EXPR_DIZZY, 220);
}

// Returns Q8.8 fixed-point blink ratio (0=open, FP_ONE=closed)
// Smooth continuous animation with easing for cute, natural blinks
static int16_t copilot_update_blink(uint32_t now) {
    if (s_ui.blink_phase == 0) {
        // Waiting phase - eyes open
        if (now >= s_ui.next_blink_ms) {
            s_ui.blink_phase = 1;
            s_ui.blink_phase_start = now;
        }
        return 0;
    }

    if (s_ui.blink_phase == 1) {
        // Closing phase - smooth ease-in
        uint32_t elapsed = now - s_ui.blink_phase_start;
        if (elapsed >= BLINK_CLOSE_MS) {
            s_ui.blink_phase = 2;
            s_ui.blink_phase_start = now;
            return FP_ONE;  // Fully closed
        }
        // Smooth interpolation with ease-in (fast close)
        uint32_t eased = copilot_ease_in_out(elapsed, BLINK_CLOSE_MS);
        return (int16_t)((eased * FP_ONE) / BLINK_CLOSE_MS);
    }

    if (s_ui.blink_phase == 2) {
        // Opening phase - smooth ease-out (slow open for cute effect)
        uint32_t elapsed = now - s_ui.blink_phase_start;
        if (elapsed >= BLINK_OPEN_MS) {
            s_ui.blink_phase = 0;
            // Random interval with some variation
            uint32_t jitter = (now * 7919) % BLINK_INTERVAL_JITTER;
            s_ui.next_blink_ms = now + BLINK_INTERVAL_MIN + jitter;
            return 0;  // Fully open
        }
        // Smooth interpolation with ease-out (slow open)
        uint32_t eased = copilot_ease_in_out(elapsed, BLINK_OPEN_MS);
        return FP_ONE - (int16_t)((eased * FP_ONE) / BLINK_OPEN_MS);
    }

    return 0;
}

// Update breathing animation - returns vertical offset in pixels
static int16_t copilot_update_breath(uint32_t now) {
    if (s_ui.breath_start_ms == 0) {
        s_ui.breath_start_ms = now;
    }

    uint32_t elapsed = (now - s_ui.breath_start_ms) % BREATH_CYCLE_MS;
    int16_t angle = (int16_t)((elapsed * 360) / BREATH_CYCLE_MS);

    // Sine wave for smooth breathing
    int16_t sin_val = copilot_sin_approx(angle);  // -256 to 256
    return (int16_t)((sin_val * BREATH_AMPLITUDE) >> 8);
}

static void copilot_anim_timer(lv_timer_t *timer) {
    (void)timer;
    uint32_t now = lv_tick_get();
    bool need_redraw = false;
    static int16_t last_blink_ratio = -1;
    static int16_t last_breath_offset = 0;

    // Record frame for FPS calculation
    copilot_perf_frame_tick();

    // Expression transition with easing
    if (s_ui.expr_animating) {
        uint32_t elapsed = now - s_ui.expr_start_ms;
        if (elapsed >= s_ui.expr_duration_ms) {
            s_ui.expr_animating = false;
            s_ui.expr_cur = s_ui.expr_to;
        } else {
            // Apply ease-in-out for smooth expression transitions
            uint32_t eased_elapsed = copilot_ease_in_out(elapsed, s_ui.expr_duration_ms);
            copilot_lerp_frame(&s_ui.expr_cur, &s_ui.expr_from, &s_ui.expr_to, eased_elapsed, s_ui.expr_duration_ms);
        }
        need_redraw = true;
    }

    if (s_ui.uneasy_until_ms && now >= s_ui.uneasy_until_ms) {
        s_ui.uneasy_until_ms = 0;
        if (s_ui.expr_change_id == s_ui.uneasy_restore_id) {
            copilot_ui_set_expression(s_ui.uneasy_restore_expr, 300);  // Slightly longer recovery
        }
        need_redraw = true;
    }

    if (copilot_update_motion()) {
        need_redraw = true;
    }

    if (s_ui.touch_pending) {
        s_ui.touch_pending = false;
        copilot_touch_trigger(0, 0);
    }

    if (s_ui.touch_flash_active && now >= s_ui.touch_flash_until_ms) {
        s_ui.touch_flash_active = false;
        if (s_ui.touch_flash_owns_ring) {
            s_ui.touch_flash_owns_ring = false;
            copilot_ui_ring_show(false);
        }
    }

    // Smooth blink animation
    int16_t blink_ratio = copilot_update_blink(now);
    if (blink_ratio != last_blink_ratio) {
        last_blink_ratio = blink_ratio;
        need_redraw = true;
    }

    // Breathing animation for liveliness
    int16_t breath_offset = copilot_update_breath(now);
    if (breath_offset != last_breath_offset) {
        last_breath_offset = breath_offset;
        s_ui.breath_offset = breath_offset;
        need_redraw = true;
    }

    // Always redraw during animations, skip only when truly idle
    if (need_redraw || !s_ui.face_applied) {
        copilot_apply_face(&s_ui.expr_cur, blink_ratio);
    }
}

static bool copilot_inside_circle(lv_coord_t x, lv_coord_t y) {
    int32_t dx = (int32_t)x - (LCD_H_RES / 2);
    int32_t dy = (int32_t)y - (LCD_V_RES / 2);
    int32_t r = (LCD_H_RES / 2) - 2;
    return (dx * dx + dy * dy) <= (r * r);
}

static void copilot_touch_trigger(int16_t x, int16_t y) {
    (void)x;
    (void)y;
    uint32_t now = lv_tick_get();
    s_ui.touch_flash_until_ms = now + TOUCH_FLASH_MS;
    if (!s_ui.touch_flash_active) {
        s_ui.touch_flash_active = true;
        s_ui.touch_flash_owns_ring = !s_ui.ring_visible;
        if (s_ui.touch_flash_owns_ring) {
            copilot_ui_ring_show(true);
        }
    }
}

void copilot_ui_init(lv_obj_t *root) {
    if (!root || s_ui.ready) {
        return;
    }

    memset(&s_ui, 0, sizeof(s_ui));
    lv_obj_clear_flag(root, LV_OBJ_FLAG_SCROLLABLE);
    s_ui.root = lv_obj_create(root);
    lv_obj_set_size(s_ui.root, LCD_H_RES, LCD_V_RES);
    lv_obj_set_pos(s_ui.root, 0, 0);
    lv_obj_set_scrollbar_mode(s_ui.root, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(s_ui.root, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_style_bg_color(s_ui.root, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(s_ui.root, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(s_ui.root, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(s_ui.root, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    int16_t outer_radius = (int16_t)((RING_SIZE / 2) - (RING_OUTER_WIDTH / 2));
    int16_t inner_radius = outer_radius;
    copilot_build_ring_points(s_ring_outer_pts, outer_radius);
    copilot_build_ring_points(s_ring_inner_pts, inner_radius);

    s_ui.ring_outer = lv_line_create(s_ui.root);
    lv_obj_set_size(s_ui.ring_outer, RING_SIZE, RING_SIZE);
    lv_obj_set_pos(s_ui.ring_outer, RING_X, RING_Y);
    lv_line_set_points(s_ui.ring_outer, s_ring_outer_pts, RING_POINTS);
    lv_obj_set_style_line_width(s_ui.ring_outer, RING_OUTER_WIDTH, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_color(s_ui.ring_outer, lv_color_hex(0x1AB5FF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_rounded(s_ui.ring_outer, false, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_opa(s_ui.ring_outer, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(s_ui.ring_outer, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(s_ui.ring_outer, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(s_ui.ring_outer, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(s_ui.ring_outer, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(s_ui.ring_outer, LV_OBJ_FLAG_HIDDEN);

    s_ui.ring_inner = lv_line_create(s_ui.root);
    lv_obj_set_size(s_ui.ring_inner, RING_SIZE, RING_SIZE);
    lv_obj_set_pos(s_ui.ring_inner, RING_X, RING_Y);
    lv_line_set_points(s_ui.ring_inner, s_ring_inner_pts, RING_POINTS);
    lv_obj_set_style_line_width(s_ui.ring_inner, RING_INNER_WIDTH, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_color(s_ui.ring_inner, lv_color_hex(0x61E6FF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_rounded(s_ui.ring_inner, false, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_opa(s_ui.ring_inner, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(s_ui.ring_inner, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(s_ui.ring_inner, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(s_ui.ring_inner, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(s_ui.ring_inner, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(s_ui.ring_inner, LV_OBJ_FLAG_HIDDEN);

    s_ui.face_root = lv_obj_create(s_ui.root);
    lv_obj_set_size(s_ui.face_root, FACE_SIZE, FACE_SIZE);
    lv_obj_set_pos(s_ui.face_root, FACE_X, FACE_Y);
    lv_obj_set_scrollbar_mode(s_ui.face_root, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(s_ui.face_root, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(s_ui.face_root, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(s_ui.face_root, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all(s_ui.face_root, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_color_t line_color = lv_color_hex(0xE6F2FF);

    s_ui.eye_l = lv_arc_create(s_ui.face_root);
    s_ui.eye_r = lv_arc_create(s_ui.face_root);
    s_ui.mouth = lv_arc_create(s_ui.face_root);

    lv_obj_set_size(s_ui.eye_l, EYE_SIZE, EYE_SIZE);
    lv_obj_set_size(s_ui.eye_r, EYE_SIZE, EYE_SIZE);
    lv_obj_set_size(s_ui.mouth, MOUTH_SIZE, MOUTH_SIZE);
    lv_arc_set_bg_angles(s_ui.eye_l, 0, 360);
    lv_arc_set_bg_angles(s_ui.eye_r, 0, 360);
    lv_arc_set_bg_angles(s_ui.mouth, 0, 360);

    copilot_arc_style(s_ui.eye_l, line_color, EYE_LINE_WIDTH);
    copilot_arc_style(s_ui.eye_r, line_color, EYE_LINE_WIDTH);
    copilot_arc_style(s_ui.mouth, line_color, MOUTH_LINE_WIDTH);

    s_ui.expr_current = COPILOT_EXPR_NEUTRAL;
    s_ui.expr_cur = kFaceKeyframes[s_ui.expr_current];
    copilot_apply_face(&s_ui.expr_cur, 0);

    // Animation timer at 40fps for smooth animations
    s_ui.anim_timer = lv_timer_create(copilot_anim_timer, ANIM_TIMER_MS, nullptr);
    s_ui.next_blink_ms = lv_tick_get() + 1500;  // First blink sooner
    s_ui.breath_start_ms = lv_tick_get();
    s_ui.ready = true;

    ESP_LOGI(TAG, "UI ready");
}

bool copilot_ui_is_ready(void) {
    return s_ui.ready;
}

void copilot_ui_set_expression(copilot_expr_t expr, uint32_t duration_ms) {
    if (!s_ui.ready || expr >= COPILOT_EXPR_COUNT) {
        return;
    }

    if (expr == s_ui.expr_current && !s_ui.expr_animating) {
        return;
    }

    ESP_LOGI(TAG, "Expression change: %d -> %d (%u ms)", (int)s_ui.expr_current, (int)expr, (unsigned)duration_ms);
    s_ui.expr_change_id++;
    s_ui.expr_current = expr;

    if (duration_ms == 0) {
        s_ui.expr_animating = false;
        s_ui.expr_cur = kFaceKeyframes[expr];
        copilot_apply_face(&s_ui.expr_cur, 0);
        return;
    }

    s_ui.expr_from = s_ui.expr_cur;
    s_ui.expr_to = kFaceKeyframes[expr];
    s_ui.expr_start_ms = lv_tick_get();
    s_ui.expr_duration_ms = duration_ms;
    s_ui.expr_animating = true;
}

void copilot_ui_set_motion(const copilot_motion_t *motion) {
    if (!s_ui.ready || !motion) {
        return;
    }

#ifdef CONFIG_COPILOT_MOTION_SOURCE_DISABLED
    // Motion disabled - ignore all input
    return;
#endif

    s_ui.motion_target = *motion;
    ESP_LOGD(TAG, "Motion target ax=%d ay=%d yaw=%d speed=%d (Q8.8)",
             motion->ax, motion->ay, motion->yaw_deg, motion->speed);

    // Check uneasy threshold using fixed-point
    // acc = max(|ax|, |ay|), values are Q8.8
    int16_t ax_abs = (motion->ax >= 0) ? motion->ax : -motion->ax;
    int16_t ay_abs = (motion->ay >= 0) ? motion->ay : -motion->ay;
    int16_t acc = (ax_abs > ay_abs) ? ax_abs : ay_abs;
    
    // acc_threshold = CONFIG_COPILOT_UNEASY_ACCEL_CG / 100.0, convert to Q8.8
    int16_t acc_threshold = (CONFIG_COPILOT_UNEASY_ACCEL_CG * FP_ONE) / 100;
    // yaw_threshold in degrees, convert to Q8.8
    int16_t yaw_threshold = CONFIG_COPILOT_UNEASY_YAW_DEG << FP_SHIFT;
    int16_t yaw_abs = (motion->yaw_deg >= 0) ? motion->yaw_deg : -motion->yaw_deg;

    if (acc > acc_threshold || yaw_abs > yaw_threshold) {
        copilot_trigger_uneasy();
    }
}

void copilot_ui_ring_show(bool on) {
    if (!s_ui.ready) {
        return;
    }
    if (on && s_ui.touch_flash_active && s_ui.touch_flash_owns_ring) {
        s_ui.touch_flash_owns_ring = false;
    }
    if (on == s_ui.ring_visible) {
        return;
    }
    s_ui.ring_visible = on;
    ESP_LOGI(TAG, "Ring show=%s", on ? "on" : "off");

    if (on) {
        copilot_ring_set_opa(s_ui.ring_outer, 150);
        copilot_ring_set_opa(s_ui.ring_inner, 240);
        lv_obj_clear_flag(s_ui.ring_outer, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_ui.ring_inner, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_ui.ring_outer, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_ui.ring_inner, LV_OBJ_FLAG_HIDDEN);
    }
}

void copilot_ui_on_touch(uint16_t x, uint16_t y) {
    if (!s_ui.ready || !copilot_inside_circle(x, y)) {
        return;
    }
    uint32_t now = lv_tick_get();
    // Increase debounce to 300ms to reduce touch event frequency on single-core ESP32-C6
    if (now - s_ui.last_touch_ms < 300) {
        return;
    }
    s_ui.last_touch_ms = now;
    ESP_LOGI(TAG, "Touch x=%u y=%u", (unsigned)x, (unsigned)y);
    // Disable ring flash animation - too heavy for single-core ESP32-C6
    // The ring show/hide causes full screen redraws which starve WiFi
    // s_ui.touch_pending = true;
}

void copilot_ui_set_expression_async(copilot_expr_t expr, uint32_t duration_ms) {
    if (!s_ui.ready) {
        return;
    }
    if (bsp_display_lock(0)) {
        copilot_ui_set_expression(expr, duration_ms);
        bsp_display_unlock();
    }
}

void copilot_ui_set_motion_async(const copilot_motion_t *motion) {
    if (!s_ui.ready || !motion) {
        return;
    }
    if (bsp_display_lock(0)) {
        copilot_ui_set_motion(motion);
        bsp_display_unlock();
    }
}

void copilot_ui_set_motion_only_async(const copilot_motion_t *motion) {
    if (!s_ui.ready || !motion) {
        return;
    }
    if (bsp_display_lock(0)) {
        // Only update motion target without checking uneasy threshold
        // This is used by internal IMU to avoid expression changes
        s_ui.motion_target = *motion;
        ESP_LOGD(TAG, "Motion only ax=%d ay=%d yaw=%d (Q8.8)",
                 motion->ax, motion->ay, motion->yaw_deg);
        bsp_display_unlock();
    }
}

void copilot_ui_ring_show_async(bool on) {
    if (!s_ui.ready) {
        return;
    }
    if (bsp_display_lock(0)) {
        copilot_ui_ring_show(on);
        bsp_display_unlock();
    }
}
