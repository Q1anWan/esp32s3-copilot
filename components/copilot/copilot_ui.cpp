#include "copilot_ui.h"

#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "sdkconfig.h"
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "copilot_perf.h"
#include "copilot_voice_ui.h"

#if CONFIG_COPILOT_SERVO_ENABLE
#include "copilot_servo.h"
#endif

#define LCD_H_RES BSP_LCD_H_RES
#define LCD_V_RES BSP_LCD_V_RES

static const char *TAG = "copilot_ui";

#if CONFIG_COPILOT_LOG_UI
#define LOGI_UI(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#else
#define LOGI_UI(fmt, ...) do {} while (0)
#endif

#define UI_OFFSET_X (-8)
#define UI_OFFSET_Y (-4)

#define FACE_BOX_SIZE ((LCD_H_RES * 80) / 100)
#define FACE_BOX_X (((LCD_H_RES - FACE_BOX_SIZE) / 2) + UI_OFFSET_X)
#define FACE_BOX_Y (((LCD_V_RES - FACE_BOX_SIZE) / 2) + UI_OFFSET_Y)

#define RING_SIZE ((LCD_H_RES * 94) / 100)
#define RING_X (((LCD_H_RES - RING_SIZE) / 2) + UI_OFFSET_X)
#define RING_Y (((LCD_V_RES - RING_SIZE) / 2) + UI_OFFSET_Y)
#define RING_SEGMENTS 60
#define RING_POINTS (RING_SEGMENTS + 1)
#define RING_OUTER_WIDTH 14
#define RING_INNER_WIDTH 7

#define ANIM_TIMER_MS 16
#define TOUCH_FLASH_MS  180
#define SPEAKING_HOLD_MS 120

/*
 * Warm reddish-grayscale palette — high contrast for OLED night visibility.
 *   C_BG      pure black (AMOLED off)
 *   C_HEAD    warm dark brown-gray (head fill)
 *   C_OUTLINE bright warm beige (head / feature outlines)
 *   C_SHADOW  warm medium shadow (mouth)
 *   C_CHEEK   rosy blush
 *   C_EYE     warm off-white (eye whites)
 *   C_SPARK   pure white (catchlights, tooth)
 *   C_NOSE    warm light tan (nose highlight)
 */
#define C_BG      0x000000
#define C_HEAD    0x4A2828
#define C_OUTLINE 0xE0B8A8
#define C_SHADOW  0x6A3838
#define C_CHEEK   0xD09080
#define C_EYE     0xFFF6EE
#define C_SPARK   0xFFFFFF
#define C_NOSE    0xD4A898
#define C_MOUTH_INNER 0x4A2020

// --- Head ---
#define HEAD_W ((FACE_BOX_SIZE * 65) / 100)
#define HEAD_H ((FACE_BOX_SIZE * 62) / 100)
#define HEAD_X  (((FACE_BOX_SIZE - HEAD_W) / 2))
#define HEAD_Y  (((FACE_BOX_SIZE - HEAD_H) / 2))
#define HEAD_OUTLINE ((FACE_BOX_SIZE * 2) / 100)   // outline thickness
#define HEAD_SHADOW_OFS 5                           // head drop‑shadow offset (px)

// --- Eyes (front‑facing reference) ---
#define EYE_W_REF  ((FACE_BOX_SIZE * 10) / 100)
#define EYE_H_REF  ((FACE_BOX_SIZE * 15) / 100)
#define EYE_GAP_REF ((FACE_BOX_SIZE * 22) / 100)
#define EYE_SHADOW_OFS 2

// --- Cheek blush ---
#define CHEEK_W ((FACE_BOX_SIZE * 7) / 100)
#define CHEEK_H ((FACE_BOX_SIZE * 4) / 100)

// --- Nose ---
#define NOSE_W ((FACE_BOX_SIZE * 3) / 100)
#define NOSE_H ((FACE_BOX_SIZE * 5) / 100)

// --- Mouth ---
#define MOUTH_IDLE_W ((FACE_BOX_SIZE * 15) / 100)
#define MOUTH_IDLE_H 3
#define MOUTH_OPEN_W ((FACE_BOX_SIZE * 16) / 100)
#define MOUTH_OPEN_H ((FACE_BOX_SIZE * 17) / 100)

#define MOTION_SHIFT_X_SIGN (-1)
#define MOTION_SHIFT_Y_SIGN (1)
#define MOTION_YAW_SIGN     (-1)
#define MOTION_ALPHA 46
#define FACE_ANGLE_SMOOTH 72

/*
 * Object role map  (rich shaded cartoon):
 *   head         → dark filled oval + mid‑tone outline
 *   eye_l / eye_r → bright white capsules
 *   eye_l_cut / eye_r_cut → pure black pupils
 *   eye_l_spark / eye_r_spark → pure white catchlights
 *   cheek_l / cheek_r → warm gray blush circles
 *   nose         → light gray nose tip
 *   face_patch   → nose bridge (profile only)
 *   mouth        → dark pill (idle) / dark oval + tooth (speaking)
 *   mouth_inner  → deeper shadow inside open mouth
 *   tooth        → white bar
 */

struct copilot_ui_state_t {
    lv_obj_t *root;
    lv_obj_t *face_root;
    lv_obj_t *ring_outer;
    lv_obj_t *ring_inner;

    lv_obj_t *head;
    lv_obj_t *cheek_l;
    lv_obj_t *cheek_r;
    lv_obj_t *face_patch;
    lv_obj_t *eye_l;
    lv_obj_t *eye_r;
    lv_obj_t *eye_l_cut;
    lv_obj_t *eye_r_cut;
    lv_obj_t *eye_l_spark;
    lv_obj_t *eye_r_spark;
    lv_obj_t *nose;
    lv_obj_t *mouth;
    lv_obj_t *mouth_inner;
    lv_obj_t *tooth;

    copilot_motion_t motion_target;
    copilot_motion_t motion_current;
    int16_t motion_roll;

    int16_t face_angle;        // Q8.8 degrees, [-90, +90]
    int16_t face_angle_prev;   // previous frame value (for rate‑of‑change)

    lv_timer_t *anim_timer;
    bool ring_visible;
    bool ready;
    uint32_t last_touch_ms;
    bool touch_flash_active;
    bool touch_flash_owns_ring;
    uint32_t touch_flash_until_ms;

    bool manual_speaking;
    bool manual_speaking_latched;
    uint32_t manual_speaking_until_ms;
    uint32_t speaking_until_ms;
    uint8_t mouth_progress;
    bool last_speaking;
    uint32_t last_idle_pose_bucket;
};

static copilot_ui_state_t s_ui = {};
static lv_point_precise_t s_ring_outer_pts[RING_POINTS];
static lv_point_precise_t s_ring_inner_pts[RING_POINTS];

static int16_t copilot_abs_i16(int16_t v) { return v < 0 ? (int16_t)-v : v; }

static void copilot_build_ring_points(lv_point_precise_t *pts, int16_t radius) {
    if (!pts || radius <= 0) return;
    const int16_t cx = (int16_t)(RING_SIZE / 2);
    const int16_t cy = (int16_t)(RING_SIZE / 2);
    for (int i = 0; i <= RING_SEGMENTS; ++i) {
        int16_t angle = (int16_t)((360 * i) / RING_SEGMENTS);
        int32_t cos_v = lv_trigo_cos(angle);
        int32_t sin_v = lv_trigo_sin(angle);
        pts[i].x = (lv_coord_t)((int32_t)cx + (((int32_t)radius * cos_v) >> LV_TRIGO_SHIFT));
        pts[i].y = (lv_coord_t)((int32_t)cy + (((int32_t)radius * sin_v) >> LV_TRIGO_SHIFT));
    }
}

static void copilot_line_style(lv_obj_t *line, lv_color_t color, uint8_t width) {
    lv_obj_set_style_line_width(line, width, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_color(line, color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_opa(line, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_rounded(line, false, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(line, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(line, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(line, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(line, LV_OBJ_FLAG_CLICKABLE);
}

static void copilot_blob_fill(lv_obj_t *obj, lv_color_t fill) {
    lv_obj_set_style_bg_color(obj, fill, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(obj, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(obj, LV_RADIUS_CIRCLE, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE);
}

static void copilot_blob_outline(lv_obj_t *obj, lv_color_t fill, lv_color_t border, uint8_t border_w) {
    lv_obj_set_style_bg_color(obj, fill, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(obj, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(obj, border, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(obj, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(obj, border_w, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(obj, LV_RADIUS_CIRCLE, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE);
}

static void copilot_ring_set_opa(lv_obj_t *line, lv_opa_t opa) {
    lv_obj_set_style_line_opa(line, opa, LV_PART_MAIN | LV_STATE_DEFAULT);
}

static bool copilot_voice_speaking(uint32_t now) {
    copilot_voice_state_t voice_state = copilot_voice_ui_get_state();
    uint8_t mouth_open = copilot_voice_ui_get_mouth_open();
    if (voice_state == VOICE_STATE_SPEAKING || mouth_open > 18) {
        s_ui.speaking_until_ms = now + SPEAKING_HOLD_MS;
        return true;
    }
    return now < s_ui.speaking_until_ms;
}

static bool copilot_manual_speaking(uint32_t now) {
    if (!s_ui.manual_speaking) return false;
    if (s_ui.manual_speaking_latched) return true;
    if (now < s_ui.manual_speaking_until_ms) return true;
    s_ui.manual_speaking = false;
    return false;
}

static bool copilot_is_speaking(uint32_t now) {
    return copilot_voice_speaking(now) || copilot_manual_speaking(now);
}

/* Sine‑based elastic offset — breathing (idle) or bounce (speaking) */
static int16_t copilot_wave_offset(uint32_t now, bool speaking) {
    if (speaking) {
        uint32_t t = now % 340;
        int16_t a = (int16_t)(t * 3600u / 340u);
        int32_t sv = lv_trigo_sin(a);
        return (int16_t)((sv * 12) >> LV_TRIGO_SHIFT);   // ±12 px
    } else {
        uint32_t t = now % 2600;
        int16_t a = (int16_t)(t * 3600u / 2600u);
        int32_t sv = lv_trigo_sin(a);
        return (int16_t)((sv * 2) >> LV_TRIGO_SHIFT);    // ±2 px
    }
}

/* Mouth openness pulse — 4‑step rhythm when speaking */
static uint8_t copilot_speaking_pulse(uint32_t now) {
    switch ((now / 80) & 3) {
        case 0: return 255;
        case 1: return 90;
        case 2: return 210;
        default: return 55;
    }
}

/*
 * Idle demo: discrete angle cycling  (front → 3/4L → full‑L → 3/4L → front → …)
 * Speaking: face forward + micro‑wobble.
 */
static int16_t copilot_face_angle_target(uint32_t now, bool speaking) {
    if (speaking) {
        uint32_t t = now % 280;
        int16_t a = (int16_t)(t * 3600u / 280u);
        int32_t sv = lv_trigo_sin(a);
        return (int16_t)((sv * 12) >> LV_TRIGO_SHIFT);
    }
    static const int16_t angles[] = {
        0, (45 << FP_SHIFT), (90 << FP_SHIFT), (45 << FP_SHIFT),
        0, (-45 << FP_SHIFT), (-90 << FP_SHIFT), (-45 << FP_SHIFT),
    };
    const int n = sizeof(angles) / sizeof(angles[0]);
    uint32_t seg = (now / 1400) % n;
    return angles[seg];
}

static void copilot_apply_motion_transform(void) {
    int16_t ax = s_ui.motion_current.ax;
    int16_t ay = s_ui.motion_current.ay;
    int16_t yaw = s_ui.motion_current.yaw_deg;
    if (ax > FP_ONE) ax = FP_ONE;
    if (ax < -FP_ONE) ax = -FP_ONE;
    if (ay > FP_ONE) ay = FP_ONE;
    if (ay < -FP_ONE) ay = -FP_ONE;
    int16_t max_shift = CONFIG_COPILOT_MOTION_MAX_SHIFT_PX;
    int16_t shift_x = (int16_t)(((int32_t)ay * max_shift * MOTION_SHIFT_X_SIGN) >> FP_SHIFT);
    int16_t shift_y = (int16_t)(((int32_t)ax * max_shift * MOTION_SHIFT_Y_SIGN) >> FP_SHIFT);
    int16_t max_angle = CONFIG_COPILOT_MOTION_MAX_ANGLE_DEG;
    if (max_angle < 1) max_angle = 1;
    int16_t max_angle_fp = max_angle << FP_SHIFT;
    if (yaw > max_angle_fp) yaw = max_angle_fp;
    if (yaw < -max_angle_fp) yaw = -max_angle_fp;
    s_ui.motion_roll = (int16_t)(((int32_t)yaw * MOTION_YAW_SIGN * FP_ONE) / max_angle_fp);
    if (s_ui.motion_roll > FP_ONE) s_ui.motion_roll = FP_ONE;
    if (s_ui.motion_roll < -FP_ONE) s_ui.motion_roll = -FP_ONE;
    lv_obj_set_pos(s_ui.face_root, FACE_BOX_X + shift_x, FACE_BOX_Y + shift_y);
}

static bool copilot_update_motion(void) {
    int16_t old_ax = s_ui.motion_current.ax;
    int16_t old_ay = s_ui.motion_current.ay;
    int16_t old_yaw = s_ui.motion_current.yaw_deg;
    s_ui.motion_current.ax += (int16_t)(((int32_t)(s_ui.motion_target.ax - s_ui.motion_current.ax) * MOTION_ALPHA) >> FP_SHIFT);
    s_ui.motion_current.ay += (int16_t)(((int32_t)(s_ui.motion_target.ay - s_ui.motion_current.ay) * MOTION_ALPHA) >> FP_SHIFT);
    s_ui.motion_current.yaw_deg += (int16_t)(((int32_t)(s_ui.motion_target.yaw_deg - s_ui.motion_current.yaw_deg) * MOTION_ALPHA) >> FP_SHIFT);
    s_ui.motion_current.speed += (int16_t)(((int32_t)(s_ui.motion_target.speed - s_ui.motion_current.speed) * MOTION_ALPHA) >> FP_SHIFT);
    bool changed = copilot_abs_i16((int16_t)(s_ui.motion_current.ax - old_ax)) > 2 ||
                   copilot_abs_i16((int16_t)(s_ui.motion_current.ay - old_ay)) > 2 ||
                   copilot_abs_i16((int16_t)(s_ui.motion_current.yaw_deg - old_yaw)) > 2;
    if (changed) copilot_apply_motion_transform();
    return changed;
}

/*
 * 3D face renderer — rich grayscale cartoon with depth shading.
 *
 * face_t ∈ [-90°, +90°] Q8.8  →  face_t_norm ∈ [-1, +1]
 */
static void copilot_apply_head(uint32_t now, bool speaking) {
    /* —— mouth progress (smoothed) —— */
    int16_t target = speaking ? 255 : 0;
    int16_t delta = (int16_t)target - (int16_t)s_ui.mouth_progress;
    uint8_t alpha = speaking ? 96 : 52;
    s_ui.mouth_progress = (uint8_t)((int16_t)s_ui.mouth_progress + ((delta * alpha) >> 8));
    uint8_t state_open = s_ui.mouth_progress;
    uint8_t mouth_open = speaking ? (uint8_t)(((uint16_t)state_open * copilot_speaking_pulse(now)) / 255) : state_open;

    /* —— face angle (smoothed with overshoot) —— */
    int16_t angle_target = copilot_face_angle_target(now, speaking);
    s_ui.face_angle_prev = s_ui.face_angle;
    int16_t angle_delta = angle_target - s_ui.face_angle;
    // Overshoot: add a fraction of rate‑of‑change for cartoon springiness
    int16_t angle_velocity = s_ui.face_angle - s_ui.face_angle_prev;
    int16_t angle_force = angle_delta + (angle_velocity / 4);
    s_ui.face_angle += (int16_t)(((int32_t)angle_force * FACE_ANGLE_SMOOTH) >> FP_SHIFT);

    int16_t face_t = s_ui.face_angle;
    int16_t face_t_norm = (int16_t)(((int32_t)face_t * FP_ONE) / (90 * FP_ONE));
    int16_t abs_t = copilot_abs_i16(face_t_norm);

    int16_t wobble = copilot_wave_offset(now, speaking);
    int16_t roll  = (int16_t)((s_ui.motion_roll * 5) >> FP_SHIFT);
    // Exaggerated squash: wider & stronger
    int16_t squash = speaking ? (mouth_open / 60) : 0;

    /* —— Head: dark filled oval + mid‑tone outline + drop shadow —— */
    int16_t head_w_scale = FP_ONE - (abs_t * 24 / 100);
    int16_t head_w = (HEAD_W * head_w_scale) >> FP_SHIFT;
    int16_t head_h = HEAD_H - squash;
    int16_t head_x = HEAD_X + ((HEAD_W - head_w) / 2) + roll - squash / 2;
    int16_t head_y = HEAD_Y + wobble;
    lv_obj_set_size(s_ui.head, head_w, head_h);
    lv_obj_set_pos(s_ui.head, head_x, head_y);
    lv_obj_set_style_bg_opa(s_ui.head, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    int16_t hcx = head_x + head_w / 2;
    int16_t hcy = head_y + head_h / 2;

    /* —— Eyes —— */
    int16_t eye_base_y = hcy - (head_h * 12) / 100 + wobble / 2;
    int16_t eye_gap = EYE_GAP_REF;
    int16_t eye_l_cx = hcx - (eye_gap / 2) + ((face_t_norm * (eye_gap * 15 / 100)) >> FP_SHIFT);
    int16_t eye_r_cx = hcx + (eye_gap / 2) + ((face_t_norm * (eye_gap * 15 / 100)) >> FP_SHIFT);

    // Far eye shrinks to nothing at full profile
    int16_t far_scale = FP_ONE - ((abs_t * 100) >> FP_SHIFT);
    if (far_scale < 0) far_scale = 0;
    bool far_visible = far_scale > (FP_ONE * 8 / 100);

    // Near eye gets slightly BIGGER when speaking (exaggerated expression)
    int16_t eye_near_w = EYE_W_REF + (speaking ? 3 : 0);
    int16_t eye_near_h = EYE_H_REF - (speaking ? (mouth_open / 14) : 0);
    if (eye_near_h < 7) eye_near_h = 7;
    int16_t eye_far_w = far_visible ? ((EYE_W_REF * far_scale) >> FP_SHIFT) : 0;
    int16_t eye_far_h = far_visible ? ((eye_near_h * far_scale) >> FP_SHIFT) : 0;
    if (eye_far_h < 4 && far_visible) eye_far_h = 4;
    if (eye_far_w < 3 && far_visible) eye_far_w = 3;

    bool left_near = face_t_norm >= 0;
    int16_t eye_l_w = left_near ? eye_near_w : eye_far_w;
    int16_t eye_l_h = left_near ? eye_near_h : eye_far_h;
    int16_t eye_r_w = left_near ? eye_far_w : eye_near_w;
    int16_t eye_r_h = left_near ? eye_far_h : eye_near_h;

    int16_t eye_l_x = eye_l_cx - eye_l_w / 2;
    int16_t eye_l_y = eye_base_y - eye_l_h / 2 + (left_near ? 0 : (wobble / 4));
    int16_t eye_r_x = eye_r_cx - eye_r_w / 2;
    int16_t eye_r_y = eye_base_y - eye_r_h / 2 + (left_near ? (wobble / 4) : 0);

    // Set eye positions
    lv_obj_set_size(s_ui.eye_l, eye_l_w > 0 ? eye_l_w : 1, eye_l_h > 0 ? eye_l_h : 1);
    lv_obj_set_pos(s_ui.eye_l, eye_l_x, eye_l_y);
    lv_obj_set_size(s_ui.eye_r, eye_r_w > 0 ? eye_r_w : 1, eye_r_h > 0 ? eye_r_h : 1);
    lv_obj_set_pos(s_ui.eye_r, eye_r_x, eye_r_y);
    if (eye_l_w <= 3) lv_obj_add_flag(s_ui.eye_l, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_clear_flag(s_ui.eye_l, LV_OBJ_FLAG_HIDDEN);
    if (eye_r_w <= 3) lv_obj_add_flag(s_ui.eye_r, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_clear_flag(s_ui.eye_r, LV_OBJ_FLAG_HIDDEN);

    /* —— Pupils + catchlights —— */
    int16_t pupil_l_w = (eye_l_w * 24) / 100; if (pupil_l_w < 3) pupil_l_w = 3;
    int16_t pupil_l_h = (eye_l_h * 36) / 100; if (pupil_l_h < 5) pupil_l_h = 5;
    lv_obj_set_size(s_ui.eye_l_cut, pupil_l_w, pupil_l_h);
    lv_obj_set_pos(s_ui.eye_l_cut,
                   (eye_l_w - pupil_l_w) * 55 / 100, (eye_l_h - pupil_l_h) * 38 / 100);
    int16_t spk_l_w = pupil_l_w * 45 / 100; if (spk_l_w < 2) spk_l_w = 2;
    int16_t spk_l_h = pupil_l_h * 22 / 100; if (spk_l_h < 2) spk_l_h = 2;
    lv_obj_set_size(s_ui.eye_l_spark, spk_l_w, spk_l_h);
    lv_obj_set_pos(s_ui.eye_l_spark,
                   (eye_l_w - pupil_l_w) * 52 / 100, (eye_l_h - pupil_l_h) * 30 / 100);

    int16_t pupil_r_w = (eye_r_w * 24) / 100; if (pupil_r_w < 3) pupil_r_w = 3;
    int16_t pupil_r_h = (eye_r_h * 36) / 100; if (pupil_r_h < 5) pupil_r_h = 5;
    lv_obj_set_size(s_ui.eye_r_cut, pupil_r_w, pupil_r_h);
    lv_obj_set_pos(s_ui.eye_r_cut,
                   (eye_r_w - pupil_r_w) * 55 / 100, (eye_r_h - pupil_r_h) * 38 / 100);
    int16_t spk_r_w = pupil_r_w * 45 / 100; if (spk_r_w < 2) spk_r_w = 2;
    int16_t spk_r_h = pupil_r_h * 22 / 100; if (spk_r_h < 2) spk_r_h = 2;
    lv_obj_set_size(s_ui.eye_r_spark, spk_r_w, spk_r_h);
    lv_obj_set_pos(s_ui.eye_r_spark,
                   (eye_r_w - pupil_r_w) * 52 / 100, (eye_r_h - pupil_r_h) * 30 / 100);

    /* —— Cheek blush (warm gray, softens the face) —— */
    int16_t cheek_l_cx = eye_l_cx + (left_near ? 6 : -4);
    int16_t cheek_l_cy = eye_base_y + (EYE_H_REF * 48) / 100;
    int16_t cheek_r_cx = eye_r_cx + (left_near ? -4 : 6);
    int16_t cheek_r_cy = eye_base_y + (EYE_H_REF * 48) / 100;
    // Cheeks puff slightly when speaking
    int16_t cheek_scale = FP_ONE + (speaking ? (mouth_open * 20 / 255) : 0);
    int16_t ck_w = (CHEEK_W * cheek_scale) >> FP_SHIFT;
    int16_t ck_h = (CHEEK_H * cheek_scale) >> FP_SHIFT;
    lv_obj_set_size(s_ui.cheek_l, ck_w, ck_h);
    lv_obj_set_pos(s_ui.cheek_l, cheek_l_cx - ck_w / 2, cheek_l_cy - ck_h / 2);
    lv_obj_set_size(s_ui.cheek_r, ck_w, ck_h);
    lv_obj_set_pos(s_ui.cheek_r, cheek_r_cx - ck_w / 2, cheek_r_cy - ck_h / 2);
    // Hide far‑side cheek at full profile
    lv_opa_t cheek_l_opa = (left_near || abs_t < (FP_ONE * 60 / 100)) ? 200 : 0;
    lv_opa_t cheek_r_opa = (!left_near || abs_t < (FP_ONE * 60 / 100)) ? 200 : 0;
    lv_obj_set_style_bg_opa(s_ui.cheek_l, cheek_l_opa, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(s_ui.cheek_r, cheek_r_opa, LV_PART_MAIN | LV_STATE_DEFAULT);
    if (cheek_l_opa == 0) lv_obj_add_flag(s_ui.cheek_l, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_clear_flag(s_ui.cheek_l, LV_OBJ_FLAG_HIDDEN);
    if (cheek_r_opa == 0) lv_obj_add_flag(s_ui.cheek_r, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_clear_flag(s_ui.cheek_r, LV_OBJ_FLAG_HIDDEN);

    /* —— Nose tip (light gray highlight) —— */
    int16_t nose_shift = (int16_t)(((int32_t)face_t_norm * (head_w * 32 / 100)) >> FP_SHIFT);
    int16_t nose_x = hcx + nose_shift - NOSE_W / 2;
    int16_t nose_y = hcy + (head_h * 4) / 100 - NOSE_H / 2;
    lv_obj_set_size(s_ui.nose, NOSE_W, NOSE_H);
    lv_obj_set_pos(s_ui.nose, nose_x, nose_y);

    /* —— Nose bridge (visible in 3/4 and profile) —— */
    if (abs_t > (FP_ONE * 25 / 100)) {
        int16_t br_opacity = ((abs_t - (FP_ONE / 4)) * 4) >> FP_SHIFT;
        int16_t br_w = 4;
        int16_t br_h = ((head_h * 25) / 100) * br_opacity >> FP_SHIFT;
        int16_t br_x = hcx + ((face_t_norm * (head_w * 36 / 100)) >> FP_SHIFT) - br_w / 2;
        int16_t br_y = hcy - (head_h * 8) / 100 - br_h / 2;
        lv_obj_clear_flag(s_ui.face_patch, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_size(s_ui.face_patch, br_w, br_h > 2 ? br_h : 2);
        lv_obj_set_pos(s_ui.face_patch, br_x, br_y);
    } else {
        lv_obj_add_flag(s_ui.face_patch, LV_OBJ_FLAG_HIDDEN);
    }

    /* —— Mouth (dark pill → open dark oval + white tooth) —— */
    int16_t mouth_w = MOUTH_IDLE_W + (int16_t)(((MOUTH_OPEN_W - MOUTH_IDLE_W) * mouth_open) >> 8);
    int16_t mouth_h = MOUTH_IDLE_H + (int16_t)(((MOUTH_OPEN_H - MOUTH_IDLE_H) * mouth_open) >> 8);
    int16_t mouth_shift = (int16_t)(((int32_t)face_t_norm * (head_w * 8 / 100)) >> FP_SHIFT);
    int16_t mouth_x = hcx + mouth_shift - mouth_w / 2;
    int16_t mouth_y = hcy + (head_h * 22) / 100 - mouth_h / 2;
    lv_obj_set_size(s_ui.mouth, mouth_w, mouth_h);
    lv_obj_set_pos(s_ui.mouth, mouth_x, mouth_y);

    if (mouth_open > 25) {
        int16_t pad = 4 + ((int16_t)mouth_open / 38);
        lv_obj_clear_flag(s_ui.mouth_inner, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_size(s_ui.mouth_inner, mouth_w - pad * 2, mouth_h - pad * 2);
        lv_obj_set_pos(s_ui.mouth_inner, pad, pad);
        lv_obj_clear_flag(s_ui.tooth, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_size(s_ui.tooth, mouth_w - pad * 3, 3);
        lv_obj_set_pos(s_ui.tooth, pad * 3 / 2, pad);
    } else {
        lv_obj_add_flag(s_ui.mouth_inner, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_ui.tooth, LV_OBJ_FLAG_HIDDEN);
    }

#if CONFIG_COPILOT_SERVO_ENABLE
    if (!copilot_servo_get_manual()) {
        // Drive servos from face angle (front-left-up frame)
        float face_deg = (float)s_ui.face_angle / 256.0f;  // Q8.8 → degrees
        float wobble_deg = -(float)wobble / 4.0f;           // px → approx deg
        float yaw_s   = face_deg   * (CONFIG_COPILOT_SERVO_YAW_SCALE   / 100.0f);
        float pitch_s = wobble_deg * (CONFIG_COPILOT_SERVO_PITCH_SCALE / 100.0f);
        copilot_servo_set_target(pitch_s, yaw_s);
    }
#endif
}

static bool copilot_inside_circle(lv_coord_t x, lv_coord_t y) {
    int32_t dx = (int32_t)x - (LCD_H_RES / 2);
    int32_t dy = (int32_t)y - (LCD_V_RES / 2);
    int32_t r = (LCD_H_RES / 2) - 2;
    return (dx * dx + dy * dy) <= (r * r);
}

static void copilot_anim_timer(lv_timer_t *timer) {
    (void)timer;
    uint32_t now = lv_tick_get();
    copilot_perf_frame_tick();

    bool motion_changed = copilot_update_motion();

    if (s_ui.touch_flash_active && now >= s_ui.touch_flash_until_ms) {
        s_ui.touch_flash_active = false;
        if (s_ui.touch_flash_owns_ring) {
            s_ui.touch_flash_owns_ring = false;
            copilot_ui_ring_show(false);
        }
    }

    // Auto‑speak demo cycle
    uint32_t speak_cycle = now % 5600;
    bool auto_speak = speak_cycle < 1600;
    if (auto_speak && !s_ui.manual_speaking)
        copilot_ui_set_expression(COPILOT_EXPR_SPEAKING, 0);
    else if (!auto_speak && s_ui.manual_speaking && s_ui.manual_speaking_latched)
        copilot_ui_set_expression(COPILOT_EXPR_IDLE, 0);

    bool speaking = copilot_is_speaking(now);
    bool speaking_changed = speaking != s_ui.last_speaking;
    if (speaking_changed) {
        s_ui.last_speaking = speaking;
        if (speaking) copilot_ui_ring_show(true);
        else if (!s_ui.touch_flash_active) copilot_ui_ring_show(false);
    }

    bool mouth_animating = (speaking && s_ui.mouth_progress < 250) ||
                           (!speaking && s_ui.mouth_progress > 4);
    bool angle_moving = copilot_abs_i16(copilot_face_angle_target(now, speaking) - s_ui.face_angle) > 8;

    if (speaking || speaking_changed || mouth_animating || motion_changed || angle_moving || !speaking) {
        copilot_apply_head(now, speaking);
    }
}

void copilot_ui_init(lv_obj_t *root) {
    if (!root || s_ui.ready) return;
    memset(&s_ui, 0, sizeof(s_ui));
    lv_obj_clear_flag(root, LV_OBJ_FLAG_SCROLLABLE);

    lv_color_t c_bg      = lv_color_hex(C_BG);
    lv_color_t c_head    = lv_color_hex(C_HEAD);
    lv_color_t c_outline = lv_color_hex(C_OUTLINE);
    lv_color_t c_eye     = lv_color_hex(C_EYE);
    lv_color_t c_pupil   = lv_color_hex(C_BG);
    lv_color_t c_spark   = lv_color_hex(C_SPARK);
    lv_color_t c_cheek   = lv_color_hex(C_CHEEK);
    lv_color_t c_nose    = lv_color_hex(C_NOSE);
    lv_color_t c_shadow  = lv_color_hex(C_SHADOW);
    lv_color_t c_mouth_inner = lv_color_hex(C_MOUTH_INNER);

    s_ui.root = lv_obj_create(root);
    lv_obj_set_size(s_ui.root, LCD_H_RES, LCD_V_RES);
    lv_obj_set_pos(s_ui.root, 0, 0);
    lv_obj_set_scrollbar_mode(s_ui.root, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(s_ui.root, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(s_ui.root, c_bg, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(s_ui.root, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(s_ui.root, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    int16_t outer_radius = (int16_t)((RING_SIZE / 2) - (RING_OUTER_WIDTH / 2));
    copilot_build_ring_points(s_ring_outer_pts, outer_radius);
    copilot_build_ring_points(s_ring_inner_pts, outer_radius);

    s_ui.ring_outer = lv_line_create(s_ui.root);
    lv_obj_set_size(s_ui.ring_outer, RING_SIZE, RING_SIZE);
    lv_obj_set_pos(s_ui.ring_outer, RING_X, RING_Y);
    lv_line_set_points(s_ui.ring_outer, s_ring_outer_pts, RING_POINTS);
    copilot_line_style(s_ui.ring_outer, lv_color_hex(0x1AB5FF), RING_OUTER_WIDTH);
    lv_obj_add_flag(s_ui.ring_outer, LV_OBJ_FLAG_HIDDEN);
    copilot_ring_set_opa(s_ui.ring_outer, 0);

    s_ui.ring_inner = lv_line_create(s_ui.root);
    lv_obj_set_size(s_ui.ring_inner, RING_SIZE, RING_SIZE);
    lv_obj_set_pos(s_ui.ring_inner, RING_X, RING_Y);
    lv_line_set_points(s_ui.ring_inner, s_ring_inner_pts, RING_POINTS);
    copilot_line_style(s_ui.ring_inner, lv_color_hex(0x61E6FF), RING_INNER_WIDTH);
    lv_obj_add_flag(s_ui.ring_inner, LV_OBJ_FLAG_HIDDEN);
    copilot_ring_set_opa(s_ui.ring_inner, 0);

    s_ui.face_root = lv_obj_create(s_ui.root);
    lv_obj_set_size(s_ui.face_root, FACE_BOX_SIZE, FACE_BOX_SIZE);
    lv_obj_set_pos(s_ui.face_root, FACE_BOX_X, FACE_BOX_Y);
    lv_obj_set_scrollbar_mode(s_ui.face_root, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(s_ui.face_root, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(s_ui.face_root, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(s_ui.face_root, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all(s_ui.face_root, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    /* Creation order (back → front) */
    s_ui.head       = lv_obj_create(s_ui.face_root);
    s_ui.face_patch = lv_obj_create(s_ui.face_root);
    s_ui.cheek_r    = lv_obj_create(s_ui.face_root);
    s_ui.cheek_l    = lv_obj_create(s_ui.face_root);
    s_ui.eye_r      = lv_obj_create(s_ui.face_root);
    s_ui.eye_l      = lv_obj_create(s_ui.face_root);
    s_ui.nose       = lv_obj_create(s_ui.face_root);
    s_ui.mouth      = lv_obj_create(s_ui.face_root);
    s_ui.eye_l_cut   = lv_obj_create(s_ui.eye_l);
    s_ui.eye_l_spark = lv_obj_create(s_ui.eye_l);
    s_ui.eye_r_cut   = lv_obj_create(s_ui.eye_r);
    s_ui.eye_r_spark = lv_obj_create(s_ui.eye_r);
    s_ui.mouth_inner = lv_obj_create(s_ui.mouth);
    s_ui.tooth       = lv_obj_create(s_ui.mouth);

    /* —— Styling —— */

    /* Head: dark fill + mid‑tone outline → solid 3D presence */
    copilot_blob_outline(s_ui.head, c_head, c_outline, HEAD_OUTLINE);

    /* Nose bridge: mid‑gray, hidden initially */
    copilot_blob_fill(s_ui.face_patch, c_outline);
    lv_obj_add_flag(s_ui.face_patch, LV_OBJ_FLAG_HIDDEN);

    /* Cheeks: warm gray blush, semi‑transparent */
    copilot_blob_fill(s_ui.cheek_l, c_cheek);
    copilot_blob_fill(s_ui.cheek_r, c_cheek);

    /* Eyes: bright white capsules */
    copilot_blob_fill(s_ui.eye_l, c_eye);
    copilot_blob_fill(s_ui.eye_r, c_eye);
    /* Pupils: pure black */
    copilot_blob_fill(s_ui.eye_l_cut, c_pupil);
    copilot_blob_fill(s_ui.eye_r_cut, c_pupil);
    /* Catchlights: purest white */
    copilot_blob_fill(s_ui.eye_l_spark, c_spark);
    copilot_blob_fill(s_ui.eye_r_spark, c_spark);

    /* Nose: light gray highlight */
    copilot_blob_fill(s_ui.nose, c_nose);

    /* Mouth: dark shadow pill / open mouth with deeper inner */
    copilot_blob_fill(s_ui.mouth, c_shadow);
    copilot_blob_fill(s_ui.mouth_inner, c_mouth_inner);
    copilot_blob_fill(s_ui.tooth, c_spark);

    s_ui.motion_target.speed = FP_ONE;
    s_ui.motion_current.speed = FP_ONE;
    copilot_apply_head(lv_tick_get(), false);

    s_ui.anim_timer = lv_timer_create(copilot_anim_timer, ANIM_TIMER_MS, nullptr);
    s_ui.ready = true;

    LOGI_UI("UI ready: rich grayscale rubber-hose face, 50fps");
}

bool copilot_ui_is_ready(void) { return s_ui.ready; }

void copilot_ui_set_expression(copilot_expr_t expr, uint32_t duration_ms) {
    if (!s_ui.ready || expr >= COPILOT_EXPR_COUNT) return;
    uint32_t now = lv_tick_get();
    if (expr == COPILOT_EXPR_SPEAKING) {
        s_ui.manual_speaking = true;
        if (duration_ms > 0) {
            s_ui.manual_speaking_latched = false;
            s_ui.manual_speaking_until_ms = now + duration_ms;
        } else {
            s_ui.manual_speaking_latched = true;
        }
        LOGI_UI("Expression speaking duration=%u", (unsigned)duration_ms);
    } else {
        s_ui.manual_speaking = false;
        s_ui.manual_speaking_latched = false;
        s_ui.manual_speaking_until_ms = 0;
        LOGI_UI("Expression idle");
    }
}

void copilot_ui_set_motion(const copilot_motion_t *motion) {
    if (!s_ui.ready || !motion) return;
#ifdef CONFIG_COPILOT_MOTION_SOURCE_DISABLED
    return;
#endif
    s_ui.motion_target = *motion;
    ESP_LOGD(TAG, "Motion target ax=%d ay=%d yaw=%d speed=%d (Q8.8)",
             motion->ax, motion->ay, motion->yaw_deg, motion->speed);
}

void copilot_ui_ring_show(bool on) {
    if (!s_ui.ready) return;
    if (on && s_ui.touch_flash_active && s_ui.touch_flash_owns_ring)
        s_ui.touch_flash_owns_ring = false;
    if (on == s_ui.ring_visible) return;
    s_ui.ring_visible = on;
    LOGI_UI("Ring show=%s", on ? "on" : "off");
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
    if (!s_ui.ready || !copilot_inside_circle(x, y)) return;
    uint32_t now = lv_tick_get();
    if (now - s_ui.last_touch_ms < 300) return;
    s_ui.last_touch_ms = now;
    LOGI_UI("Touch x=%u y=%u", (unsigned)x, (unsigned)y);
    s_ui.touch_flash_until_ms = now + TOUCH_FLASH_MS;
    if (!s_ui.touch_flash_active) {
        s_ui.touch_flash_active = true;
        s_ui.touch_flash_owns_ring = !s_ui.ring_visible;
        if (s_ui.touch_flash_owns_ring) copilot_ui_ring_show(true);
    }
}

void copilot_ui_set_expression_async(copilot_expr_t expr, uint32_t duration_ms) {
    if (!s_ui.ready) return;
    if (bsp_display_lock(0)) { copilot_ui_set_expression(expr, duration_ms); bsp_display_unlock(); }
}

void copilot_ui_set_motion_async(const copilot_motion_t *motion) {
    if (!s_ui.ready || !motion) return;
    if (bsp_display_lock(0)) { copilot_ui_set_motion(motion); bsp_display_unlock(); }
}

void copilot_ui_set_motion_only_async(const copilot_motion_t *motion) {
    if (!s_ui.ready || !motion) return;
    if (bsp_display_lock(0)) {
        s_ui.motion_target = *motion;
        ESP_LOGD(TAG, "Motion only ax=%d ay=%d yaw=%d (Q8.8)",
                 motion->ax, motion->ay, motion->yaw_deg);
        bsp_display_unlock();
    }
}

void copilot_ui_ring_show_async(bool on) {
    if (!s_ui.ready) return;
    if (bsp_display_lock(0)) { copilot_ui_ring_show(on); bsp_display_unlock(); }
}
