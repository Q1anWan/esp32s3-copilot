#include "copilot_ui.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "sdkconfig.h"
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "copilot_perf.h"
#include "copilot_voice_ui.h"

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

#define ANIM_TIMER_MS 33
#define TOUCH_FLASH_MS 180
#define SPEAKING_HOLD_MS 120

#define INK_COLOR 0x000000
#define PAPER_COLOR 0xF8FAFF

#define HEAD_W ((FACE_BOX_SIZE * 70) / 100)
#define HEAD_H ((FACE_BOX_SIZE * 74) / 100)
#define HEAD_X ((FACE_BOX_SIZE - HEAD_W) / 2)
#define HEAD_Y ((FACE_BOX_SIZE * 8) / 100)
#define HEAD_BORDER ((FACE_BOX_SIZE * 3) / 100)

#define CHEEK_W ((FACE_BOX_SIZE * 26) / 100)
#define CHEEK_H ((FACE_BOX_SIZE * 28) / 100)
#define FACE_PATCH_W ((FACE_BOX_SIZE * 48) / 100)
#define FACE_PATCH_H ((FACE_BOX_SIZE * 45) / 100)

#define EYE_W ((FACE_BOX_SIZE * 12) / 100)
#define EYE_H ((FACE_BOX_SIZE * 21) / 100)
#define EYE_CUT_W ((FACE_BOX_SIZE * 5) / 100)
#define EYE_CUT_H ((FACE_BOX_SIZE * 12) / 100)
#define EYE_SPARK_W ((FACE_BOX_SIZE * 3) / 100)
#define EYE_SPARK_H ((FACE_BOX_SIZE * 5) / 100)

#define NOSE_W ((FACE_BOX_SIZE * 7) / 100)
#define NOSE_H ((FACE_BOX_SIZE * 5) / 100)

#define MOUTH_IDLE_W ((FACE_BOX_SIZE * 24) / 100)
#define MOUTH_IDLE_H ((FACE_BOX_SIZE * 5) / 100)
#define MOUTH_OPEN_W ((FACE_BOX_SIZE * 23) / 100)
#define MOUTH_OPEN_H ((FACE_BOX_SIZE * 24) / 100)

#define MOTION_SHIFT_X_SIGN (-1)
#define MOTION_SHIFT_Y_SIGN (1)
#define MOTION_YAW_SIGN     (-1)
#define MOTION_ALPHA 46

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

static int16_t copilot_abs_i16(int16_t v) {
    return v < 0 ? (int16_t)-v : v;
}

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

static void copilot_blob_style(lv_obj_t *obj, lv_color_t fill, lv_color_t border, uint8_t border_w) {
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
    if (!s_ui.manual_speaking) {
        return false;
    }
    if (s_ui.manual_speaking_latched) {
        return true;
    }
    if (now < s_ui.manual_speaking_until_ms) {
        return true;
    }
    s_ui.manual_speaking = false;
    return false;
}

static bool copilot_is_speaking(uint32_t now) {
    return copilot_voice_speaking(now) || copilot_manual_speaking(now);
}

static int16_t copilot_wave_offset(uint32_t now, bool speaking) {
    uint32_t phase = (now / (speaking ? 95 : 520)) & 3;
    int16_t amp = speaking ? 9 : 2;
    if (phase == 0) return amp;
    if (phase == 1) return amp / 2;
    if (phase == 2) return (int16_t)-amp;
    return (int16_t)(-amp / 2);
}

static uint8_t copilot_speaking_pulse(uint32_t now) {
    switch ((now / 85) & 3) {
        case 0: return 255;
        case 1: return 110;
        case 2: return 215;
        default: return 70;
    }
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
    if (changed) {
        copilot_apply_motion_transform();
    }
    return changed;
}

static void copilot_apply_head(uint32_t now, bool speaking) {
    int16_t target = speaking ? 255 : 0;
    int16_t delta = (int16_t)target - (int16_t)s_ui.mouth_progress;
    uint8_t alpha = speaking ? 96 : 52;
    s_ui.mouth_progress = (uint8_t)((int16_t)s_ui.mouth_progress + ((delta * alpha) >> 8));

    uint8_t state_open = s_ui.mouth_progress;
    uint8_t mouth_open = speaking ? (uint8_t)(((uint16_t)state_open * copilot_speaking_pulse(now)) / 255) : state_open;
    int16_t wobble = copilot_wave_offset(now, speaking);
    int16_t roll = (int16_t)((s_ui.motion_roll * 11) >> FP_SHIFT);
    int16_t squash = speaking ? (mouth_open / 38) : 0;

    int16_t head_w = HEAD_W + squash;
    int16_t head_h = HEAD_H - (squash / 2);
    int16_t head_x = HEAD_X + roll - (squash / 2);
    int16_t head_y = HEAD_Y + (wobble / 2) + (squash / 3);

    lv_obj_set_size(s_ui.cheek_l, CHEEK_W, CHEEK_H);
    lv_obj_set_size(s_ui.cheek_r, CHEEK_W, CHEEK_H);
    lv_obj_set_pos(s_ui.cheek_l, head_x - (CHEEK_W / 4), head_y + (HEAD_H * 42) / 100);
    lv_obj_set_pos(s_ui.cheek_r, head_x + head_w - ((CHEEK_W * 3) / 4), head_y + (HEAD_H * 42) / 100);

    lv_obj_set_size(s_ui.head, head_w, head_h);
    lv_obj_set_pos(s_ui.head, head_x, head_y);

    int16_t patch_w = FACE_PATCH_W + (squash / 2);
    int16_t patch_h = FACE_PATCH_H - (squash / 3);
    int16_t patch_x = head_x + (head_w - patch_w) / 2;
    int16_t patch_y = head_y + (HEAD_H * 39) / 100 - (squash / 4);
    lv_obj_set_size(s_ui.face_patch, patch_w, patch_h);
    lv_obj_set_pos(s_ui.face_patch, patch_x, patch_y);

    int16_t eye_y = head_y + (HEAD_H * 24) / 100 + (speaking ? wobble / 4 : 0);
    int16_t eye_gap = (HEAD_W * 18) / 100;
    int16_t eye_l_x = head_x + (head_w / 2) - eye_gap - EYE_W;
    int16_t eye_r_x = head_x + (head_w / 2) + eye_gap;
    lv_obj_set_size(s_ui.eye_l, EYE_W, EYE_H);
    lv_obj_set_size(s_ui.eye_r, EYE_W, EYE_H);
    lv_obj_set_pos(s_ui.eye_l, eye_l_x, eye_y);
    lv_obj_set_pos(s_ui.eye_r, eye_r_x, eye_y + (speaking ? 2 : 0));
    lv_obj_set_size(s_ui.eye_l_cut, EYE_CUT_W, EYE_CUT_H);
    lv_obj_set_size(s_ui.eye_r_cut, EYE_CUT_W, EYE_CUT_H);
    lv_obj_set_pos(s_ui.eye_l_cut, EYE_W - EYE_CUT_W + 1, (EYE_H - EYE_CUT_H) / 2);
    lv_obj_set_pos(s_ui.eye_r_cut, -1, (EYE_H - EYE_CUT_H) / 2);
    lv_obj_set_size(s_ui.eye_l_spark, EYE_SPARK_W, EYE_SPARK_H);
    lv_obj_set_size(s_ui.eye_r_spark, EYE_SPARK_W, EYE_SPARK_H);
    lv_obj_set_pos(s_ui.eye_l_spark, EYE_W / 4, EYE_H / 5);
    lv_obj_set_pos(s_ui.eye_r_spark, EYE_W / 4, EYE_H / 5);

    int16_t nose_x = patch_x + (patch_w - NOSE_W) / 2 + (roll / 4);
    int16_t nose_y = patch_y + (patch_h * 40) / 100 + (speaking ? wobble / 5 : 0);
    lv_obj_set_size(s_ui.nose, NOSE_W, NOSE_H);
    lv_obj_set_pos(s_ui.nose, nose_x, nose_y);

    int16_t mouth_w = MOUTH_IDLE_W + (int16_t)(((MOUTH_OPEN_W - MOUTH_IDLE_W) * mouth_open) >> 8);
    int16_t mouth_h = MOUTH_IDLE_H + (int16_t)(((MOUTH_OPEN_H - MOUTH_IDLE_H) * mouth_open) >> 8);
    int16_t mouth_x = patch_x + (patch_w - mouth_w) / 2;
    int16_t mouth_y = patch_y + (patch_h * 64) / 100 - (mouth_h / 2);
    lv_obj_set_size(s_ui.mouth, mouth_w, mouth_h);
    lv_obj_set_pos(s_ui.mouth, mouth_x, mouth_y);

    if (mouth_open > 28) {
        int16_t pad = 5 + ((int16_t)mouth_open / 44);
        lv_obj_clear_flag(s_ui.mouth_inner, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_size(s_ui.mouth_inner, mouth_w - (pad * 2), mouth_h - (pad * 2));
        lv_obj_set_pos(s_ui.mouth_inner, pad, pad);

        lv_obj_clear_flag(s_ui.tooth, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_size(s_ui.tooth, mouth_w - (pad * 3), 5);
        lv_obj_set_pos(s_ui.tooth, (pad * 3) / 2, pad);
    } else {
        lv_obj_add_flag(s_ui.mouth_inner, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_ui.tooth, LV_OBJ_FLAG_HIDDEN);
    }
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

    bool speaking = copilot_is_speaking(now);
    bool speaking_changed = speaking != s_ui.last_speaking;
    if (speaking_changed) {
        s_ui.last_speaking = speaking;
        if (speaking) {
            copilot_ui_ring_show(true);
        } else if (!s_ui.touch_flash_active) {
            copilot_ui_ring_show(false);
        }
    }

    bool mouth_animating = (speaking && s_ui.mouth_progress < 250) ||
                           (!speaking && s_ui.mouth_progress > 4);
    uint32_t idle_pose_bucket = now / 390;
    bool idle_pose_changed = !speaking && idle_pose_bucket != s_ui.last_idle_pose_bucket;
    if (idle_pose_changed) {
        s_ui.last_idle_pose_bucket = idle_pose_bucket;
    }
    if (speaking || speaking_changed || mouth_animating || motion_changed || idle_pose_changed) {
        copilot_apply_head(now, speaking);
    }
}

void copilot_ui_init(lv_obj_t *root) {
    if (!root || s_ui.ready) {
        return;
    }

    memset(&s_ui, 0, sizeof(s_ui));
    lv_obj_clear_flag(root, LV_OBJ_FLAG_SCROLLABLE);

    lv_color_t ink = lv_color_hex(INK_COLOR);
    lv_color_t paper = lv_color_hex(PAPER_COLOR);

    s_ui.root = lv_obj_create(root);
    lv_obj_set_size(s_ui.root, LCD_H_RES, LCD_V_RES);
    lv_obj_set_pos(s_ui.root, 0, 0);
    lv_obj_set_scrollbar_mode(s_ui.root, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(s_ui.root, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(s_ui.root, ink, LV_PART_MAIN | LV_STATE_DEFAULT);
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

    s_ui.cheek_l = lv_obj_create(s_ui.face_root);
    s_ui.cheek_r = lv_obj_create(s_ui.face_root);
    s_ui.head = lv_obj_create(s_ui.face_root);
    s_ui.face_patch = lv_obj_create(s_ui.face_root);
    s_ui.eye_l = lv_obj_create(s_ui.face_root);
    s_ui.eye_r = lv_obj_create(s_ui.face_root);
    s_ui.eye_l_cut = lv_obj_create(s_ui.eye_l);
    s_ui.eye_r_cut = lv_obj_create(s_ui.eye_r);
    s_ui.eye_l_spark = lv_obj_create(s_ui.eye_l);
    s_ui.eye_r_spark = lv_obj_create(s_ui.eye_r);
    s_ui.nose = lv_obj_create(s_ui.face_root);
    s_ui.mouth = lv_obj_create(s_ui.face_root);
    s_ui.mouth_inner = lv_obj_create(s_ui.mouth);
    s_ui.tooth = lv_obj_create(s_ui.mouth);

    copilot_blob_style(s_ui.cheek_l, ink, paper, HEAD_BORDER);
    copilot_blob_style(s_ui.cheek_r, ink, paper, HEAD_BORDER);
    copilot_blob_style(s_ui.head, ink, paper, HEAD_BORDER);
    copilot_blob_style(s_ui.face_patch, paper, ink, HEAD_BORDER / 2);
    copilot_blob_style(s_ui.eye_l, ink, ink, 0);
    copilot_blob_style(s_ui.eye_r, ink, ink, 0);
    copilot_blob_style(s_ui.eye_l_cut, paper, paper, 0);
    copilot_blob_style(s_ui.eye_r_cut, paper, paper, 0);
    copilot_blob_style(s_ui.eye_l_spark, paper, paper, 0);
    copilot_blob_style(s_ui.eye_r_spark, paper, paper, 0);
    copilot_blob_style(s_ui.nose, ink, ink, 0);
    copilot_blob_style(s_ui.mouth, ink, ink, 0);
    copilot_blob_style(s_ui.mouth_inner, paper, paper, 0);
    copilot_blob_style(s_ui.tooth, paper, paper, 0);

    s_ui.motion_target.speed = FP_ONE;
    s_ui.motion_current.speed = FP_ONE;
    copilot_apply_head(lv_tick_get(), false);

    s_ui.anim_timer = lv_timer_create(copilot_anim_timer, ANIM_TIMER_MS, nullptr);
    s_ui.ready = true;

    LOGI_UI("UI ready: rubber-hose head, states=idle/speaking");
}

bool copilot_ui_is_ready(void) {
    return s_ui.ready;
}

void copilot_ui_set_expression(copilot_expr_t expr, uint32_t duration_ms) {
    if (!s_ui.ready || expr >= COPILOT_EXPR_COUNT) {
        return;
    }

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
    if (!s_ui.ready || !motion) {
        return;
    }

#ifdef CONFIG_COPILOT_MOTION_SOURCE_DISABLED
    return;
#endif

    s_ui.motion_target = *motion;
    ESP_LOGD(TAG, "Motion target ax=%d ay=%d yaw=%d speed=%d (Q8.8)",
             motion->ax, motion->ay, motion->yaw_deg, motion->speed);
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
    if (!s_ui.ready || !copilot_inside_circle(x, y)) {
        return;
    }
    uint32_t now = lv_tick_get();
    if (now - s_ui.last_touch_ms < 300) {
        return;
    }
    s_ui.last_touch_ms = now;
    LOGI_UI("Touch x=%u y=%u", (unsigned)x, (unsigned)y);

    s_ui.touch_flash_until_ms = now + TOUCH_FLASH_MS;
    if (!s_ui.touch_flash_active) {
        s_ui.touch_flash_active = true;
        s_ui.touch_flash_owns_ring = !s_ui.ring_visible;
        if (s_ui.touch_flash_owns_ring) {
            copilot_ui_ring_show(true);
        }
    }
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
