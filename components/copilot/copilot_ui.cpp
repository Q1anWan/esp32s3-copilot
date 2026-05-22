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

/*
 * Speaking indicator: a large square-line halo around the face, placed inside
 * the circular screen's safe center area. It uses only a few line points and
 * switches instantly to avoid per-frame opacity redraws while audio is playing.
 */
#define RING_SIZE ((LCD_H_RES * 68) / 100)
#define RING_X ((LCD_H_RES - RING_SIZE) / 2)
#define RING_Y ((LCD_V_RES - RING_SIZE) / 2)
#define RING_POINTS 5
#define RING_OUTER_WIDTH 7
#define RING_INNER_WIDTH 3
#define RING_OPA 255

#define ANIM_TIMER_MS 16
#define UI_ASYNC_LOCK_MS 1000
#define TOUCH_FLASH_MS  180
#define SPEAKING_HOLD_MS 120

#define SCREEN_RETURN_MS 520
#define PRECUE_MIN_MS 300
#define PRECUE_MAX_MS 800
#define ORIENT_DEG 24
#define ROBOT_DISPLAY_VERSION "screen_copilot_v1.0"
#define ASSET_VERSION_HASH "face_rubberhose_balanced_20260514"

/*
 * Participant-facing palette: neutral dark screen with fixed soft cyan/white
 * accents. No red/orange/yellow semantic or alarm colors.
 */
#define C_BG      0x05090D
#define C_HEAD    0x16242C
#define C_OUTLINE 0xBDE7F3
#define C_SHADOW  0x0A1218
#define C_CHEEK   0x5F9FB0
#define C_EYE     0xEEF9FF
#define C_SPARK   0xFFFFFF
#define C_NOSE    0x9FD3DE
#define C_MOUTH_INNER 0x071015

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
#define MOUTH_IDLE_W ((FACE_BOX_SIZE * 18) / 100)
#define MOUTH_IDLE_H 4
#define MOUTH_OPEN_W ((FACE_BOX_SIZE * 25) / 100)
#define MOUTH_OPEN_H ((FACE_BOX_SIZE * 18) / 100)

#define MOTION_SHIFT_X_SIGN (-1)
#define MOTION_SHIFT_Y_SIGN (1)
#define MOTION_YAW_SIGN     (-1)
#define MOTION_ALPHA 46
#define FACE_ANGLE_SMOOTH 52
#define IDLE_HEAD_SWAY_DEG 9
#define IDLE_HEAD_SWAY_FAST_DEG 4
#define IDLE_BOB_PX 4
#define SPEAKING_BOB_PX 6
#define SPEAKING_SQUASH_MAX 11
#define EYE_LOOK_MAX_X 5
#define EYE_LOOK_MAX_Y 3

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
    int16_t face_angle_target;
    int16_t eye_look_x;
    int16_t eye_look_y;
    int16_t eye_look_target_x;
    int16_t eye_look_target_y;
    uint32_t eye_next_ms;
    uint32_t eye_rng;
    int8_t eye_side_balance;
    int8_t eye_last_side;
    uint8_t eye_same_side_streak;

    lv_timer_t *anim_timer;
    bool ring_visible;
    bool ring_objects_visible;
    uint8_t ring_opa_current;
    uint8_t ring_opa_target;
    uint8_t ring_opa_applied;
    bool ready;
    uint32_t last_touch_ms;
    bool touch_flash_active;
    bool touch_flash_owns_ring;
    uint32_t touch_flash_until_ms;

    copilot_screen_state_t screen_state;
    copilot_screen_orientation_t screen_orientation;
    uint32_t screen_state_started_ms;
    uint32_t screen_state_until_ms;
    uint32_t t_cue_ms;
    uint32_t t_speech_start_ms;
    uint32_t t_speech_end_ms;
    uint32_t event_seq;
    char message_id[32];
    bool calibration_content_present;
    bool visual_semantic_content_present;

    bool manual_speaking;
    bool manual_speaking_latched;
    uint32_t manual_speaking_until_ms;
    uint32_t speaking_until_ms;
    uint8_t mouth_progress;
    uint8_t mouth_audio_smooth;
    bool last_speaking;
    uint32_t last_idle_pose_bucket;
    uint32_t last_servo_update_ms;
    int16_t last_servo_angle;
    int16_t servo_idle_pitch;
    int16_t servo_idle_yaw;
    int16_t servo_idle_pitch_target;
    int16_t servo_idle_yaw_target;
    uint32_t servo_idle_next_ms;
};

static copilot_ui_state_t s_ui = {};
static lv_point_precise_t s_ring_outer_pts[RING_POINTS];
static lv_point_precise_t s_ring_inner_pts[RING_POINTS];

static int16_t copilot_abs_i16(int16_t v) { return v < 0 ? (int16_t)-v : v; }

static int16_t copilot_clamp_i16(int16_t v, int16_t lo, int16_t hi) {
    if (lo > hi) return lo;
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static uint8_t copilot_clamp_u8_i16(int16_t v, uint8_t lo, uint8_t hi) {
    if (v < (int16_t)lo) return lo;
    if (v > (int16_t)hi) return hi;
    return (uint8_t)v;
}

static int16_t copilot_approach_i16(int16_t current, int16_t target, uint8_t alpha) {
    int16_t delta = target - current;
    if (delta == 0) return current;
    int16_t step = (int16_t)(((int32_t)delta * alpha) >> FP_SHIFT);
    if (step == 0) step = delta > 0 ? 1 : -1;
    return current + step;
}

static int16_t copilot_wave_px(uint32_t now, uint32_t period_ms, int16_t amplitude, uint16_t phase_deg) {
    if (period_ms == 0 || amplitude == 0) return 0;
    uint32_t t = now % period_ms;
    int16_t angle = (int16_t)(((t * 360u) / period_ms) + phase_deg);
    int32_t sv = lv_trigo_sin(angle);
    return (int16_t)((sv * amplitude) >> LV_TRIGO_SHIFT);
}

static int16_t copilot_wave_deg_fp(uint32_t now, uint32_t period_ms, int16_t amplitude_deg, uint16_t phase_deg) {
    if (period_ms == 0 || amplitude_deg == 0) return 0;
    uint32_t t = now % period_ms;
    int16_t angle = (int16_t)(((t * 360u) / period_ms) + phase_deg);
    int32_t sv = lv_trigo_sin(angle);
    return (int16_t)((sv * amplitude_deg * FP_ONE) >> LV_TRIGO_SHIFT);
}

static uint32_t copilot_eye_rand(uint32_t now) {
    if (s_ui.eye_rng == 0) {
        s_ui.eye_rng = (now ^ 0xA5A55A5Au) + (s_ui.event_seq * 747796405u) + 2891336453u;
    }
    s_ui.eye_rng = s_ui.eye_rng * 1664525u + 1013904223u;
    return s_ui.eye_rng;
}

static int16_t copilot_pupil_axis_pos(int16_t eye_size, int16_t pupil_size, int16_t gaze) {
    int16_t travel = eye_size - pupil_size;
    if (travel <= 0) return 0;
    int16_t center = travel / 2;
    int16_t safe = travel / 3;
    if (safe < 1) safe = 1;
    int16_t pos = center + copilot_clamp_i16(gaze, (int16_t)-safe, safe);
    return copilot_clamp_i16(pos, 0, travel);
}

static int16_t copilot_pupil_size(int16_t eye_size, uint8_t percent, int16_t min_size) {
    if (eye_size <= 2) return 1;
    int16_t size = (eye_size * percent) / 100;
    if (size < min_size) size = min_size;
    int16_t max_size = eye_size - 2;
    if (max_size < 1) max_size = 1;
    if (size > max_size) size = max_size;
    return size;
}

static const char *copilot_screen_state_name(copilot_screen_state_t state) {
    switch (state) {
        case COPILOT_SCREEN_STATE_NEUTRAL_IDLE: return "neutral_idle";
        case COPILOT_SCREEN_STATE_PRE_MESSAGE_ORIENT: return "pre_message_orient";
        case COPILOT_SCREEN_STATE_SPEAKING: return "speaking";
        case COPILOT_SCREEN_STATE_RETURN_NEUTRAL: return "return_neutral";
        case COPILOT_SCREEN_STATE_SILENT_NEUTRAL: return "silent_neutral";
        case COPILOT_SCREEN_STATE_DEBUG: return "debug";
        default: return "unknown";
    }
}

static const char *copilot_screen_orientation_name(copilot_screen_orientation_t orientation) {
    switch (orientation) {
        case COPILOT_SCREEN_ORIENT_LEFT: return "left";
        case COPILOT_SCREEN_ORIENT_RIGHT: return "right";
        default: return "front";
    }
}

static int16_t copilot_orientation_angle(copilot_screen_orientation_t orientation) {
    switch (orientation) {
        case COPILOT_SCREEN_ORIENT_LEFT:
            return (int16_t)(ORIENT_DEG << FP_SHIFT);
        case COPILOT_SCREEN_ORIENT_RIGHT:
            return (int16_t)(-(ORIENT_DEG << FP_SHIFT));
        default:
            return 0;
    }
}

static uint32_t copilot_clamp_u32(uint32_t v, uint32_t lo, uint32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void copilot_log_screen_event(const char *phase) {
    ESP_LOGI(TAG,
             "screen_event {\"phase\":\"%s\",\"seq\":%lu,"
             "\"calibration_content_present\":%s,"
             "\"visual_semantic_content_present\":%s,"
             "\"screen_state\":\"%s\",\"screen_orientation\":\"%s\","
             "\"screen_text\":\"\",\"screen_icon\":\"\","
             "\"screen_color_profile\":\"fixed_soft_cyan_neutral\","
             "\"animation_profile\":\"%s\","
             "\"brightness_level\":\"low\","
             "\"t_state_start\":%lu,\"t_state_end\":%lu,"
             "\"t_cue\":%lu,\"t_speech_start\":%lu,\"t_speech_end\":%lu,"
             "\"audio_file_id\":\"%s\",\"sync_error_ms\":0,"
             "\"dropped_frame_count\":0,"
             "\"robot_display_version\":\"%s\","
             "\"asset_version_hash\":\"%s\"}",
             phase ? phase : "state",
             (unsigned long)s_ui.event_seq,
             s_ui.calibration_content_present ? "true" : "false",
             s_ui.visual_semantic_content_present ? "true" : "false",
             copilot_screen_state_name(s_ui.screen_state),
             copilot_screen_orientation_name(s_ui.screen_orientation),
             copilot_screen_state_name(s_ui.screen_state),
             (unsigned long)s_ui.screen_state_started_ms,
             (unsigned long)s_ui.screen_state_until_ms,
             (unsigned long)s_ui.t_cue_ms,
             (unsigned long)s_ui.t_speech_start_ms,
             (unsigned long)s_ui.t_speech_end_ms,
             s_ui.message_id,
             ROBOT_DISPLAY_VERSION,
             ASSET_VERSION_HASH);
}

static void copilot_build_ring_points(lv_point_precise_t *pts, int16_t inset) {
    if (!pts) return;
    inset = copilot_clamp_i16(inset, 1, (RING_SIZE / 2) - 1);
    const int16_t hi = (int16_t)(RING_SIZE - inset);

    pts[0].x = inset; pts[0].y = inset;
    pts[1].x = hi;    pts[1].y = inset;
    pts[2].x = hi;    pts[2].y = hi;
    pts[3].x = inset; pts[3].y = hi;
    pts[4].x = inset; pts[4].y = inset;
}

static void copilot_line_style(lv_obj_t *line, lv_color_t color, uint8_t width) {
    lv_obj_set_style_line_width(line, width, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_color(line, color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_opa(line, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_rounded(line, true, LV_PART_MAIN | LV_STATE_DEFAULT);
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

static void copilot_ring_set_visible(bool visible) {
    if (s_ui.ring_objects_visible == visible) {
        return;
    }
    s_ui.ring_objects_visible = visible;
    if (visible) {
        lv_obj_clear_flag(s_ui.ring_outer, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_ui.ring_inner, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_ui.ring_outer, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_ui.ring_inner, LV_OBJ_FLAG_HIDDEN);
    }
}

static void copilot_ring_apply_opa(uint8_t opa) {
    if (s_ui.ring_opa_applied == opa) {
        return;
    }
    s_ui.ring_opa_applied = opa;
    copilot_ring_set_opa(s_ui.ring_outer, (lv_opa_t)opa);
    copilot_ring_set_opa(s_ui.ring_inner, (lv_opa_t)opa);
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
    s_ui.manual_speaking_until_ms = 0;
    return false;
}

static bool copilot_is_speaking(uint32_t now) {
    return copilot_voice_speaking(now) || copilot_manual_speaking(now);
}

/* Rubber-hose mouth driver: exaggerated, fast, and still audio-reactive. */
static uint8_t copilot_mouth_target(uint32_t now, bool speaking) {
    if (!speaking) {
        return 0;
    }

    uint8_t env = copilot_voice_ui_get_mouth_open();
    if (env > 8) {
        int16_t delta = (int16_t)env - (int16_t)s_ui.mouth_audio_smooth;
        s_ui.mouth_audio_smooth = (uint8_t)((int16_t)s_ui.mouth_audio_smooth + ((delta * 140) >> 8));
        int16_t jitter = copilot_wave_px(now, 124, 42, 0) + copilot_wave_px(now, 67, 22, 90);
        int16_t shaped = 54 + ((int16_t)s_ui.mouth_audio_smooth * 190) / 255 + jitter;
        return copilot_clamp_u8_i16(shaped, 30, 250);
    }

    if (s_ui.mouth_audio_smooth > 0) {
        s_ui.mouth_audio_smooth = (uint8_t)(((uint16_t)s_ui.mouth_audio_smooth * 178u) >> 8);
    }

    int16_t pulse = 128;
    pulse += copilot_wave_px(now, 210, 78, 0);
    pulse += copilot_wave_px(now, 96, 44, 70);
    pulse += copilot_wave_px(now, 53, 24, 180);
    pulse += ((now / 72) & 1) ? 24 : -18;
    return copilot_clamp_u8_i16(pulse, 35, 245);
}

static int16_t copilot_face_angle_target(uint32_t now, bool speaking) {
    if (speaking) {
        return 0;
    }

    int16_t target = s_ui.face_angle_target;
    bool neutral_alive = s_ui.screen_state == COPILOT_SCREEN_STATE_NEUTRAL_IDLE ||
                         s_ui.screen_state == COPILOT_SCREEN_STATE_SILENT_NEUTRAL ||
                         s_ui.screen_state == COPILOT_SCREEN_STATE_RETURN_NEUTRAL;
    if (neutral_alive) {
        target += copilot_wave_deg_fp(now, 2600, IDLE_HEAD_SWAY_DEG, 0);
        target += copilot_wave_deg_fp(now, 1370, IDLE_HEAD_SWAY_FAST_DEG, 115);
    }
    return copilot_clamp_i16(target, (int16_t)(-(36 << FP_SHIFT)), (int16_t)(36 << FP_SHIFT));
}

static void copilot_update_eye_look(uint32_t now, bool speaking) {
    if (speaking) {
        s_ui.eye_look_target_x = 0;
        s_ui.eye_look_target_y = 0;
        s_ui.eye_next_ms = now + 260;
    } else if (s_ui.eye_next_ms == 0 || now >= s_ui.eye_next_ms) {
        uint32_t r = copilot_eye_rand(now);
        bool look_center = (r % 5u) == 0u;
        int8_t side = (r & 1u) ? 1 : -1;
        if (s_ui.eye_side_balance >= 2) side = -1;
        if (s_ui.eye_side_balance <= -2) side = 1;
        if (side == s_ui.eye_last_side) {
            s_ui.eye_same_side_streak++;
            if (s_ui.eye_same_side_streak >= 2) {
                side = (int8_t)-side;
                s_ui.eye_same_side_streak = 0;
            }
        } else {
            s_ui.eye_same_side_streak = 0;
        }

        int16_t mag = 2 + (int16_t)((r >> 8) % (EYE_LOOK_MAX_X - 1));
        int16_t y = (int16_t)((int16_t)((r >> 12) % (EYE_LOOK_MAX_Y * 2 + 1)) - EYE_LOOK_MAX_Y);
        if (look_center) {
            s_ui.eye_look_target_x = 0;
            s_ui.eye_look_target_y = y / 2;
        } else {
            s_ui.eye_look_target_x = (int16_t)(side * mag);
            s_ui.eye_look_target_y = y;
            s_ui.eye_side_balance = (int8_t)copilot_clamp_i16((int16_t)(s_ui.eye_side_balance + side), -3, 3);
            s_ui.eye_last_side = side;
        }
        s_ui.last_idle_pose_bucket++;
        s_ui.eye_next_ms = now + 420 + ((r >> 16) % 420u);
    }

    uint8_t alpha = speaking ? 150 : 74;
    s_ui.eye_look_x = copilot_approach_i16(s_ui.eye_look_x, s_ui.eye_look_target_x, alpha);
    s_ui.eye_look_y = copilot_approach_i16(s_ui.eye_look_y, s_ui.eye_look_target_y, alpha);
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

#if CONFIG_COPILOT_SERVO_ENABLE
static void copilot_update_servo_idle_motion(uint32_t now, bool speaking) {
    if (speaking) {
        s_ui.servo_idle_pitch_target = 0;
        s_ui.servo_idle_yaw_target = 0;
        s_ui.servo_idle_next_ms = now + 260;
    } else if (s_ui.servo_idle_next_ms == 0 || now >= s_ui.servo_idle_next_ms) {
        uint32_t r = copilot_eye_rand(now + 0x314159u);
        bool settle = (r % 6u) == 0u;
        int16_t yaw_deg = (int16_t)((int16_t)((r >> 8) % 21u) - 10);
        int16_t pitch_deg = (int16_t)((int16_t)((r >> 17) % 11u) - 5);
        if (settle) {
            yaw_deg = 0;
            pitch_deg = 0;
        }
        s_ui.servo_idle_yaw_target = (int16_t)(yaw_deg << FP_SHIFT);
        s_ui.servo_idle_pitch_target = (int16_t)(pitch_deg << FP_SHIFT);
        s_ui.servo_idle_next_ms = now + 320 + ((r >> 24) % 820u);
    }

    uint8_t alpha = speaking ? 74 : 34;
    s_ui.servo_idle_pitch = copilot_approach_i16(s_ui.servo_idle_pitch,
                                                 s_ui.servo_idle_pitch_target,
                                                 alpha);
    s_ui.servo_idle_yaw = copilot_approach_i16(s_ui.servo_idle_yaw,
                                               s_ui.servo_idle_yaw_target,
                                               alpha);
}
#endif

/*
 * 3D face renderer — rich grayscale cartoon with depth shading.
 *
 * face_t ∈ [-90°, +90°] Q8.8  →  face_t_norm ∈ [-1, +1]
 */
static void copilot_apply_head(uint32_t now, bool speaking) {
    /* —— mouth progress (smoothed) —— */
    int16_t target = copilot_mouth_target(now, speaking);
    int16_t delta = (int16_t)target - (int16_t)s_ui.mouth_progress;
    uint8_t alpha = speaking ? 150 : 60;
    s_ui.mouth_progress = (uint8_t)((int16_t)s_ui.mouth_progress + ((delta * alpha) >> 8));
    uint8_t mouth_open = s_ui.mouth_progress;
    copilot_update_eye_look(now, speaking);

    /* —— face angle: alive in idle, forward-facing while speaking —— */
    int16_t angle_target = copilot_face_angle_target(now, speaking);
    s_ui.face_angle_prev = s_ui.face_angle;
    int16_t angle_delta = angle_target - s_ui.face_angle;
    s_ui.face_angle += (int16_t)(((int32_t)angle_delta * FACE_ANGLE_SMOOTH) >> FP_SHIFT);

    int16_t face_t = s_ui.face_angle;
    int16_t face_t_norm = (int16_t)(((int32_t)face_t * FP_ONE) / (90 * FP_ONE));
    int16_t abs_t = copilot_abs_i16(face_t_norm);

    int16_t roll  = (int16_t)((s_ui.motion_roll * 5) >> FP_SHIFT);
    int16_t wobble = speaking
        ? (int16_t)(copilot_wave_px(now, 176, SPEAKING_BOB_PX, 0) + copilot_wave_px(now, 93, 2, 80))
        : (int16_t)(copilot_wave_px(now, 2200, IDLE_BOB_PX, 40) + copilot_wave_px(now, 1370, 2, 190));
    int16_t squash = speaking ? (int16_t)(((uint16_t)mouth_open * SPEAKING_SQUASH_MAX) / 255u) : 0;

    /* —— Head: dark filled oval + mid‑tone outline + drop shadow —— */
    int16_t head_w_scale = FP_ONE - (abs_t * 24 / 100);
    int16_t head_w = ((HEAD_W * head_w_scale) >> FP_SHIFT) + squash;
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

    int16_t idle_eye_pulse = speaking ? 0 : copilot_wave_px(now, 2800, 2, 40);
    int16_t eye_near_w = EYE_W_REF + (speaking ? 4 : 2);
    int16_t eye_near_h = EYE_H_REF + idle_eye_pulse - (speaking ? (mouth_open / 38) : 0);
    if (eye_near_h < 9) eye_near_h = 9;
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
    int16_t gaze_x = speaking ? 0 : s_ui.eye_look_x;
    int16_t gaze_y = speaking ? 0 : s_ui.eye_look_y;

    int16_t pupil_l_w = copilot_pupil_size(eye_l_w, 34, 4);
    int16_t pupil_l_h = copilot_pupil_size(eye_l_h, 50, 6);
    int16_t pupil_l_x = copilot_pupil_axis_pos(eye_l_w, pupil_l_w, gaze_x);
    int16_t pupil_l_y = copilot_pupil_axis_pos(eye_l_h, pupil_l_h, gaze_y);
    lv_obj_set_size(s_ui.eye_l_cut, pupil_l_w, pupil_l_h);
    lv_obj_set_pos(s_ui.eye_l_cut, pupil_l_x, pupil_l_y);
    int16_t spk_l_w = pupil_l_w * 45 / 100; if (spk_l_w < 2) spk_l_w = 2;
    int16_t spk_l_h = pupil_l_h * 22 / 100; if (spk_l_h < 2) spk_l_h = 2;
    int16_t spk_l_x = copilot_clamp_i16((int16_t)(pupil_l_x + pupil_l_w / 5), 0, (int16_t)(eye_l_w - spk_l_w));
    int16_t spk_l_y = copilot_clamp_i16((int16_t)(pupil_l_y + pupil_l_h / 8), 0, (int16_t)(eye_l_h - spk_l_h));
    lv_obj_set_size(s_ui.eye_l_spark, spk_l_w, spk_l_h);
    lv_obj_set_pos(s_ui.eye_l_spark, spk_l_x, spk_l_y);

    int16_t pupil_r_w = copilot_pupil_size(eye_r_w, 34, 4);
    int16_t pupil_r_h = copilot_pupil_size(eye_r_h, 50, 6);
    int16_t pupil_r_x = copilot_pupil_axis_pos(eye_r_w, pupil_r_w, gaze_x);
    int16_t pupil_r_y = copilot_pupil_axis_pos(eye_r_h, pupil_r_h, gaze_y);
    lv_obj_set_size(s_ui.eye_r_cut, pupil_r_w, pupil_r_h);
    lv_obj_set_pos(s_ui.eye_r_cut, pupil_r_x, pupil_r_y);
    int16_t spk_r_w = pupil_r_w * 45 / 100; if (spk_r_w < 2) spk_r_w = 2;
    int16_t spk_r_h = pupil_r_h * 22 / 100; if (spk_r_h < 2) spk_r_h = 2;
    int16_t spk_r_x = copilot_clamp_i16((int16_t)(pupil_r_x + pupil_r_w / 5), 0, (int16_t)(eye_r_w - spk_r_w));
    int16_t spk_r_y = copilot_clamp_i16((int16_t)(pupil_r_y + pupil_r_h / 8), 0, (int16_t)(eye_r_h - spk_r_h));
    lv_obj_set_size(s_ui.eye_r_spark, spk_r_w, spk_r_h);
    lv_obj_set_pos(s_ui.eye_r_spark, spk_r_x, spk_r_y);

    /* —— Soft cyan cheek marks; decorative, non-semantic —— */
    int16_t cheek_l_cx = eye_l_cx + (left_near ? 6 : -4);
    int16_t cheek_l_cy = eye_base_y + (EYE_H_REF * 48) / 100;
    int16_t cheek_r_cx = eye_r_cx + (left_near ? -4 : 6);
    int16_t cheek_r_cy = eye_base_y + (EYE_H_REF * 48) / 100;
    int16_t cheek_scale = FP_ONE + (speaking
        ? (int16_t)(((uint16_t)mouth_open * 42u) / 255u)
        : (int16_t)(8 + copilot_wave_px(now, 2600, 5, 80)));
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
    int16_t mouth_jitter_x = speaking
        ? (int16_t)(copilot_wave_px(now, 92, 4, 0) + copilot_wave_px(now, 57, 2, 130))
        : 0;
    int16_t mouth_jitter_y = speaking
        ? (int16_t)(copilot_wave_px(now, 128, 3, 90) + copilot_wave_px(now, 71, 2, 210))
        : 0;
    int16_t mouth_x = hcx + mouth_shift - mouth_w / 2 + mouth_jitter_x;
    int16_t mouth_y = hcy + (head_h * 22) / 100 - mouth_h / 2 + mouth_jitter_y;
    lv_obj_set_size(s_ui.mouth, mouth_w, mouth_h);
    lv_obj_set_pos(s_ui.mouth, mouth_x, mouth_y);

    if (mouth_open > 25) {
        int16_t pad = 4 + ((int16_t)mouth_open / 38);
        int16_t inner_w = mouth_w - pad * 2;
        int16_t inner_h = mouth_h - pad * 2;
        int16_t tooth_w = mouth_w - pad * 3;
        if (inner_w < 2) inner_w = 2;
        if (inner_h < 2) inner_h = 2;
        if (tooth_w < 2) tooth_w = 2;
        lv_obj_clear_flag(s_ui.mouth_inner, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_size(s_ui.mouth_inner, inner_w, inner_h);
        lv_obj_set_pos(s_ui.mouth_inner, pad, pad);
        lv_obj_clear_flag(s_ui.tooth, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_size(s_ui.tooth, tooth_w, 3);
        lv_obj_set_pos(s_ui.tooth, pad * 3 / 2, pad);
    } else {
        lv_obj_add_flag(s_ui.mouth_inner, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_ui.tooth, LV_OBJ_FLAG_HIDDEN);
    }

#if CONFIG_COPILOT_SERVO_ENABLE
    if (!copilot_servo_get_manual()) {
        copilot_update_servo_idle_motion(now, speaking);
        // Drive servos from the stable head cue plus small idle wandering,
        // never from mouth motion.
        float face_deg = (float)s_ui.face_angle / 256.0f;  // Q8.8 -> degrees
        float pitch_idle = (float)s_ui.servo_idle_pitch / 256.0f;
        float yaw_idle = (float)s_ui.servo_idle_yaw / 256.0f;
        float pitch_s = pitch_idle * (CONFIG_COPILOT_SERVO_PITCH_SCALE / 100.0f);
        float yaw_s = (face_deg + yaw_idle) * (CONFIG_COPILOT_SERVO_YAW_SCALE / 100.0f);
        if ((now - s_ui.last_servo_update_ms) >= 60 ||
            copilot_abs_i16((int16_t)(s_ui.face_angle - s_ui.last_servo_angle)) > (1 << FP_SHIFT)) {
            copilot_servo_set_target(pitch_s, yaw_s);
            s_ui.last_servo_update_ms = now;
            s_ui.last_servo_angle = s_ui.face_angle;
        }
    }
#endif
}

static void copilot_update_ring(void) {
    if (s_ui.ring_visible) {
        copilot_ring_set_visible(true);
        copilot_ring_apply_opa(RING_OPA);
        s_ui.ring_opa_current = RING_OPA;
        s_ui.ring_opa_target = RING_OPA;
    } else {
        s_ui.ring_opa_current = 0;
        s_ui.ring_opa_target = 0;
        copilot_ring_apply_opa(0);
        copilot_ring_set_visible(false);
    }
}

static void copilot_enter_return_neutral(uint32_t now, uint32_t duration_ms) {
    if (duration_ms == 0) {
        duration_ms = SCREEN_RETURN_MS;
    }
    s_ui.screen_state = COPILOT_SCREEN_STATE_RETURN_NEUTRAL;
    s_ui.screen_orientation = COPILOT_SCREEN_ORIENT_FRONT;
    s_ui.screen_state_started_ms = now;
    s_ui.screen_state_until_ms = now + duration_ms;
    s_ui.face_angle_target = 0;
    s_ui.manual_speaking = false;
    s_ui.manual_speaking_latched = false;
    s_ui.manual_speaking_until_ms = 0;
    copilot_ui_ring_show(false);
    copilot_log_screen_event("return");
}

static void copilot_update_screen_deadlines(uint32_t now, bool speaking) {
    switch (s_ui.screen_state) {
        case COPILOT_SCREEN_STATE_PRE_MESSAGE_ORIENT:
            if (s_ui.screen_state_until_ms && now >= s_ui.screen_state_until_ms && !speaking) {
                copilot_enter_return_neutral(now, SCREEN_RETURN_MS);
            }
            break;
        case COPILOT_SCREEN_STATE_SPEAKING:
            if (s_ui.screen_state_until_ms && now >= s_ui.screen_state_until_ms) {
                s_ui.t_speech_end_ms = now;
                copilot_enter_return_neutral(now, SCREEN_RETURN_MS);
            } else if (!speaking && s_ui.screen_state_until_ms == 0) {
                s_ui.t_speech_end_ms = now;
                copilot_enter_return_neutral(now, SCREEN_RETURN_MS);
            }
            break;
        case COPILOT_SCREEN_STATE_RETURN_NEUTRAL:
            if (s_ui.screen_state_until_ms && now >= s_ui.screen_state_until_ms) {
                s_ui.screen_state = COPILOT_SCREEN_STATE_NEUTRAL_IDLE;
                s_ui.screen_orientation = COPILOT_SCREEN_ORIENT_FRONT;
                s_ui.screen_state_started_ms = now;
                s_ui.screen_state_until_ms = 0;
                s_ui.face_angle_target = 0;
                copilot_ui_ring_show(false);
                copilot_log_screen_event("neutral");
            }
            break;
        default:
            break;
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
    copilot_update_screen_deadlines(now, speaking);
    bool speaking_changed = speaking != s_ui.last_speaking;
    if (speaking_changed) {
        s_ui.last_speaking = speaking;
        if (speaking) {
            copilot_ui_ring_show(true);
            if (s_ui.screen_state != COPILOT_SCREEN_STATE_SPEAKING) {
                copilot_screen_event_t event = {};
                event.state = COPILOT_SCREEN_STATE_SPEAKING;
                event.orientation = s_ui.screen_orientation;
                event.duration_ms = 0;
                event.message_id = s_ui.message_id;
                event.calibration_content_present = s_ui.calibration_content_present;
                event.visual_semantic_content_present = s_ui.visual_semantic_content_present;
                copilot_ui_set_screen_event(&event);
                s_ui.manual_speaking = false;
                s_ui.manual_speaking_latched = false;
                s_ui.manual_speaking_until_ms = 0;
            }
        } else if (!s_ui.touch_flash_active &&
                   s_ui.screen_state != COPILOT_SCREEN_STATE_PRE_MESSAGE_ORIENT &&
                   s_ui.screen_state != COPILOT_SCREEN_STATE_SPEAKING) {
            copilot_ui_ring_show(false);
        }
    }
    copilot_update_ring();

    bool mouth_animating = (speaking && s_ui.mouth_progress < 250) ||
                           (!speaking && s_ui.mouth_progress > 4);
    bool angle_moving = copilot_abs_i16(copilot_face_angle_target(now, speaking) - s_ui.face_angle) > 8;

    if (speaking || speaking_changed || mouth_animating || motion_changed || angle_moving ||
        !speaking || s_ui.screen_state == COPILOT_SCREEN_STATE_RETURN_NEUTRAL) {
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

    copilot_build_ring_points(s_ring_outer_pts, (int16_t)(RING_OUTER_WIDTH / 2 + 1));
    copilot_build_ring_points(s_ring_inner_pts, (int16_t)(RING_OUTER_WIDTH + 8));

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
    s_ui.ring_objects_visible = false;
    s_ui.ring_opa_applied = 0;

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
    s_ui.screen_state = COPILOT_SCREEN_STATE_NEUTRAL_IDLE;
    s_ui.screen_orientation = COPILOT_SCREEN_ORIENT_FRONT;
    s_ui.face_angle_target = 0;
    s_ui.screen_state_started_ms = lv_tick_get();
    s_ui.message_id[0] = '\0';
    copilot_apply_head(lv_tick_get(), false);
    lv_obj_move_foreground(s_ui.ring_outer);
    lv_obj_move_foreground(s_ui.ring_inner);

    s_ui.anim_timer = lv_timer_create(copilot_anim_timer, ANIM_TIMER_MS, nullptr);
    s_ui.ready = true;

    ESP_LOGI(TAG, "UI ready: %s, asset=%s, soft neutral face, 60fps timer", ROBOT_DISPLAY_VERSION, ASSET_VERSION_HASH);
    copilot_log_screen_event("init");
}

bool copilot_ui_is_ready(void) { return s_ui.ready; }

void copilot_ui_set_screen_event(const copilot_screen_event_t *event) {
    if (!s_ui.ready || !event) return;

    uint32_t now = lv_tick_get();
    s_ui.event_seq++;
    s_ui.screen_state = event->state;
    s_ui.screen_orientation = event->orientation;
    s_ui.screen_state_started_ms = now;
    s_ui.screen_state_until_ms = event->duration_ms > 0 ? now + event->duration_ms : 0;
    s_ui.calibration_content_present = event->calibration_content_present;
    s_ui.visual_semantic_content_present = event->visual_semantic_content_present;
    if (event->message_id && event->message_id[0] != '\0') {
        strncpy(s_ui.message_id, event->message_id, sizeof(s_ui.message_id) - 1);
        s_ui.message_id[sizeof(s_ui.message_id) - 1] = '\0';
    } else if (event->state == COPILOT_SCREEN_STATE_NEUTRAL_IDLE ||
               event->state == COPILOT_SCREEN_STATE_SILENT_NEUTRAL) {
        s_ui.message_id[0] = '\0';
    }

    switch (event->state) {
        case COPILOT_SCREEN_STATE_PRE_MESSAGE_ORIENT:
            s_ui.face_angle_target = copilot_orientation_angle(event->orientation);
            s_ui.manual_speaking = false;
            s_ui.manual_speaking_latched = false;
            s_ui.manual_speaking_until_ms = 0;
            if (event->duration_ms > 0) {
                uint32_t cue_ms = copilot_clamp_u32(event->duration_ms, PRECUE_MIN_MS, PRECUE_MAX_MS + 200);
                s_ui.screen_state_until_ms = now + cue_ms;
            }
            s_ui.t_cue_ms = now;
            copilot_ui_ring_show(true);
            break;
        case COPILOT_SCREEN_STATE_SPEAKING:
            s_ui.face_angle_target = copilot_orientation_angle(event->orientation);
            s_ui.manual_speaking = true;
            s_ui.manual_speaking_latched = event->duration_ms == 0;
            s_ui.manual_speaking_until_ms = event->duration_ms > 0 ? now + event->duration_ms : 0;
            s_ui.t_speech_start_ms = now;
            s_ui.t_speech_end_ms = event->duration_ms > 0 ? now + event->duration_ms : 0;
            copilot_ui_ring_show(true);
            break;
        case COPILOT_SCREEN_STATE_RETURN_NEUTRAL:
            s_ui.screen_state_until_ms = now + (event->duration_ms > 0 ? event->duration_ms : SCREEN_RETURN_MS);
            s_ui.face_angle_target = 0;
            s_ui.manual_speaking = false;
            s_ui.manual_speaking_latched = false;
            s_ui.manual_speaking_until_ms = 0;
            if (s_ui.t_speech_start_ms != 0 && s_ui.t_speech_end_ms == 0) {
                s_ui.t_speech_end_ms = now;
            }
            copilot_ui_ring_show(false);
            break;
        case COPILOT_SCREEN_STATE_SILENT_NEUTRAL:
        case COPILOT_SCREEN_STATE_NEUTRAL_IDLE:
            s_ui.screen_state_until_ms = 0;
            s_ui.face_angle_target = 0;
            s_ui.manual_speaking = false;
            s_ui.manual_speaking_latched = false;
            s_ui.manual_speaking_until_ms = 0;
            s_ui.mouth_audio_smooth = 0;
            copilot_ui_ring_show(false);
            break;
        case COPILOT_SCREEN_STATE_DEBUG:
        default:
            s_ui.face_angle_target = copilot_orientation_angle(event->orientation);
            copilot_ui_ring_show(false);
            break;
    }

    copilot_log_screen_event("set");
}

void copilot_ui_set_expression(copilot_expr_t expr, uint32_t duration_ms) {
    if (!s_ui.ready || expr >= COPILOT_EXPR_COUNT) return;
    copilot_screen_event_t event = {};
    if (expr == COPILOT_EXPR_SPEAKING) {
        event.state = COPILOT_SCREEN_STATE_SPEAKING;
        event.orientation = s_ui.screen_orientation;
        event.duration_ms = duration_ms;
        event.message_id = s_ui.message_id;
        copilot_ui_set_screen_event(&event);
        LOGI_UI("Expression speaking duration=%u", (unsigned)duration_ms);
    } else {
        event.state = COPILOT_SCREEN_STATE_RETURN_NEUTRAL;
        event.orientation = COPILOT_SCREEN_ORIENT_FRONT;
        event.duration_ms = SCREEN_RETURN_MS;
        copilot_ui_set_screen_event(&event);
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
    s_ui.ring_opa_current = on ? RING_OPA : 0;
    s_ui.ring_opa_target = s_ui.ring_opa_current;
    LOGI_UI("Ring show=%s", on ? "on" : "off");
    if (on) {
        copilot_ring_set_visible(true);
        copilot_ring_apply_opa(RING_OPA);
    } else {
        copilot_ring_apply_opa(0);
        copilot_ring_set_visible(false);
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

static bool copilot_ui_lock_async(const char *op) {
    if (bsp_display_lock(UI_ASYNC_LOCK_MS)) {
        return true;
    }
    ESP_LOGW(TAG, "LVGL lock timeout while applying %s", op ? op : "ui event");
    return false;
}

void copilot_ui_set_screen_event_async(const copilot_screen_event_t *event) {
    if (!s_ui.ready || !event) return;
    if (copilot_ui_lock_async("screen event")) { copilot_ui_set_screen_event(event); bsp_display_unlock(); }
}

void copilot_ui_set_expression_async(copilot_expr_t expr, uint32_t duration_ms) {
    if (!s_ui.ready) return;
    if (copilot_ui_lock_async("expression")) { copilot_ui_set_expression(expr, duration_ms); bsp_display_unlock(); }
}

void copilot_ui_set_motion_async(const copilot_motion_t *motion) {
    if (!s_ui.ready || !motion) return;
    if (copilot_ui_lock_async("motion")) { copilot_ui_set_motion(motion); bsp_display_unlock(); }
}

void copilot_ui_set_motion_only_async(const copilot_motion_t *motion) {
    if (!s_ui.ready || !motion) return;
    if (copilot_ui_lock_async("motion-only")) {
        s_ui.motion_target = *motion;
        ESP_LOGD(TAG, "Motion only ax=%d ay=%d yaw=%d (Q8.8)",
                 motion->ax, motion->ay, motion->yaw_deg);
        bsp_display_unlock();
    }
}

void copilot_ui_ring_show_async(bool on) {
    if (!s_ui.ready) return;
    if (copilot_ui_lock_async("ring")) { copilot_ui_ring_show(on); bsp_display_unlock(); }
}
