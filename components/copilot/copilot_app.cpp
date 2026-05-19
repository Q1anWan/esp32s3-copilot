#include "copilot_app.h"

#include <string.h>
#include <math.h>

#include "cJSON.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "copilot_audio.h"
#include "copilot_imu.h"
#include "copilot_mqtt.h"
#include "copilot_perf.h"
#include "copilot_serial_console.h"
#include "copilot_tcp_host.h"
#include "copilot_ui.h"
#include "copilot_voice.h"
#include "copilot_voice_ui.h"
#if CONFIG_COPILOT_SERVO_ENABLE
#include "copilot_servo.h"
#endif
#include "copilot_axp2101.h"

static const char *TAG = "copilot_app";

// Conditional logging
#if CONFIG_COPILOT_LOG_APP
#define LOGI_APP(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#else
#define LOGI_APP(fmt, ...) do {} while(0)
#endif

static int copilot_normalize_core(int core) {
    if (core < 0) {
        return -1;
    }
    if (core >= (int)configNUM_CORES) {
        return -1;
    }
    return core;
}

enum action_type_t {
    ACTION_NONE = 0,
    ACTION_EXPR,
    ACTION_SOUND,
    ACTION_SCREEN_STATE,
    ACTION_SCREEN_MESSAGE,
};

struct copilot_action_t {
    action_type_t type;
    copilot_expr_t expr;
    copilot_screen_state_t screen_state;
    copilot_screen_orientation_t orientation;
    uint32_t duration_ms;
    uint32_t prelight_ms;
    char sound_id[16];
    char scene_id[32];
    char message_id[32];
    uint16_t sequence_id;
    bool use_scene_audio;
    bool calibration_content_present;
    bool visual_semantic_content_present;
};

static QueueHandle_t s_action_queue = nullptr;
static TaskHandle_t s_action_task = nullptr;
static bool s_network_started = false;
static char s_screen_event_publish_buf[512];

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

static void copilot_publish_screen_event(const copilot_action_t *action, copilot_screen_state_t state) {
    if (!action) {
        return;
    }
    snprintf(s_screen_event_publish_buf, sizeof(s_screen_event_publish_buf),
             "{\"type\":\"screen_event\","
             "\"calibration_content_present\":%s,"
             "\"visual_semantic_content_present\":%s,"
             "\"screen_state\":\"%s\","
             "\"screen_orientation\":\"%s\","
             "\"screen_text\":\"\","
             "\"screen_icon\":\"\","
             "\"t_cue\":0,"
             "\"t_speech_start\":0,"
             "\"t_speech_end\":0,"
             "\"sync_error_ms\":0,"
             "\"asset_version_hash\":\"face_soft_neutral_20260514\","
             "\"message_id\":\"%s\"}",
             action->calibration_content_present ? "true" : "false",
             action->visual_semantic_content_present ? "true" : "false",
             copilot_screen_state_name(state),
             copilot_screen_orientation_name(action->orientation),
             action->message_id);
    copilot_mqtt_publish("status", s_screen_event_publish_buf);
}

static void copilot_apply_screen_action(const copilot_action_t *action,
                                        copilot_screen_state_t state,
                                        uint32_t duration_ms) {
    if (!action) {
        return;
    }
    copilot_screen_event_t event = {};
    event.state = state;
    event.orientation = action->orientation;
    event.duration_ms = duration_ms;
    event.message_id = action->message_id;
    event.calibration_content_present = action->calibration_content_present;
    event.visual_semantic_content_present = action->visual_semantic_content_present;
    copilot_ui_set_screen_event_async(&event);
    copilot_publish_screen_event(action, state);
}

static void copilot_action_task(void *arg) {
    (void)arg;
    copilot_action_t action = {};
    for (;;) {
        if (xQueueReceive(s_action_queue, &action, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (action.type == ACTION_SCREEN_MESSAGE) {
            uint32_t cue_ms = action.prelight_ms > 0 ? action.prelight_ms : CONFIG_COPILOT_PRELIGHT_MS;
            copilot_apply_screen_action(&action, COPILOT_SCREEN_STATE_PRE_MESSAGE_ORIENT, cue_ms + 150);
            vTaskDelay(pdMS_TO_TICKS(cue_ms));
            copilot_apply_screen_action(&action, COPILOT_SCREEN_STATE_SPEAKING, action.duration_ms);
        } else if (action.type == ACTION_SCREEN_STATE) {
            copilot_apply_screen_action(&action, action.screen_state, action.duration_ms);
        } else if (action.type == ACTION_EXPR) {
            LOGI_APP( "Apply expression=%d duration=%u sound=%s", (int)action.expr, (unsigned)action.duration_ms,
                     action.sound_id[0] ? action.sound_id : "none");
            if (action.expr == COPILOT_EXPR_SPEAKING) {
                if (action.prelight_ms > 0) {
                    copilot_apply_screen_action(&action, COPILOT_SCREEN_STATE_PRE_MESSAGE_ORIENT, action.prelight_ms + 150);
                    vTaskDelay(pdMS_TO_TICKS(action.prelight_ms));
                }
                copilot_apply_screen_action(&action, COPILOT_SCREEN_STATE_SPEAKING, action.duration_ms);
            } else {
                copilot_apply_screen_action(&action, COPILOT_SCREEN_STATE_RETURN_NEUTRAL, 520);
            }
        } else if (action.type == ACTION_SOUND) {
            if (action.use_scene_audio) {
                if (action.prelight_ms > 0) {
                    copilot_apply_screen_action(&action, COPILOT_SCREEN_STATE_PRE_MESSAGE_ORIENT, action.prelight_ms + 150);
                    vTaskDelay(pdMS_TO_TICKS(action.prelight_ms));
                }
                copilot_apply_screen_action(&action, COPILOT_SCREEN_STATE_SPEAKING,
                                            action.duration_ms > 0 ? action.duration_ms : 0);
                LOGI_APP("Play scene audio scene=%s seq=%u", action.scene_id, (unsigned)action.sequence_id);
                copilot_audio_play_scene(action.scene_id, action.sequence_id);
                continue;
            } else if (action.prelight_ms > 0) {
                copilot_apply_screen_action(&action, COPILOT_SCREEN_STATE_PRE_MESSAGE_ORIENT, action.prelight_ms + 150);
                vTaskDelay(pdMS_TO_TICKS(action.prelight_ms));
                copilot_apply_screen_action(&action, COPILOT_SCREEN_STATE_RETURN_NEUTRAL, 520);
            }
            LOGI_APP( "Play sound=%s", action.sound_id[0] ? action.sound_id : "none");
        }

        if (action.sound_id[0] != '\0') {
            copilot_audio_play(action.sound_id);
        }
    }
}

static const struct {
    const char *name;
    copilot_expr_t expr;
} kExprMap[] = {
    {"idle", COPILOT_EXPR_IDLE},
    {"neutral", COPILOT_EXPR_NEUTRAL},
    {"happy", COPILOT_EXPR_HAPPY},
    {"sad", COPILOT_EXPR_SAD},
    {"angry", COPILOT_EXPR_ANGRY},
    {"surprised", COPILOT_EXPR_SURPRISED},
    {"sleepy", COPILOT_EXPR_SLEEPY},
    {"dizzy", COPILOT_EXPR_DIZZY},
    {"speaking", COPILOT_EXPR_SPEAKING},
    {"talking", COPILOT_EXPR_TALKING},
    {"mouth_open", COPILOT_EXPR_SPEAKING},
    {"open_mouth", COPILOT_EXPR_SPEAKING},
};

static copilot_expr_t copilot_expr_from_name(const char *name) {
    if (!name) {
        return COPILOT_EXPR_NEUTRAL;
    }
    for (size_t i = 0; i < sizeof(kExprMap) / sizeof(kExprMap[0]); ++i) {
        if (strcmp(name, kExprMap[i].name) == 0) {
            return kExprMap[i].expr;
        }
    }
    return COPILOT_EXPR_NEUTRAL;
}

static void copilot_enqueue_action(const copilot_action_t *action) {
    if (!s_action_queue || !action) {
        return;
    }
    if (xQueueSend(s_action_queue, action, 0) != pdTRUE) {
        copilot_action_t dropped = {};
        xQueueReceive(s_action_queue, &dropped, 0);
        if (xQueueSend(s_action_queue, action, 0) != pdTRUE) {
            ESP_LOGW(TAG, "Action queue full, drop command");
        }
    }
}

static void copilot_ensure_action_task(void) {
    if (s_action_queue) {
        return;
    }

    s_action_queue = xQueueCreate(8, sizeof(copilot_action_t));
    if (!s_action_queue) {
        ESP_LOGE(TAG, "Failed to create action queue");
        return;
    }

    int core = copilot_normalize_core(CONFIG_COPILOT_ACTION_CORE);
    BaseType_t task_ok;
    if (core >= 0) {
        task_ok = xTaskCreatePinnedToCore(copilot_action_task, "copilot_action", 6 * 1024, nullptr, 3,
                                          &s_action_task, core);
    } else {
        task_ok = xTaskCreate(copilot_action_task, "copilot_action", 6 * 1024, nullptr, 3, &s_action_task);
    }
    if (task_ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create action task");
    } else {
        LOGI_APP( "Action task core=%d", core);
    }
}

static void copilot_copy_action_text(char *dst, size_t dst_len, const char *src) {
    if (!dst || dst_len == 0) {
        return;
    }
    if (src && src[0] != '\0') {
        strncpy(dst, src, dst_len - 1);
        dst[dst_len - 1] = '\0';
    } else {
        dst[0] = '\0';
    }
}

static void copilot_schedule_expression(copilot_expr_t expr, uint32_t duration_ms, uint32_t prelight_ms,
                                        const char *sound_id, copilot_screen_orientation_t orientation,
                                        const char *message_id) {
    copilot_action_t action = {};
    action.type = ACTION_EXPR;
    action.expr = expr;
    action.screen_state = expr == COPILOT_EXPR_SPEAKING ? COPILOT_SCREEN_STATE_SPEAKING
                                                        : COPILOT_SCREEN_STATE_RETURN_NEUTRAL;
    action.orientation = orientation;
    action.duration_ms = duration_ms;
    action.prelight_ms = prelight_ms;
    copilot_copy_action_text(action.sound_id, sizeof(action.sound_id), sound_id);
    copilot_copy_action_text(action.message_id, sizeof(action.message_id), message_id);
    LOGI_APP( "Schedule expression=%d duration=%u prelight=%u orient=%s sound=%s", (int)expr, (unsigned)duration_ms,
             (unsigned)prelight_ms, copilot_screen_orientation_name(orientation), sound_id ? sound_id : "none");
    copilot_enqueue_action(&action);
}

static void copilot_schedule_sound(const char *sound_id, uint32_t prelight_ms) {
    copilot_action_t action = {};
    action.type = ACTION_SOUND;
    action.expr = COPILOT_EXPR_NEUTRAL;
    action.screen_state = COPILOT_SCREEN_STATE_RETURN_NEUTRAL;
    action.orientation = COPILOT_SCREEN_ORIENT_FRONT;
    action.duration_ms = 0;
    action.prelight_ms = prelight_ms;
    copilot_copy_action_text(action.sound_id, sizeof(action.sound_id), sound_id);
    LOGI_APP( "Schedule sound=%s prelight=%u", sound_id ? sound_id : "none", (unsigned)prelight_ms);
    copilot_enqueue_action(&action);
}

static void copilot_schedule_scene_sound(const char *scene_id, uint16_t sequence_id,
                                         uint32_t prelight_ms, uint32_t duration_ms,
                                         copilot_screen_orientation_t orientation,
                                         const char *message_id) {
    copilot_action_t action = {};
    action.type = ACTION_SOUND;
    action.expr = COPILOT_EXPR_SPEAKING;
    action.screen_state = COPILOT_SCREEN_STATE_SPEAKING;
    action.orientation = orientation;
    action.duration_ms = duration_ms;
    action.prelight_ms = prelight_ms;
    action.use_scene_audio = true;
    action.sequence_id = sequence_id;
    copilot_copy_action_text(action.scene_id, sizeof(action.scene_id), scene_id ? scene_id : "default");
    copilot_copy_action_text(action.message_id, sizeof(action.message_id), message_id);
    LOGI_APP("Schedule scene audio scene=%s seq=%u prelight=%u duration=%u",
             action.scene_id, (unsigned)sequence_id, (unsigned)prelight_ms, (unsigned)duration_ms);
    copilot_enqueue_action(&action);
}

static void copilot_schedule_screen_state(copilot_screen_state_t state, copilot_screen_orientation_t orientation,
                                          uint32_t duration_ms, const char *message_id,
                                          bool calibration_content_present,
                                          bool visual_semantic_content_present) {
    copilot_action_t action = {};
    action.type = ACTION_SCREEN_STATE;
    action.expr = state == COPILOT_SCREEN_STATE_SPEAKING ? COPILOT_EXPR_SPEAKING : COPILOT_EXPR_NEUTRAL;
    action.screen_state = state;
    action.orientation = orientation;
    action.duration_ms = duration_ms;
    action.calibration_content_present = calibration_content_present;
    action.visual_semantic_content_present = visual_semantic_content_present;
    copilot_copy_action_text(action.message_id, sizeof(action.message_id), message_id);
    LOGI_APP("Schedule screen state=%s duration=%u orient=%s msg=%s",
             copilot_screen_state_name(state), (unsigned)duration_ms,
             copilot_screen_orientation_name(orientation), action.message_id);
    copilot_enqueue_action(&action);
}

static void copilot_schedule_screen_message(copilot_screen_orientation_t orientation, uint32_t prelight_ms,
                                            uint32_t speech_ms, const char *message_id,
                                            bool calibration_content_present,
                                            bool visual_semantic_content_present) {
    copilot_action_t action = {};
    action.type = ACTION_SCREEN_MESSAGE;
    action.expr = COPILOT_EXPR_SPEAKING;
    action.screen_state = COPILOT_SCREEN_STATE_SPEAKING;
    action.orientation = orientation;
    action.duration_ms = speech_ms;
    action.prelight_ms = prelight_ms;
    action.calibration_content_present = calibration_content_present;
    action.visual_semantic_content_present = visual_semantic_content_present;
    copilot_copy_action_text(action.message_id, sizeof(action.message_id), message_id);
    LOGI_APP("Schedule screen message cue=%u speech=%u orient=%s msg=%s",
             (unsigned)prelight_ms, (unsigned)speech_ms,
             copilot_screen_orientation_name(orientation), action.message_id);
    copilot_enqueue_action(&action);
}

// Get number from JSON and convert to Q8.8 fixed-point
static int16_t copilot_get_number_fp(const cJSON *obj, const char *key, int16_t def_value) {
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsNumber(item)) {
        // Convert float to Q8.8 fixed-point
        return (int16_t)(item->valuedouble * FP_ONE);
    }
    return def_value;
}

static uint32_t copilot_get_u32(const cJSON *obj, const char *key, uint32_t def_value) {
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsNumber(item) && item->valuedouble >= 0) {
        return (uint32_t)item->valuedouble;
    }
    return def_value;
}

static bool copilot_get_bool(const cJSON *obj, const char *key, bool def_value) {
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsBool(item)) {
        return cJSON_IsTrue(item);
    }
    return def_value;
}

static const char *copilot_get_string_any(const cJSON *obj, const char *key_a, const char *key_b, const char *def_value) {
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key_a);
    if (cJSON_IsString(item) && item->valuestring) {
        return item->valuestring;
    }
    if (key_b) {
        item = cJSON_GetObjectItemCaseSensitive(obj, key_b);
        if (cJSON_IsString(item) && item->valuestring) {
            return item->valuestring;
        }
    }
    return def_value;
}

static bool copilot_get_scene_string(const cJSON *obj, char *out, size_t out_len) {
    if (!out || out_len == 0) {
        return false;
    }
    out[0] = '\0';
    const cJSON *scene = cJSON_GetObjectItemCaseSensitive(obj, "scene");
    if (!scene) {
        scene = cJSON_GetObjectItemCaseSensitive(obj, "scene_id");
    }
    if (!scene) {
        scene = cJSON_GetObjectItemCaseSensitive(obj, "category");
    }
    if (cJSON_IsString(scene) && scene->valuestring) {
        strncpy(out, scene->valuestring, out_len - 1);
        out[out_len - 1] = '\0';
        return true;
    }
    if (cJSON_IsNumber(scene)) {
        snprintf(out, out_len, "%d", (int)scene->valuedouble);
        return true;
    }
    return false;
}

static uint16_t copilot_get_sequence_id(const cJSON *obj, uint16_t def_value) {
    const char *keys[] = {"seq", "sequence", "sequence_id", "index", "number"};
    for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); ++i) {
        const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, keys[i]);
        if (cJSON_IsNumber(item) && item->valuedouble >= 0 && item->valuedouble <= 65535) {
            return (uint16_t)item->valuedouble;
        }
    }
    const cJSON *id = cJSON_GetObjectItemCaseSensitive(obj, "id");
    if (cJSON_IsNumber(id) && id->valuedouble >= 0 && id->valuedouble <= 65535) {
        return (uint16_t)id->valuedouble;
    }
    return def_value;
}

static copilot_screen_orientation_t copilot_orientation_from_name(const char *name) {
    if (!name) {
        return COPILOT_SCREEN_ORIENT_FRONT;
    }
    if (strcmp(name, "left") == 0 || strcmp(name, "l") == 0) {
        return COPILOT_SCREEN_ORIENT_LEFT;
    }
    if (strcmp(name, "right") == 0 || strcmp(name, "r") == 0) {
        return COPILOT_SCREEN_ORIENT_RIGHT;
    }
    return COPILOT_SCREEN_ORIENT_FRONT;
}

static copilot_screen_orientation_t copilot_get_orientation(const cJSON *obj) {
    const char *name = copilot_get_string_any(obj, "orientation", "direction", nullptr);
    if (!name) {
        name = copilot_get_string_any(obj, "dir", nullptr, "front");
    }
    return copilot_orientation_from_name(name);
}

static copilot_screen_state_t copilot_screen_state_from_name(const char *name) {
    if (!name) {
        return COPILOT_SCREEN_STATE_NEUTRAL_IDLE;
    }
    if (strcmp(name, "pre_message_orient") == 0 || strcmp(name, "pre_message") == 0 ||
        strcmp(name, "pre_cue") == 0 || strcmp(name, "cue") == 0) {
        return COPILOT_SCREEN_STATE_PRE_MESSAGE_ORIENT;
    }
    if (strcmp(name, "speaking") == 0 || strcmp(name, "talking") == 0) {
        return COPILOT_SCREEN_STATE_SPEAKING;
    }
    if (strcmp(name, "return_neutral") == 0 || strcmp(name, "return") == 0) {
        return COPILOT_SCREEN_STATE_RETURN_NEUTRAL;
    }
    if (strcmp(name, "silent_neutral") == 0 || strcmp(name, "silent") == 0) {
        return COPILOT_SCREEN_STATE_SILENT_NEUTRAL;
    }
    if (strcmp(name, "debug") == 0) {
        return COPILOT_SCREEN_STATE_DEBUG;
    }
    return COPILOT_SCREEN_STATE_NEUTRAL_IDLE;
}

void copilot_app_handle_command(const char *payload, int payload_len) {
    LOGI_APP( "MQTT payload: %.*s", payload_len, payload);
    cJSON *root = cJSON_ParseWithLength(payload, payload_len);
    if (!root) {
        ESP_LOGW(TAG, "Invalid JSON payload");
        return;
    }

    const cJSON *type = cJSON_GetObjectItemCaseSensitive(root, "type");
    if (!cJSON_IsString(type) || !type->valuestring) {
        cJSON_Delete(root);
        return;
    }

    if (strcmp(type->valuestring, "wifi") == 0) {
        const char *ssid = copilot_get_string_any(root, "ssid", nullptr, "");
        const char *password = copilot_get_string_any(root, "password", "pass", "");
        bool ok = copilot_mqtt_configure_wifi(ssid, password);
        ESP_LOGI(TAG, "WiFi config command: ssid=%s result=%s", ssid, ok ? "ok" : "failed");
    } else if (strcmp(type->valuestring, "screen") == 0 || strcmp(type->valuestring, "display") == 0) {
        const char *action = copilot_get_string_any(root, "action", nullptr, "");
        const char *state_name = copilot_get_string_any(root, "state", nullptr, "");
        const char *message_id = copilot_get_string_any(root, "message_id", "id", "");
        copilot_screen_orientation_t orientation = copilot_get_orientation(root);
        bool calibration_content_present = copilot_get_bool(root, "calibration_content_present", false);
        bool visual_semantic_content_present = copilot_get_bool(root, "visual_semantic_content_present", false);

        if (strcmp(action, "message") == 0 || strcmp(state_name, "message") == 0 ||
            strcmp(action, "timeline") == 0) {
            uint32_t pre_cue_ms = copilot_get_u32(root, "pre_cue_ms",
                                      copilot_get_u32(root, "prelight_ms", CONFIG_COPILOT_PRELIGHT_MS));
            uint32_t speech_ms = copilot_get_u32(root, "speech_ms",
                                    copilot_get_u32(root, "duration_ms", CONFIG_COPILOT_EXPR_TRANSITION_MS));
            copilot_schedule_screen_message(orientation, pre_cue_ms, speech_ms, message_id,
                                            calibration_content_present,
                                            visual_semantic_content_present);
        } else {
            copilot_screen_state_t state = copilot_screen_state_from_name(state_name);
            uint32_t duration_ms = copilot_get_u32(root, "duration_ms", 0);
            LOGI_APP("Screen cmd: state=%s duration=%u orient=%s msg=%s",
                     copilot_screen_state_name(state), (unsigned)duration_ms,
                     copilot_screen_orientation_name(orientation), message_id);
            copilot_schedule_screen_state(state, orientation, duration_ms, message_id,
                                          calibration_content_present,
                                          visual_semantic_content_present);
        }
    } else if (strcmp(type->valuestring, "motion") == 0) {
        copilot_motion_t motion = {};

        // Check if quaternion is provided (external IMU with orientation)
        const cJSON *qw_item = cJSON_GetObjectItemCaseSensitive(root, "qw");
        if (cJSON_IsNumber(qw_item)) {
            // Quaternion mode: extract roll/pitch/yaw from quaternion
            // Vehicle body frame: X=front, Y=left, Z=up
            // Quaternion represents vehicle orientation in world frame
            float qw = qw_item->valuedouble;
            float qx = 0, qy = 0, qz = 0;
            const cJSON *qx_item = cJSON_GetObjectItemCaseSensitive(root, "qx");
            const cJSON *qy_item = cJSON_GetObjectItemCaseSensitive(root, "qy");
            const cJSON *qz_item = cJSON_GetObjectItemCaseSensitive(root, "qz");
            if (cJSON_IsNumber(qx_item)) qx = qx_item->valuedouble;
            if (cJSON_IsNumber(qy_item)) qy = qy_item->valuedouble;
            if (cJSON_IsNumber(qz_item)) qz = qz_item->valuedouble;

            // Quaternion to Euler angles (ZYX convention)
            // Roll (X): left/right tilt → animation ax
            // Pitch (Y): forward/backward tilt → animation ay
            // Yaw (Z): heading rotation → animation yaw
            float sinr_cosp = 2.0f * (qw * qx + qy * qz);
            float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
            float roll = atan2f(sinr_cosp, cosr_cosp);  // radians

            float sinp = 2.0f * (qw * qy - qz * qx);
            float pitch;
            if (fabsf(sinp) >= 1.0f) {
                pitch = copysignf(M_PI / 2.0f, sinp);  // Use 90 degrees if out of range
            } else {
                pitch = asinf(sinp);
            }

            float siny_cosp = 2.0f * (qw * qz + qx * qy);
            float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
            float yaw = atan2f(siny_cosp, cosy_cosp);  // radians

            // Convert to animation values
            // Roll → ax (left/right drift): positive roll = right side down = drift right
            // Pitch → ay (up/down drift): positive pitch = nose up = drift up
            // Yaw → yaw_deg
            float ax = sinf(roll);   // sin(roll) gives normalized tilt (-1 to +1)
            float ay = sinf(pitch);  // sin(pitch) gives normalized tilt
            float yaw_deg = yaw * 57.2957795f;  // Convert to degrees

            motion.ax = FP_FROM_FLOAT(ax);
            motion.ay = FP_FROM_FLOAT(ay);
            motion.yaw_deg = FP_FROM_FLOAT(yaw_deg);
            motion.speed = copilot_get_number_fp(root, "speed", FP_ONE);

            LOGI_APP( "Motion (quat) ax=%d ay=%d yaw=%d speed=%d (Q8.8)",
                     motion.ax, motion.ay, motion.yaw_deg, motion.speed);
        } else {
            // Direct mode: ax/ay/yaw values directly from JSON
            // ax/ay are in g units (-1 to +1), yaw in degrees
            motion.ax = copilot_get_number_fp(root, "ax", 0);
            motion.ay = copilot_get_number_fp(root, "ay", 0);
            motion.yaw_deg = copilot_get_number_fp(root, "yaw", 0);
            motion.speed = copilot_get_number_fp(root, "speed", FP_ONE);

            LOGI_APP( "Motion ax=%d ay=%d yaw=%d speed=%d (Q8.8)",
                     motion.ax, motion.ay, motion.yaw_deg, motion.speed);
        }

        copilot_ui_set_motion_async(&motion);
    } else if (strcmp(type->valuestring, "emotion") == 0 || strcmp(type->valuestring, "expression") == 0) {
        const cJSON *name = cJSON_GetObjectItemCaseSensitive(root, "name");
        const cJSON *id = cJSON_GetObjectItemCaseSensitive(root, "id");
        copilot_expr_t expr = COPILOT_EXPR_NEUTRAL;
        if (cJSON_IsString(name)) {
            expr = copilot_expr_from_name(name->valuestring);
        } else if (cJSON_IsNumber(id)) {
            int idx = (int)id->valuedouble;
            if (idx >= 0 && idx < (int)COPILOT_EXPR_COUNT) {
                expr = (copilot_expr_t)idx;
            }
        }

        uint32_t duration_ms = copilot_get_u32(root, "duration_ms", CONFIG_COPILOT_EXPR_TRANSITION_MS);
        uint32_t prelight_ms = copilot_get_u32(root, "prelight_ms", CONFIG_COPILOT_PRELIGHT_MS);
        copilot_screen_orientation_t orientation = copilot_get_orientation(root);
        const char *message_id = copilot_get_string_any(root, "message_id", nullptr, "");

        const char *sound_id = nullptr;
        const cJSON *sound = cJSON_GetObjectItemCaseSensitive(root, "sound");
        if (cJSON_IsString(sound)) {
            sound_id = sound->valuestring;
        } else if (cJSON_IsTrue(sound)) {
            sound_id = "beep_short";
        }

        LOGI_APP( "Expression cmd: name=%s id=%s expr=%d duration=%u prelight=%u orient=%s sound=%s",
                 cJSON_IsString(name) ? name->valuestring : "null",
                 cJSON_IsNumber(id) ? "num" : "null",
                 (int)expr, (unsigned)duration_ms, (unsigned)prelight_ms,
                 copilot_screen_orientation_name(orientation),
                 sound_id ? sound_id : "none");
        copilot_schedule_expression(expr, duration_ms, prelight_ms, sound_id, orientation, message_id);
    } else if (strcmp(type->valuestring, "sound") == 0 ||
               strcmp(type->valuestring, "audio") == 0 ||
               strcmp(type->valuestring, "play") == 0) {
        char scene_id[32];
        bool has_scene = copilot_get_scene_string(root, scene_id, sizeof(scene_id));
        const cJSON *seq_item = cJSON_GetObjectItemCaseSensitive(root, "seq");
        if (!seq_item) {
            seq_item = cJSON_GetObjectItemCaseSensitive(root, "sequence");
        }
        if (!seq_item) {
            seq_item = cJSON_GetObjectItemCaseSensitive(root, "sequence_id");
        }
        bool has_sequence = cJSON_IsNumber(seq_item);
        if (has_scene || has_sequence) {
            if (scene_id[0] == '\0') {
                strncpy(scene_id, "default", sizeof(scene_id) - 1);
                scene_id[sizeof(scene_id) - 1] = '\0';
            }
            uint16_t sequence_id = copilot_get_sequence_id(root, 1);
            uint32_t prelight_ms = copilot_get_u32(root, "prelight_ms", 0);
            uint32_t duration_ms = copilot_get_u32(root, "duration_ms", 0);
            copilot_screen_orientation_t orientation = copilot_get_orientation(root);
            const char *message_id = copilot_get_string_any(root, "message_id", nullptr, "");
            ESP_LOGI(TAG, "Scene audio cmd: scene=%s seq=%u", scene_id, (unsigned)sequence_id);
            copilot_schedule_scene_sound(scene_id, sequence_id, prelight_ms, duration_ms, orientation, message_id);
            cJSON_Delete(root);
            return;
        }
        const cJSON *id = cJSON_GetObjectItemCaseSensitive(root, "id");
        const char *sound_id = cJSON_IsString(id) ? id->valuestring : "beep_short";
        uint32_t prelight_ms = copilot_get_u32(root, "prelight_ms", CONFIG_COPILOT_PRELIGHT_MS);
        LOGI_APP( "Sound id=%s prelight=%u", sound_id, (unsigned)prelight_ms);
        copilot_schedule_sound(sound_id, prelight_ms);
    } else if (strcmp(type->valuestring, "ring") == 0) {
        const cJSON *on = cJSON_GetObjectItemCaseSensitive(root, "on");
        if (cJSON_IsBool(on)) {
            LOGI_APP( "Ring on=%s", cJSON_IsTrue(on) ? "true" : "false");
            copilot_ui_ring_show_async(cJSON_IsTrue(on));
        }
    } else if (strcmp(type->valuestring, "calibrate") == 0) {
        // IMU gyroscope zero-bias calibration
        const cJSON *target = cJSON_GetObjectItemCaseSensitive(root, "target");
        const char *target_str = cJSON_IsString(target) ? target->valuestring : "gyro";

        if (strcmp(target_str, "gyro") == 0) {
            LOGI_APP( "IMU gyro calibration requested via MQTT");
            if (copilot_imu_start_calibration()) {
                LOGI_APP( "Gyro calibration started. Keep device stationary!");
            } else {
                ESP_LOGW(TAG, "Gyro calibration failed to start (IMU not ready or already calibrating)");
            }
        } else {
            ESP_LOGW(TAG, "Unknown calibration target: %s", target_str);
        }
    } else if (strcmp(type->valuestring, "status") == 0) {
        // Query device status
        const cJSON *query = cJSON_GetObjectItemCaseSensitive(root, "query");
        const char *query_str = cJSON_IsString(query) ? query->valuestring : "all";

        if (strcmp(query_str, "imu") == 0 || strcmp(query_str, "all") == 0) {
#if CONFIG_COPILOT_LOG_APP
            float bias = copilot_imu_get_gyro_bias();
            bool calibrating = copilot_imu_is_calibrating();
            bool ready = copilot_imu_is_ready();
            LOGI_APP("IMU status: ready=%d calibrating=%d bias=%.2f dps",
                     ready ? 1 : 0, calibrating ? 1 : 0, bias);
#endif
        }
        if (strcmp(query_str, "power") == 0 || strcmp(query_str, "all") == 0) {
            copilot_power_status_t ps;
            if (copilot_axp2101_get_status(&ps)) {
                char buf[256];
                snprintf(buf, sizeof(buf),
                    "{\"type\":\"status\",\"power\":{"
                    "\"battery_pct\":%d,\"batt_mv\":%u,\"vbus_mv\":%u,\"sys_mv\":%u,"
                    "\"temp_c\":%.1f,\"charging\":%s,\"vbus_in\":%s,\"batt_conn\":%s}}",
                    ps.battery_percent, ps.batt_voltage_mv, ps.vbus_voltage_mv,
                    ps.system_voltage_mv, ps.temperature_c,
                    ps.is_charging ? "true" : "false",
                    ps.is_vbus_in ? "true" : "false",
                    ps.is_battery_connect ? "true" : "false");
                copilot_mqtt_publish("status", buf);
                ESP_LOGI(TAG, "Power status: bat=%d%% %umV vbus=%umV sys=%umV temp=%.1fC chg=%d",
                         ps.battery_percent, ps.batt_voltage_mv, ps.vbus_voltage_mv,
                         ps.system_voltage_mv, ps.temperature_c, ps.is_charging ? 1 : 0);
            } else {
                copilot_mqtt_publish("status", "{\"type\":\"status\",\"power\":{\"error\":\"pmu not ready\"}}");
            }
        }
        if (strcmp(query_str, "voice") == 0 || strcmp(query_str, "all") == 0) {
            copilot_voice_state_t state = copilot_voice_get_state();
            bool active = copilot_voice_is_active();
            bool loopback = copilot_voice_is_loopback_running();
            const char *state_names[] = {"IDLE", "READY", "CONNECTING", "LISTENING", "PROCESSING", "SPEAKING", "ERROR"};
            const char *state_name = (state < sizeof(state_names)/sizeof(state_names[0])) ? state_names[state] : "UNKNOWN";
            ESP_LOGI(TAG, "Voice status: state=%s active=%d loopback=%d", state_name, active ? 1 : 0, loopback ? 1 : 0);
        }
        if (strcmp(query_str, "display") == 0 || strcmp(query_str, "screen") == 0 || strcmp(query_str, "all") == 0) {
            copilot_mqtt_publish("status",
                "{\"type\":\"display_status\","
                "\"robot_display_version\":\"screen_copilot_v1.0\","
                "\"asset_version_hash\":\"face_soft_neutral_20260514\","
                "\"participant_facing_debug\":false,"
                "\"screen_text\":\"\","
                "\"screen_icon\":\"\"}");
        }
    } else if (strcmp(type->valuestring, "servo") == 0) {
#if CONFIG_COPILOT_SERVO_ENABLE
        const cJSON *cmd_mode   = cJSON_GetObjectItemCaseSensitive(root, "mode");
        const cJSON *cmd_calib  = cJSON_GetObjectItemCaseSensitive(root, "calibrate");
        const cJSON *cmd_reset  = cJSON_GetObjectItemCaseSensitive(root, "reset");
        const cJSON *cmd_query  = cJSON_GetObjectItemCaseSensitive(root, "query");
        const cJSON *cmd_pitch  = cJSON_GetObjectItemCaseSensitive(root, "pitch");
        const cJSON *cmd_yaw    = cJSON_GetObjectItemCaseSensitive(root, "yaw");
        const cJSON *cmd_calib_obj = cJSON_GetObjectItemCaseSensitive(root, "calib");

        // Mode switch: "auto" (animation drives servos) or "manual" (MQTT drives servos)
        if (cJSON_IsString(cmd_mode)) {
            if (strcmp(cmd_mode->valuestring, "auto") == 0) {
                copilot_servo_set_manual(false);
                ESP_LOGI(TAG, "Servo mode: AUTO (animation-driven)");
            } else if (strcmp(cmd_mode->valuestring, "manual") == 0) {
                copilot_servo_set_manual(true);
                ESP_LOGI(TAG, "Servo mode: MANUAL (MQTT-driven)");
            }
        }

        if (cJSON_IsBool(cmd_calib)) {
            bool enable = cJSON_IsTrue(cmd_calib);
            copilot_servo_set_calibration(enable);
            ESP_LOGI(TAG, "Servo calibration mode: %s", enable ? "ON" : "OFF");
        } else if (cJSON_IsTrue(cmd_reset)) {
            copilot_servo_reset_calib_to_defaults();
            ESP_LOGI(TAG, "Servo calibration reset to defaults");
        } else if (cJSON_IsString(cmd_query)) {
            const char *q = cmd_query->valuestring;
            if (strcmp(q, "status") == 0 || strcmp(q, "all") == 0) {
                float pitch_deg, yaw_deg;
                copilot_servo_get_current_angle(&pitch_deg, &yaw_deg);
                uint16_t pitch_us, yaw_us;
                copilot_servo_get_pulse_us(&pitch_us, &yaw_us);
                bool calib_mode = copilot_servo_get_calibration();
                bool manual_mode = copilot_servo_get_manual();
                copilot_servo_calib_t calib;
                copilot_servo_get_calib(&calib);
                ESP_LOGI(TAG, "Servo status: pitch=%.1fdeg/%uus yaw=%.1fdeg/%uus calib=%d manual=%d "
                         "soft_p=[%.0f,%.0f] soft_y=[%.0f,%.0f]",
                         pitch_deg, pitch_us, yaw_deg, yaw_us, calib_mode ? 1 : 0, manual_mode ? 1 : 0,
                         calib.pitch.soft_limit_min, calib.pitch.soft_limit_max,
                         calib.yaw.soft_limit_min, calib.yaw.soft_limit_max);
                // Also publish status response
                char buf[256];
                snprintf(buf, sizeof(buf),
                    "{\"type\":\"servo_status\","
                    "\"pitch_deg\":%.1f,\"yaw_deg\":%.1f,"
                    "\"pitch_us\":%u,\"yaw_us\":%u,"
                    "\"calib\":%s,\"manual\":%s,"
                    "\"limits_pitch\":[%.0f,%.0f],\"limits_yaw\":[%.0f,%.0f]}",
                    pitch_deg, yaw_deg, pitch_us, yaw_us,
                    calib_mode ? "true" : "false",
                    manual_mode ? "true" : "false",
                    calib.pitch.soft_limit_min, calib.pitch.soft_limit_max,
                    calib.yaw.soft_limit_min, calib.yaw.soft_limit_max);
                copilot_mqtt_publish("status", buf);
            }
        } else if (cJSON_IsObject(cmd_calib_obj)) {
            copilot_servo_calib_t calib;
            copilot_servo_get_calib(&calib);  // start from current, overlay what's provided
            const cJSON *jp = cJSON_GetObjectItemCaseSensitive(cmd_calib_obj, "pitch");
            const cJSON *jy = cJSON_GetObjectItemCaseSensitive(cmd_calib_obj, "yaw");
            auto apply_ch = [](const cJSON *j, copilot_servo_ch_calib_t *ch) {
                if (!cJSON_IsObject(j)) return;
                const cJSON *v;
                if ((v = cJSON_GetObjectItemCaseSensitive(j, "angle_min")) && cJSON_IsNumber(v))
                    ch->angle_min = v->valuedouble;
                if ((v = cJSON_GetObjectItemCaseSensitive(j, "angle_max")) && cJSON_IsNumber(v))
                    ch->angle_max = v->valuedouble;
                if ((v = cJSON_GetObjectItemCaseSensitive(j, "soft_limit_min")) && cJSON_IsNumber(v))
                    ch->soft_limit_min = v->valuedouble;
                if ((v = cJSON_GetObjectItemCaseSensitive(j, "soft_limit_max")) && cJSON_IsNumber(v))
                    ch->soft_limit_max = v->valuedouble;
                if ((v = cJSON_GetObjectItemCaseSensitive(j, "pulse_min")) && cJSON_IsNumber(v))
                    ch->pulse_min_us = (uint16_t)v->valuedouble;
                if ((v = cJSON_GetObjectItemCaseSensitive(j, "pulse_max")) && cJSON_IsNumber(v))
                    ch->pulse_max_us = (uint16_t)v->valuedouble;
                if ((v = cJSON_GetObjectItemCaseSensitive(j, "pulse_center")) && cJSON_IsNumber(v))
                    ch->pulse_center_us = (uint16_t)v->valuedouble;
            };
            apply_ch(jp, &calib.pitch);
            apply_ch(jy, &calib.yaw);
            copilot_servo_set_calib(&calib);
            ESP_LOGI(TAG, "Servo calibration updated");
        } else if (cJSON_IsNumber(cmd_pitch) || cJSON_IsNumber(cmd_yaw)) {
            float pitch = cJSON_IsNumber(cmd_pitch) ? (float)cmd_pitch->valuedouble : 0.0f;
            float yaw   = cJSON_IsNumber(cmd_yaw)   ? (float)cmd_yaw->valuedouble   : 0.0f;
            if (!copilot_servo_get_manual()) {
                copilot_servo_set_manual(true);  // MQTT takes over from animation
                ESP_LOGI(TAG, "Servo mode auto→manual (MQTT target received)");
            }
            copilot_servo_set_target(pitch, yaw);
            ESP_LOGI(TAG, "Servo target: pitch=%.1f yaw=%.1f", pitch, yaw);
        }
#else
        ESP_LOGW(TAG, "Servo command received but servo module is disabled");
#endif
    } else if (strcmp(type->valuestring, "voice") == 0) {
        // Voice module control
        const cJSON *action = cJSON_GetObjectItemCaseSensitive(root, "action");
        const char *action_str = cJSON_IsString(action) ? action->valuestring : "";

        if (strcmp(action_str, "start") == 0) {
            LOGI_APP("Voice session start requested");
            if (copilot_voice_start_session()) {
                LOGI_APP("Voice session started");
            } else {
                ESP_LOGW(TAG, "Voice session failed to start");
            }
        } else if (strcmp(action_str, "stop") == 0) {
            LOGI_APP("Voice session stop requested");
            copilot_voice_stop_session();
            LOGI_APP("Voice session stopped");
        } else if (strcmp(action_str, "loopback") == 0) {
            // Toggle loopback test
            if (copilot_voice_is_loopback_running()) {
                LOGI_APP("Stopping voice loopback");
                copilot_voice_stop_loopback();
            } else {
                LOGI_APP("Starting voice loopback");
                if (copilot_voice_start_loopback()) {
                    LOGI_APP("Voice loopback started - speak into mic!");
                } else {
                    ESP_LOGW(TAG, "Voice loopback failed to start");
                }
            }
        } else {
            ESP_LOGW(TAG, "Unknown voice action: %s", action_str);
        }

        // Handle volume/gain settings
        const cJSON *volume = cJSON_GetObjectItemCaseSensitive(root, "volume");
        if (cJSON_IsNumber(volume)) {
            int vol = (int)volume->valuedouble;
            if (vol >= 0 && vol <= 100) {
                copilot_voice_set_speaker_volume(vol);
                LOGI_APP("Voice speaker volume set to %d", vol);
            }
        }
        const cJSON *mic_gain = cJSON_GetObjectItemCaseSensitive(root, "mic_gain");
        if (cJSON_IsNumber(mic_gain)) {
            int gain = (int)mic_gain->valuedouble;
            if (gain >= 0 && gain <= 36) {
                copilot_voice_set_mic_gain(gain);
                LOGI_APP("Voice mic gain set to %d dB", gain);
            }
        }
    }

    cJSON_Delete(root);
}

static void copilot_mqtt_cmd_handler(const char *topic, const char *payload, int payload_len) {
    if (!payload || payload_len <= 0) {
        return;
    }
    LOGI_APP( "MQTT cmd topic=%s len=%d", topic ? topic : "null", payload_len);
    copilot_app_handle_command(payload, payload_len);
}

bool copilot_app_format_status(char *out, unsigned out_len) {
    if (!out || out_len == 0) {
        return false;
    }

    copilot_network_status_t net = {};
    copilot_audio_status_t audio = {};
    copilot_mqtt_get_status(&net);
    copilot_audio_get_status(&audio);

    snprintf(out, out_len,
             "{\"type\":\"status\","
             "\"device_id\":\"%s\","
             "\"wifi\":{\"started\":%s,\"connected\":%s,\"ssid\":\"%s\",\"ip\":\"%s\"},"
             "\"mqtt\":{\"started\":%s,\"connected\":%s},"
             "\"tcp_host\":\"%s\","
             "\"audio\":{\"ready\":%s,\"sd_mounted\":%s,\"playing_file\":%s,"
             "\"files_played\":%lu,\"current_path\":\"%s\",\"last_error\":\"%s\"},"
             "\"heap\":{\"internal_free\":%lu,\"psram_free\":%lu}}",
             net.device_id,
             net.wifi_started ? "true" : "false",
             net.wifi_connected ? "true" : "false",
             net.ssid,
             net.ip,
             net.mqtt_started ? "true" : "false",
             net.mqtt_connected ? "true" : "false",
             net.tcp_host,
             audio.ready ? "true" : "false",
             audio.sd_mounted ? "true" : "false",
             audio.playing_file ? "true" : "false",
             (unsigned long)audio.files_played,
             audio.current_path,
             audio.last_error,
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    return true;
}

void copilot_app_init(void) {
    LOGI_APP( "Init copilot app");
    copilot_perf_init();
    copilot_audio_init();
    copilot_imu_init();

    // Initialize voice module (session will start after WiFi connects)
    if (copilot_voice_init()) {
        LOGI_APP( "Voice module initialized");
        copilot_voice_ui_init();  // Register voice-UI callback for mouth animation
#if CONFIG_COPILOT_VOICE_LOOPBACK_TEST
        // Loopback test doesn't need network, start immediately
        if (copilot_voice_start_loopback()) {
            LOGI_APP( "Voice loopback test started - speak into mic!");
        }
#else
        // Streaming session will be started by copilot_mqtt after WiFi connects
        LOGI_APP( "Voice streaming will start after WiFi connects");
#endif
    }
    copilot_ensure_action_task();
    copilot_mqtt_notify_voice_ready();
    copilot_app_network_start();
#if CONFIG_COPILOT_TCP_HOST_ENABLE
    copilot_tcp_host_start(copilot_app_handle_command);
#endif
}

void copilot_app_network_start(void) {
    if (s_network_started) {
        return;
    }
    copilot_ensure_action_task();
    copilot_mqtt_start(copilot_mqtt_cmd_handler);
    s_network_started = true;
}

void copilot_app_ui_init(lv_obj_t *root) {
    LOGI_APP( "Init copilot UI (root=%p)", root);
    copilot_ui_init(root);
}

void copilot_app_on_touch(uint16_t x, uint16_t y) {
    ESP_LOGD(TAG, "Touch x=%u y=%u", (unsigned)x, (unsigned)y);
    copilot_ui_on_touch(x, y);
}
