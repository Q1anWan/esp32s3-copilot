#include "copilot_app.h"

#include <string.h>
#include <math.h>

#include "cJSON.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "copilot_audio.h"
#include "copilot_imu.h"
#include "copilot_mqtt.h"
#include "copilot_perf.h"
#include "copilot_ui.h"
#include "copilot_voice.h"
#include "copilot_voice_ui.h"

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
};

struct copilot_action_t {
    action_type_t type;
    copilot_expr_t expr;
    uint32_t duration_ms;
    uint32_t prelight_ms;
    char sound_id[16];
};

static QueueHandle_t s_action_queue = nullptr;
static TaskHandle_t s_action_task = nullptr;

static void copilot_action_task(void *arg) {
    (void)arg;
    copilot_action_t action = {};
    for (;;) {
        if (xQueueReceive(s_action_queue, &action, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (action.prelight_ms > 0) {
            copilot_ui_ring_show_async(true);
            vTaskDelay(pdMS_TO_TICKS(action.prelight_ms));
        }
        copilot_ui_ring_show_async(false);

        if (action.type == ACTION_EXPR) {
            LOGI_APP( "Apply expression=%d duration=%u sound=%s", (int)action.expr, (unsigned)action.duration_ms,
                     action.sound_id[0] ? action.sound_id : "none");
            copilot_ui_set_expression_async(action.expr, action.duration_ms);
        } else if (action.type == ACTION_SOUND) {
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
    {"neutral", COPILOT_EXPR_NEUTRAL},
    {"happy", COPILOT_EXPR_HAPPY},
    {"sad", COPILOT_EXPR_SAD},
    {"angry", COPILOT_EXPR_ANGRY},
    {"surprised", COPILOT_EXPR_SURPRISED},
    {"sleepy", COPILOT_EXPR_SLEEPY},
    {"dizzy", COPILOT_EXPR_DIZZY},
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

static void copilot_schedule_expression(copilot_expr_t expr, uint32_t duration_ms, uint32_t prelight_ms, const char *sound_id) {
    copilot_action_t action = {};
    action.type = ACTION_EXPR;
    action.expr = expr;
    action.duration_ms = duration_ms;
    action.prelight_ms = prelight_ms;
    if (sound_id && sound_id[0] != '\0') {
        strncpy(action.sound_id, sound_id, sizeof(action.sound_id) - 1);
        action.sound_id[sizeof(action.sound_id) - 1] = '\0';
    }
    LOGI_APP( "Schedule expression=%d duration=%u prelight=%u sound=%s", (int)expr, (unsigned)duration_ms,
             (unsigned)prelight_ms, sound_id ? sound_id : "none");
    copilot_enqueue_action(&action);
}

static void copilot_schedule_sound(const char *sound_id, uint32_t prelight_ms) {
    copilot_action_t action = {};
    action.type = ACTION_SOUND;
    action.expr = COPILOT_EXPR_NEUTRAL;
    action.duration_ms = 0;
    action.prelight_ms = prelight_ms;
    if (sound_id && sound_id[0] != '\0') {
        strncpy(action.sound_id, sound_id, sizeof(action.sound_id) - 1);
        action.sound_id[sizeof(action.sound_id) - 1] = '\0';
    }
    LOGI_APP( "Schedule sound=%s prelight=%u", sound_id ? sound_id : "none", (unsigned)prelight_ms);
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

static void copilot_handle_payload(const char *payload, int payload_len) {
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

    if (strcmp(type->valuestring, "motion") == 0) {
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

        const char *sound_id = nullptr;
        const cJSON *sound = cJSON_GetObjectItemCaseSensitive(root, "sound");
        if (cJSON_IsString(sound)) {
            sound_id = sound->valuestring;
        } else if (cJSON_IsTrue(sound)) {
            sound_id = "beep_short";
        }

        LOGI_APP( "Expression cmd: name=%s id=%s expr=%d duration=%u prelight=%u sound=%s",
                 cJSON_IsString(name) ? name->valuestring : "null",
                 cJSON_IsNumber(id) ? "num" : "null",
                 (int)expr, (unsigned)duration_ms, (unsigned)prelight_ms,
                 sound_id ? sound_id : "none");
        copilot_schedule_expression(expr, duration_ms, prelight_ms, sound_id);
    } else if (strcmp(type->valuestring, "sound") == 0) {
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
        if (strcmp(query_str, "voice") == 0 || strcmp(query_str, "all") == 0) {
            copilot_voice_state_t state = copilot_voice_get_state();
            bool active = copilot_voice_is_active();
            bool loopback = copilot_voice_is_loopback_running();
            const char *state_names[] = {"IDLE", "READY", "CONNECTING", "LISTENING", "PROCESSING", "SPEAKING", "ERROR"};
            const char *state_name = (state < sizeof(state_names)/sizeof(state_names[0])) ? state_names[state] : "UNKNOWN";
            ESP_LOGI(TAG, "Voice status: state=%s active=%d loopback=%d", state_name, active ? 1 : 0, loopback ? 1 : 0);
        }
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
    copilot_handle_payload(payload, payload_len);
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
    if (!s_action_queue) {
        s_action_queue = xQueueCreate(8, sizeof(copilot_action_t));
        if (s_action_queue) {
            int core = copilot_normalize_core(CONFIG_COPILOT_ACTION_CORE);
            BaseType_t task_ok;
            // Priority lowered from 4 to 3 to reduce UI impact during MQTT processing
            // Keep 3KB stack - audio playback path needs more stack
            if (core >= 0) {
                task_ok = xTaskCreatePinnedToCore(copilot_action_task, "copilot_action", 3 * 1024, nullptr, 3,
                                                  &s_action_task, core);
            } else {
                task_ok = xTaskCreate(copilot_action_task, "copilot_action", 3 * 1024, nullptr, 3, &s_action_task);
            }
            if (task_ok != pdPASS) {
                ESP_LOGE(TAG, "Failed to create action task");
            } else {
                LOGI_APP( "Action task core=%d", core);
            }
        } else {
            ESP_LOGE(TAG, "Failed to create action queue");
        }
    }
    copilot_mqtt_start(copilot_mqtt_cmd_handler);
}

void copilot_app_ui_init(lv_obj_t *root) {
    LOGI_APP( "Init copilot UI (root=%p)", root);
    copilot_ui_init(root);
}

void copilot_app_on_touch(uint16_t x, uint16_t y) {
    ESP_LOGD(TAG, "Touch x=%u y=%u", (unsigned)x, (unsigned)y);
    copilot_ui_on_touch(x, y);
}
