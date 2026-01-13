#include "copilot_app.h"

#include <string.h>

#include "cJSON.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "copilot_audio.h"
#include "copilot_mqtt.h"
#include "copilot_perf.h"
#include "copilot_ui.h"

static const char *TAG = "copilot_app";

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
            ESP_LOGI(TAG, "Apply expression=%d duration=%u sound=%s", (int)action.expr, (unsigned)action.duration_ms,
                     action.sound_id[0] ? action.sound_id : "none");
            copilot_ui_set_expression_async(action.expr, action.duration_ms);
        } else if (action.type == ACTION_SOUND) {
            ESP_LOGI(TAG, "Play sound=%s", action.sound_id[0] ? action.sound_id : "none");
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
    ESP_LOGI(TAG, "Schedule expression=%d duration=%u prelight=%u sound=%s", (int)expr, (unsigned)duration_ms,
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
    ESP_LOGI(TAG, "Schedule sound=%s prelight=%u", sound_id ? sound_id : "none", (unsigned)prelight_ms);
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
    ESP_LOGI(TAG, "MQTT payload: %.*s", payload_len, payload);
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
        motion.ax = copilot_get_number_fp(root, "ax", 0);
        motion.ay = copilot_get_number_fp(root, "ay", 0);
        motion.yaw_deg = copilot_get_number_fp(root, "yaw", 0);
        motion.speed = copilot_get_number_fp(root, "speed", 0);
        ESP_LOGI(TAG, "Motion ax=%d ay=%d yaw=%d speed=%d (Q8.8)", motion.ax, motion.ay, motion.yaw_deg, motion.speed);
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

        ESP_LOGI(TAG, "Expression cmd: name=%s id=%s expr=%d duration=%u prelight=%u sound=%s",
                 cJSON_IsString(name) ? name->valuestring : "null",
                 cJSON_IsNumber(id) ? "num" : "null",
                 (int)expr, (unsigned)duration_ms, (unsigned)prelight_ms,
                 sound_id ? sound_id : "none");
        copilot_schedule_expression(expr, duration_ms, prelight_ms, sound_id);
    } else if (strcmp(type->valuestring, "sound") == 0) {
        const cJSON *id = cJSON_GetObjectItemCaseSensitive(root, "id");
        const char *sound_id = cJSON_IsString(id) ? id->valuestring : "beep_short";
        uint32_t prelight_ms = copilot_get_u32(root, "prelight_ms", CONFIG_COPILOT_PRELIGHT_MS);
        ESP_LOGI(TAG, "Sound id=%s prelight=%u", sound_id, (unsigned)prelight_ms);
        copilot_schedule_sound(sound_id, prelight_ms);
    } else if (strcmp(type->valuestring, "ring") == 0) {
        const cJSON *on = cJSON_GetObjectItemCaseSensitive(root, "on");
        if (cJSON_IsBool(on)) {
            ESP_LOGI(TAG, "Ring on=%s", cJSON_IsTrue(on) ? "true" : "false");
            copilot_ui_ring_show_async(cJSON_IsTrue(on));
        }
    }

    cJSON_Delete(root);
}

static void copilot_mqtt_cmd_handler(const char *topic, const char *payload, int payload_len) {
    if (!payload || payload_len <= 0) {
        return;
    }
    ESP_LOGI(TAG, "MQTT cmd topic=%s len=%d", topic ? topic : "null", payload_len);
    copilot_handle_payload(payload, payload_len);
}

void copilot_app_init(void) {
    ESP_LOGI(TAG, "Init copilot app");
    copilot_perf_init();
    copilot_audio_init();
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
                ESP_LOGI(TAG, "Action task core=%d", core);
            }
        } else {
            ESP_LOGE(TAG, "Failed to create action queue");
        }
    }
    copilot_mqtt_start(copilot_mqtt_cmd_handler);
}

void copilot_app_ui_init(lv_obj_t *root) {
    ESP_LOGI(TAG, "Init copilot UI (root=%p)", root);
    copilot_ui_init(root);
}

void copilot_app_on_touch(uint16_t x, uint16_t y) {
    ESP_LOGD(TAG, "Touch x=%u y=%u", (unsigned)x, (unsigned)y);
    copilot_ui_on_touch(x, y);
}
