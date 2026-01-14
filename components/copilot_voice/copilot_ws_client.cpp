/**
 * @file copilot_ws_client.cpp
 * @brief WebSocket client for audio streaming with Python backend
 */

#include "copilot_ws_client.h"

#include <string.h>
#include <cJSON.h>

#include "esp_log.h"
#include "esp_websocket_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "ws_client";

// ============================================================================
// Configuration and State
// ============================================================================

typedef struct {
    esp_websocket_client_handle_t client;
    copilot_ws_client_state_t state;

    // Configuration
    char server_url[256];
    char device_id[64];
    int sample_rate;

    // Callbacks
    copilot_ws_audio_cb_t on_audio;
    copilot_ws_state_cb_t on_state;
    void *user_data;

    // Session
    char session_id[64];

    // Synchronization
    SemaphoreHandle_t mutex;
    bool initialized;
} ws_client_ctx_t;

static ws_client_ctx_t s_ctx = {};

// ============================================================================
// Internal Functions
// ============================================================================

static void set_state(copilot_ws_client_state_t new_state) {
    if (s_ctx.state == new_state) {
        return;
    }

    s_ctx.state = new_state;
    ESP_LOGI(TAG, "State: %d", (int)new_state);

    if (s_ctx.on_state) {
        s_ctx.on_state(new_state, s_ctx.user_data);
    }
}

static void send_start_message(void) {
    if (!s_ctx.client) {
        return;
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "start");
    cJSON_AddStringToObject(root, "device_id", s_ctx.device_id);
    cJSON_AddNumberToObject(root, "sample_rate", s_ctx.sample_rate);

    char *json_str = cJSON_PrintUnformatted(root);
    if (json_str) {
        esp_websocket_client_send_text(s_ctx.client, json_str, strlen(json_str), portMAX_DELAY);
        ESP_LOGI(TAG, "Sent start message: %s", json_str);
        free(json_str);
    }

    cJSON_Delete(root);
}

static void send_stop_message(void) {
    if (!s_ctx.client) {
        return;
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "stop");

    char *json_str = cJSON_PrintUnformatted(root);
    if (json_str) {
        esp_websocket_client_send_text(s_ctx.client, json_str, strlen(json_str), portMAX_DELAY);
        ESP_LOGI(TAG, "Sent stop message");
        free(json_str);
    }

    cJSON_Delete(root);
}

static void handle_text_message(const char *data, int len) {
    cJSON *root = cJSON_ParseWithLength(data, len);
    if (!root) {
        ESP_LOGW(TAG, "Failed to parse JSON: %.*s", len, data);
        return;
    }

    cJSON *type = cJSON_GetObjectItem(root, "type");
    if (type && cJSON_IsString(type)) {
        if (strcmp(type->valuestring, "started") == 0) {
            // Session started
            cJSON *session_id = cJSON_GetObjectItem(root, "session_id");
            if (session_id && cJSON_IsString(session_id)) {
                strncpy(s_ctx.session_id, session_id->valuestring, sizeof(s_ctx.session_id) - 1);
                ESP_LOGI(TAG, "Session started: %s", s_ctx.session_id);
            }
            set_state(WS_CLIENT_STATE_STREAMING);

        } else if (strcmp(type->valuestring, "error") == 0) {
            cJSON *message = cJSON_GetObjectItem(root, "message");
            ESP_LOGE(TAG, "Server error: %s",
                     (message && cJSON_IsString(message)) ? message->valuestring : "unknown");
            set_state(WS_CLIENT_STATE_ERROR);
        }
    }

    cJSON_Delete(root);
}

static void handle_binary_message(const uint8_t *data, int len) {
    // Binary data is PCM audio from server (TTS output)
    if (s_ctx.on_audio && len > 0) {
        // Data is int16_t samples
        size_t samples = len / sizeof(int16_t);
        s_ctx.on_audio((const int16_t *)data, samples, s_ctx.user_data);
    }
}

static void websocket_event_handler(void *arg, esp_event_base_t event_base,
                                    int32_t event_id, void *event_data) {
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;

    switch (event_id) {
        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "WebSocket connected");
            set_state(WS_CLIENT_STATE_CONNECTED);
            // Send start message
            send_start_message();
            break;

        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "WebSocket disconnected");
            set_state(WS_CLIENT_STATE_IDLE);
            break;

        case WEBSOCKET_EVENT_DATA:
            if (data->op_code == 0x01) {
                // Text frame
                handle_text_message((const char *)data->data_ptr, data->data_len);
            } else if (data->op_code == 0x02) {
                // Binary frame
                handle_binary_message((const uint8_t *)data->data_ptr, data->data_len);
            }
            break;

        case WEBSOCKET_EVENT_ERROR:
            ESP_LOGE(TAG, "WebSocket error");
            set_state(WS_CLIENT_STATE_ERROR);
            break;

        default:
            break;
    }
}

// ============================================================================
// Public API
// ============================================================================

bool copilot_ws_client_init(const copilot_ws_client_config_t *config) {
    if (s_ctx.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return true;
    }

    if (!config || !config->server_url) {
        ESP_LOGE(TAG, "Invalid config");
        return false;
    }

    memset(&s_ctx, 0, sizeof(s_ctx));

    // Copy configuration
    strncpy(s_ctx.server_url, config->server_url, sizeof(s_ctx.server_url) - 1);
    strncpy(s_ctx.device_id, config->device_id ? config->device_id : "esp32_copilot",
            sizeof(s_ctx.device_id) - 1);
    s_ctx.sample_rate = config->sample_rate > 0 ? config->sample_rate : 16000;
    s_ctx.on_audio = config->on_audio;
    s_ctx.on_state = config->on_state;
    s_ctx.user_data = config->user_data;

    // Create mutex
    s_ctx.mutex = xSemaphoreCreateMutex();
    if (!s_ctx.mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return false;
    }

    s_ctx.state = WS_CLIENT_STATE_IDLE;
    s_ctx.initialized = true;

    ESP_LOGI(TAG, "Initialized (server=%s, device=%s, rate=%d)",
             s_ctx.server_url, s_ctx.device_id, s_ctx.sample_rate);

    return true;
}

void copilot_ws_client_deinit(void) {
    if (!s_ctx.initialized) {
        return;
    }

    copilot_ws_client_disconnect();

    if (s_ctx.mutex) {
        vSemaphoreDelete(s_ctx.mutex);
        s_ctx.mutex = NULL;
    }

    s_ctx.initialized = false;
    ESP_LOGI(TAG, "Deinitialized");
}

bool copilot_ws_client_connect(void) {
    if (!s_ctx.initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return false;
    }

    if (s_ctx.client) {
        ESP_LOGW(TAG, "Already connected");
        return true;
    }

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

    esp_websocket_client_config_t ws_cfg = {};
    ws_cfg.uri = s_ctx.server_url;
    ws_cfg.buffer_size = 4096;
    ws_cfg.reconnect_timeout_ms = 5000;
    ws_cfg.network_timeout_ms = 10000;

    s_ctx.client = esp_websocket_client_init(&ws_cfg);
    if (!s_ctx.client) {
        ESP_LOGE(TAG, "Failed to create WebSocket client");
        xSemaphoreGive(s_ctx.mutex);
        return false;
    }

    esp_websocket_register_events(s_ctx.client, WEBSOCKET_EVENT_ANY,
                                  websocket_event_handler, NULL);

    set_state(WS_CLIENT_STATE_CONNECTING);

    esp_err_t err = esp_websocket_client_start(s_ctx.client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WebSocket client: %s", esp_err_to_name(err));
        esp_websocket_client_destroy(s_ctx.client);
        s_ctx.client = NULL;
        set_state(WS_CLIENT_STATE_ERROR);
        xSemaphoreGive(s_ctx.mutex);
        return false;
    }

    xSemaphoreGive(s_ctx.mutex);

    ESP_LOGI(TAG, "Connecting to %s...", s_ctx.server_url);
    return true;
}

void copilot_ws_client_disconnect(void) {
    if (!s_ctx.client) {
        return;
    }

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

    set_state(WS_CLIENT_STATE_DISCONNECTING);

    // Send stop message
    if (esp_websocket_client_is_connected(s_ctx.client)) {
        send_stop_message();
        vTaskDelay(pdMS_TO_TICKS(100));  // Allow message to be sent
    }

    esp_websocket_client_stop(s_ctx.client);
    esp_websocket_client_destroy(s_ctx.client);
    s_ctx.client = NULL;

    s_ctx.session_id[0] = '\0';
    set_state(WS_CLIENT_STATE_IDLE);

    xSemaphoreGive(s_ctx.mutex);

    ESP_LOGI(TAG, "Disconnected");
}

bool copilot_ws_client_send_audio(const int16_t *pcm_data, size_t samples) {
    if (!s_ctx.client || s_ctx.state != WS_CLIENT_STATE_STREAMING) {
        return false;
    }

    if (!pcm_data || samples == 0) {
        return false;
    }

    size_t len = samples * sizeof(int16_t);

    int ret = esp_websocket_client_send_bin(s_ctx.client, (const char *)pcm_data, len, portMAX_DELAY);
    if (ret < 0) {
        ESP_LOGW(TAG, "Failed to send audio: %d", ret);
        return false;
    }

    return true;
}

copilot_ws_client_state_t copilot_ws_client_get_state(void) {
    return s_ctx.state;
}

bool copilot_ws_client_is_streaming(void) {
    return s_ctx.state == WS_CLIENT_STATE_STREAMING;
}
