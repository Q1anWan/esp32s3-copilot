/**
 * @file copilot_ws_client.cpp
 * @brief WebSocket client for audio streaming with Python backend
 */

#include "copilot_ws_client.h"

#include <string.h>
#include <cJSON.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_websocket_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "ws_client";

// ============================================================================
// Configuration and State
// ============================================================================

#define WS_AUDIO_SEND_TIMEOUT_MS 1000

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

    // Connection tracking
    uint32_t connect_attempts;
    uint32_t last_connect_time;

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
        case WEBSOCKET_EVENT_BEFORE_CONNECT:
            s_ctx.connect_attempts++;
            s_ctx.last_connect_time = (uint32_t)(esp_timer_get_time() / 1000);
            ESP_LOGI(TAG, "Connecting... (attempt #%lu)", (unsigned long)s_ctx.connect_attempts);
            // Ensure we're in CONNECTING state for reconnect attempts
            if (s_ctx.state != WS_CLIENT_STATE_CONNECTING) {
                set_state(WS_CLIENT_STATE_CONNECTING);
            }
            break;

        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "WebSocket connected after %lu attempts", (unsigned long)s_ctx.connect_attempts);
            s_ctx.connect_attempts = 0;  // Reset counter on success
            s_ctx.session_id[0] = '\0';  // Clear old session ID
            set_state(WS_CLIENT_STATE_CONNECTED);
            // Send start message to initiate session
            send_start_message();
            break;

        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "WebSocket disconnected (auto-reconnect enabled)");
            // Reset session and go back to CONNECTING for auto-reconnect
            s_ctx.session_id[0] = '\0';
            if (s_ctx.state == WS_CLIENT_STATE_STREAMING ||
                s_ctx.state == WS_CLIENT_STATE_CONNECTED) {
                set_state(WS_CLIENT_STATE_CONNECTING);
            }
            // Note: esp_websocket_client handles auto-reconnect internally
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
            ESP_LOGW(TAG, "WebSocket error (state=%d, attempt=%lu, heap_free=%lu)",
                     (int)s_ctx.state, (unsigned long)s_ctx.connect_attempts,
                     (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
            // For connection errors, stay in CONNECTING to allow auto-reconnect
            // For streaming errors, also go to CONNECTING to allow recovery
            if (s_ctx.state == WS_CLIENT_STATE_CONNECTING) {
                // Connection attempt failed - esp_websocket_client will retry
                ESP_LOGI(TAG, "Server unreachable, retrying...");
            } else if (s_ctx.state == WS_CLIENT_STATE_STREAMING ||
                       s_ctx.state == WS_CLIENT_STATE_CONNECTED) {
                // Connection lost during session - go back to connecting
                s_ctx.session_id[0] = '\0';
                set_state(WS_CLIENT_STATE_CONNECTING);
            }
            break;

        case WEBSOCKET_EVENT_CLOSED:
            ESP_LOGI(TAG, "WebSocket closed by server");
            s_ctx.session_id[0] = '\0';
            // Go to CONNECTING to allow auto-reconnect (not IDLE)
            set_state(WS_CLIENT_STATE_CONNECTING);
            break;

        default:
            ESP_LOGD(TAG, "WebSocket event: %ld", event_id);
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
        // Already have a client - check if it's trying to connect
        if (s_ctx.state == WS_CLIENT_STATE_CONNECTING) {
            ESP_LOGI(TAG, "Already connecting (attempt #%lu)", (unsigned long)s_ctx.connect_attempts);
            return true;
        }
        ESP_LOGW(TAG, "Client exists, state=%d", (int)s_ctx.state);
        return true;
    }

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

    // Reset connection tracking
    s_ctx.connect_attempts = 0;
    s_ctx.last_connect_time = 0;
    s_ctx.session_id[0] = '\0';

    esp_websocket_client_config_t ws_cfg = {};
    ws_cfg.uri = s_ctx.server_url;
    ws_cfg.buffer_size = 2048;             // Buffer for 2-3 audio frames (640 bytes + overhead each)
    ws_cfg.reconnect_timeout_ms = 3000;    // Retry every 3s on failure
    ws_cfg.network_timeout_ms = 10000;     // Connection timeout 10s
    ws_cfg.ping_interval_sec = 5;          // Keep-alive ping every 5s
    ws_cfg.pingpong_timeout_sec = 10;      // Pong response timeout 10s
    ws_cfg.disable_auto_reconnect = false; // Enable auto-reconnect (default)
    ws_cfg.task_stack = 4096;              // WebSocket task stack size

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
    static uint32_t send_count = 0;
    static uint32_t last_log_time = 0;
    static uint32_t consecutive_failures = 0;

    if (!s_ctx.client || s_ctx.state != WS_CLIENT_STATE_STREAMING) {
        static uint32_t skip_count = 0;
        skip_count++;
        if (skip_count <= 3 || skip_count % 100 == 0) {
            ESP_LOGW(TAG, "send_audio skipped: client=%p, state=%d (count=%lu)",
                     s_ctx.client, (int)s_ctx.state, (unsigned long)skip_count);
        }
        return false;
    }

    // Double-check connection is actually connected
    if (!esp_websocket_client_is_connected(s_ctx.client)) {
        static uint32_t not_connected_count = 0;
        not_connected_count++;
        if (not_connected_count <= 3 || not_connected_count % 100 == 0) {
            ESP_LOGW(TAG, "send_audio: not connected (state=%d, count=%lu)",
                     (int)s_ctx.state, (unsigned long)not_connected_count);
        }
        return false;
    }

    if (!pcm_data || samples == 0) {
        return false;
    }

    size_t len = samples * sizeof(int16_t);

    // Debug: log first send attempt
    static bool first_send_logged = false;
    if (!first_send_logged) {
        ESP_LOGI(TAG, "About to send first audio frame (%d bytes)", (int)len);
        first_send_logged = true;
    }

    // Use longer timeout to tolerate WiFi jitter and avoid poll timeouts
    // Note: If send fails, the connection may have dropped - auto-reconnect will handle it
    int ret = esp_websocket_client_send_bin(
        s_ctx.client, (const char *)pcm_data, len, pdMS_TO_TICKS(WS_AUDIO_SEND_TIMEOUT_MS));

    // Debug: log first send result
    static bool first_result_logged = false;
    if (!first_result_logged) {
        ESP_LOGI(TAG, "First send result: ret=%d (expected=%d)", ret, (int)len);
        first_result_logged = true;
    }

    if (ret != (int)len) {
        consecutive_failures++;
        if (consecutive_failures <= 5 || consecutive_failures % 100 == 0) {
            ESP_LOGW(TAG, "WebSocket send failed: ret=%d len=%d (consecutive=%lu, connected=%d)",
                     ret, (int)len, (unsigned long)consecutive_failures,
                     esp_websocket_client_is_connected(s_ctx.client));
        }
        // If we have many consecutive failures, connection might be dead
        if (consecutive_failures >= 10) {
            ESP_LOGW(TAG, "Too many send failures, connection may be dead");
        }
        return false;
    }

    // Reset consecutive failure count on success
    consecutive_failures = 0;
    send_count++;

    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (now - last_log_time >= 5000) {
        last_log_time = now;
        ESP_LOGI(TAG, "Audio sent: %lu frames (%d bytes each)", (unsigned long)send_count, (int)len);
    }

    return true;
}

copilot_ws_client_state_t copilot_ws_client_get_state(void) {
    return s_ctx.state;
}

bool copilot_ws_client_is_streaming(void) {
    return s_ctx.state == WS_CLIENT_STATE_STREAMING;
}

bool copilot_ws_client_is_connected(void) {
    return s_ctx.client && esp_websocket_client_is_connected(s_ctx.client);
}

bool copilot_ws_client_is_connecting(void) {
    return s_ctx.state == WS_CLIENT_STATE_CONNECTING;
}

uint32_t copilot_ws_client_get_connect_attempts(void) {
    return s_ctx.connect_attempts;
}

bool copilot_ws_client_force_reconnect(void) {
    if (!s_ctx.initialized || !s_ctx.client) {
        return false;
    }

    ESP_LOGI(TAG, "Forcing reconnect...");

    // Stop current connection
    esp_websocket_client_stop(s_ctx.client);

    // Reset state
    s_ctx.session_id[0] = '\0';
    s_ctx.connect_attempts = 0;
    set_state(WS_CLIENT_STATE_CONNECTING);

    // Restart
    esp_err_t err = esp_websocket_client_start(s_ctx.client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Force reconnect failed: %s", esp_err_to_name(err));
        return false;
    }

    return true;
}
