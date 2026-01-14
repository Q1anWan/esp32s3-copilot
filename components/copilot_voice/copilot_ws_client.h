/**
 * @file copilot_ws_client.h
 * @brief WebSocket client for audio streaming with Python backend
 *
 * Simple WebSocket-based audio streaming protocol:
 * - Binary frames: Raw PCM audio (int16, 16kHz, mono)
 * - Text frames: JSON control messages
 *
 * Protocol:
 * 1. Connect to ws://server:port/audio/stream
 * 2. Send: {"type": "start", "device_id": "xxx"}
 * 3. Receive: {"type": "started", "session_id": "xxx"}
 * 4. Send/receive binary PCM audio frames
 * 5. Send: {"type": "stop"} to end session
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief WebSocket client state
 */
typedef enum {
    WS_CLIENT_STATE_IDLE = 0,
    WS_CLIENT_STATE_CONNECTING,
    WS_CLIENT_STATE_CONNECTED,
    WS_CLIENT_STATE_STREAMING,
    WS_CLIENT_STATE_DISCONNECTING,
    WS_CLIENT_STATE_ERROR,
} copilot_ws_client_state_t;

/**
 * @brief Callback for received audio data
 *
 * @param pcm_data PCM audio data (int16, mono)
 * @param len Length in bytes
 * @param user_data User context
 */
typedef void (*copilot_ws_audio_cb_t)(const int16_t *pcm_data, size_t len, void *user_data);

/**
 * @brief Callback for state changes
 *
 * @param state New state
 * @param user_data User context
 */
typedef void (*copilot_ws_state_cb_t)(copilot_ws_client_state_t state, void *user_data);

/**
 * @brief WebSocket client configuration
 */
typedef struct {
    const char *server_url;      /**< Server URL (e.g., "ws://192.168.1.100:8080/audio/stream") */
    const char *device_id;       /**< Device identifier */
    int sample_rate;             /**< Audio sample rate (default: 16000) */
    copilot_ws_audio_cb_t on_audio;  /**< Audio receive callback */
    copilot_ws_state_cb_t on_state;  /**< State change callback */
    void *user_data;             /**< User context for callbacks */
} copilot_ws_client_config_t;

/**
 * @brief Initialize WebSocket client
 *
 * @param config Client configuration
 * @return true on success
 */
bool copilot_ws_client_init(const copilot_ws_client_config_t *config);

/**
 * @brief Deinitialize WebSocket client
 */
void copilot_ws_client_deinit(void);

/**
 * @brief Connect to server and start audio session
 *
 * @return true on success
 */
bool copilot_ws_client_connect(void);

/**
 * @brief Disconnect from server
 */
void copilot_ws_client_disconnect(void);

/**
 * @brief Send audio data to server
 *
 * @param pcm_data PCM audio samples (int16, mono)
 * @param samples Number of samples
 * @return true on success
 */
bool copilot_ws_client_send_audio(const int16_t *pcm_data, size_t samples);

/**
 * @brief Get current client state
 *
 * @return Current state
 */
copilot_ws_client_state_t copilot_ws_client_get_state(void);

/**
 * @brief Check if client is connected and streaming
 *
 * @return true if streaming
 */
bool copilot_ws_client_is_streaming(void);

#ifdef __cplusplus
}
#endif
