#ifndef COPILOT_VOICE_H
#define COPILOT_VOICE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Voice module state
 */
typedef enum {
    VOICE_STATE_IDLE = 0,      ///< Not initialized or stopped
    VOICE_STATE_READY,         ///< Initialized, waiting for connection
    VOICE_STATE_CONNECTING,    ///< Connecting to backend
    VOICE_STATE_LISTENING,     ///< Connected, capturing microphone
    VOICE_STATE_PROCESSING,    ///< Backend processing (ASR/AI)
    VOICE_STATE_SPEAKING,      ///< Playing TTS response
    VOICE_STATE_ERROR,         ///< Error state
} copilot_voice_state_t;

/**
 * @brief Voice event callback type
 */
typedef void (*copilot_voice_event_cb_t)(copilot_voice_state_t state, void *user_data);

/**
 * @brief Initialize voice module
 *
 * Initializes audio hardware (ES8311 codec) for both microphone capture
 * and speaker playback. Does not start WebRTC connection.
 *
 * @return true on success, false on error
 */
bool copilot_voice_init(void);

/**
 * @brief Deinitialize voice module
 *
 * Stops all audio tasks and releases resources.
 */
void copilot_voice_deinit(void);

/**
 * @brief Start voice session
 *
 * Connects to the Python backend via WebRTC signaling and starts
 * bidirectional audio streaming.
 *
 * @return true if session started, false on error
 */
bool copilot_voice_start_session(void);

/**
 * @brief Stop voice session
 *
 * Disconnects from backend and stops audio streaming.
 * Audio hardware remains initialized.
 */
void copilot_voice_stop_session(void);

/**
 * @brief Get current voice state
 *
 * @return Current state of the voice module
 */
copilot_voice_state_t copilot_voice_get_state(void);

/**
 * @brief Check if voice module is ready
 *
 * @return true if initialized and ready for session
 */
bool copilot_voice_is_ready(void);

/**
 * @brief Check if voice session is active
 *
 * @return true if connected and streaming
 */
bool copilot_voice_is_active(void);

/**
 * @brief Check if loopback test is running
 *
 * @return true if loopback is running
 */
bool copilot_voice_is_loopback_running(void);

/**
 * @brief Register event callback
 *
 * @param cb Callback function
 * @param user_data User data passed to callback
 */
void copilot_voice_set_event_callback(copilot_voice_event_cb_t cb, void *user_data);

/**
 * @brief Start audio loopback test
 *
 * Routes microphone input directly to speaker output for testing.
 * Only available when CONFIG_COPILOT_VOICE_LOOPBACK_TEST is enabled.
 *
 * @return true if loopback started, false on error
 */
bool copilot_voice_start_loopback(void);

/**
 * @brief Stop audio loopback test
 */
void copilot_voice_stop_loopback(void);

/**
 * @brief Set microphone gain
 *
 * @param gain_db Gain in dB (0-36)
 */
void copilot_voice_set_mic_gain(int gain_db);

/**
 * @brief Set speaker volume
 *
 * @param volume_percent Volume percentage (0-100)
 */
void copilot_voice_set_speaker_volume(int volume_percent);

#ifdef __cplusplus
}
#endif

#endif // COPILOT_VOICE_H
