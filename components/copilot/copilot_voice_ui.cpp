/**
 * @file copilot_voice_ui.cpp
 * @brief Voice-UI Integration Bridge Implementation
 */

#include "copilot_voice_ui.h"
#include "copilot_audio_out.h"
#include "copilot_voice.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "voice_ui";

// Conditional logging
#if CONFIG_COPILOT_LOG_VOICE
#define LOGI_VUI(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#else
#define LOGI_VUI(fmt, ...) do {} while(0)
#endif

// Atomic voice state (single byte is naturally atomic on ESP32)
static volatile copilot_voice_state_t s_voice_state = VOICE_STATE_IDLE;

/**
 * @brief Voice state change callback
 *
 * Called from voice module task when state changes.
 * Updates shared state for UI to read.
 */
static void voice_state_callback(copilot_voice_state_t state, void *user_data) {
    (void)user_data;
    s_voice_state = state;
    LOGI_VUI("Voice state changed: %d", (int)state);
}

void copilot_voice_ui_init(void) {
    copilot_voice_set_event_callback(voice_state_callback, NULL);
    ESP_LOGI(TAG, "Voice-UI integration initialized");
}

uint8_t copilot_voice_ui_get_mouth_open(void) {
    // Check if voice audio is being output - envelope is valid when:
    // 1. State is SPEAKING (TTS from server)
    // 2. Loopback is running (mic -> speaker)
    // 3. Voice source is actively playing audio
    copilot_voice_state_t state = s_voice_state;

    if (state == VOICE_STATE_SPEAKING) {
        return copilot_audio_out_get_envelope();
    }

    // During loopback test, mouth should animate with voice
    if (copilot_voice_is_loopback_running()) {
        return copilot_audio_out_get_envelope();
    }

    // Check if voice source is currently active (handles TTS playback)
    if (copilot_audio_out_get_active() == AUDIO_SRC_VOICE) {
        return copilot_audio_out_get_envelope();
    }

    return 0;
}

copilot_voice_state_t copilot_voice_ui_get_state(void) {
    return s_voice_state;
}
