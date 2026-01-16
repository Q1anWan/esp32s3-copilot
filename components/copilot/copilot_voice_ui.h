/**
 * @file copilot_voice_ui.h
 * @brief Voice-UI Integration Bridge
 *
 * Bridges the voice module with the UI for visual feedback:
 * - Voice state notifications (listening, speaking, error)
 * - Audio envelope for mouth animation
 *
 * Uses atomic shared state for efficient cross-thread communication.
 */

#ifndef COPILOT_VOICE_UI_H
#define COPILOT_VOICE_UI_H

#include <stdint.h>
#include "copilot_voice.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize voice-UI integration
 *
 * Registers voice state callback and prepares envelope tracking.
 * Call after copilot_voice_init().
 */
void copilot_voice_ui_init(void);

/**
 * @brief Get current mouth opening level
 *
 * Returns audio envelope scaled for mouth animation.
 * Automatically decays when no audio is playing.
 *
 * @return Mouth opening 0-255 (Q8 fixed-point for 0.0-1.0)
 */
uint8_t copilot_voice_ui_get_mouth_open(void);

/**
 * @brief Get current voice state for UI feedback
 *
 * @return Current voice module state
 */
copilot_voice_state_t copilot_voice_ui_get_state(void);

#ifdef __cplusplus
}
#endif

#endif // COPILOT_VOICE_UI_H
