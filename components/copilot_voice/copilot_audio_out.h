/**
 * @file copilot_audio_out.h
 * @brief Unified Audio Output Manager
 *
 * Centralized audio output with:
 * - Single ES8311 codec handle (no hardware conflicts)
 * - Priority-based audio source switching
 * - DMA-optimized ring buffer for streaming
 * - Thread-safe API with mutex protection
 *
 * Audio Sources (by priority, highest first):
 * 1. VOICE - TTS/WebRTC audio (16kHz mono -> stereo)
 * 2. TONE  - Notification sounds (generated tones)
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Audio source priority levels
 */
typedef enum {
    AUDIO_SRC_NONE = 0,     ///< No active source
    AUDIO_SRC_TONE,         ///< Low priority: notification tones
    AUDIO_SRC_VOICE,        ///< High priority: TTS/WebRTC voice
} copilot_audio_src_t;

/**
 * @brief Audio output configuration
 */
typedef struct {
    int sample_rate;        ///< Sample rate in Hz (default: 16000)
    int speaker_volume;     ///< Speaker volume 0-100 (default: 80)
} copilot_audio_out_config_t;

/**
 * @brief Initialize audio output manager
 *
 * Initializes ES8311 codec and creates output task.
 * Must be called before any other audio_out functions.
 *
 * @param config Configuration (NULL for defaults)
 * @return true on success
 */
bool copilot_audio_out_init(const copilot_audio_out_config_t *config);

/**
 * @brief Deinitialize audio output manager
 */
void copilot_audio_out_deinit(void);

/**
 * @brief Check if audio output is initialized
 * @return true if ready
 */
bool copilot_audio_out_is_ready(void);

/**
 * @brief Acquire audio output for a source
 *
 * Higher priority sources can preempt lower priority ones.
 * Lower priority sources will be blocked until higher priority releases.
 *
 * @param src Source requesting access
 * @return true if acquired (or preempted lower priority)
 */
bool copilot_audio_out_acquire(copilot_audio_src_t src);

/**
 * @brief Release audio output from a source
 *
 * @param src Source releasing access
 */
void copilot_audio_out_release(copilot_audio_src_t src);

/**
 * @brief Get current active source
 * @return Current source holding the output
 */
copilot_audio_src_t copilot_audio_out_get_active(void);

/**
 * @brief Write audio samples to output buffer
 *
 * Writes mono 16-bit PCM samples. Automatically converts to stereo.
 * Uses DMA-optimized ring buffer for low-latency streaming.
 *
 * @param src Source writing data (must have acquired first)
 * @param samples Pointer to mono 16-bit PCM samples
 * @param num_samples Number of samples to write
 * @param timeout_ms Timeout in milliseconds (0 = no wait)
 * @return Number of samples actually written
 */
int copilot_audio_out_write(copilot_audio_src_t src,
                            const int16_t *samples,
                            int num_samples,
                            int timeout_ms);

/**
 * @brief Write stereo audio samples directly
 *
 * For sources that already have stereo data.
 *
 * @param src Source writing data
 * @param samples Pointer to interleaved stereo 16-bit PCM samples
 * @param num_samples Number of stereo sample pairs
 * @param timeout_ms Timeout in milliseconds
 * @return Number of sample pairs written
 */
int copilot_audio_out_write_stereo(copilot_audio_src_t src,
                                   const int16_t *samples,
                                   int num_samples,
                                   int timeout_ms);

/**
 * @brief Flush audio buffer and wait for playback to complete
 *
 * @param src Source to flush
 * @param timeout_ms Maximum wait time
 * @return true if flushed successfully
 */
bool copilot_audio_out_flush(copilot_audio_src_t src, int timeout_ms);

/**
 * @brief Set speaker volume
 *
 * @param volume Volume level 0-100
 */
void copilot_audio_out_set_volume(int volume);

/**
 * @brief Get current speaker volume
 * @return Volume level 0-100
 */
int copilot_audio_out_get_volume(void);

/**
 * @brief Get ring buffer fill level
 *
 * Useful for flow control and debugging.
 *
 * @param out_used Bytes currently in buffer (can be NULL)
 * @param out_free Bytes available for writing (can be NULL)
 */
void copilot_audio_out_get_buffer_status(int *out_used, int *out_free);

/**
 * @brief Play a short tone overlay on the current output
 *
 * Tone is mixed into the output stream without preempting the active source.
 *
 * @param freq_hz Tone frequency in Hz
 * @param duration_ms Duration in milliseconds
 * @param volume Tone volume (0-100)
 * @return true if scheduled, false on error
 */
bool copilot_audio_out_play_tone(uint16_t freq_hz, uint16_t duration_ms, uint8_t volume);

/**
 * @brief Check if a tone overlay is active
 *
 * @return true if a tone is currently playing
 */
bool copilot_audio_out_is_tone_active(void);

/**
 * @brief Get current audio envelope level for voice source
 *
 * Returns the peak amplitude envelope of recent voice audio.
 * Used for mouth animation during TTS playback.
 * Envelope decays automatically when no audio is written.
 *
 * @return Envelope level 0-255 (Q8 fixed-point for 0.0-1.0)
 */
uint8_t copilot_audio_out_get_envelope(void);

#ifdef __cplusplus
}
#endif
