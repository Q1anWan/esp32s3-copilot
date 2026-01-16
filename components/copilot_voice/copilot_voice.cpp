/**
 * @file copilot_voice.cpp
 * @brief Copilot Voice Module - WebRTC bidirectional audio
 *
 * Phase 1: Audio infrastructure with loopback test
 * Phase 2+: ByteRTC integration for full WebRTC support
 *
 * Uses unified audio output manager (copilot_audio_out) for speaker output
 * to avoid hardware conflicts with notification tones.
 */

#include "copilot_voice.h"
#include "copilot_audio_out.h"
#include "copilot_ws_client.h"

#include <string.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_codec_dev.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/idf_additions.h"
#include "bsp/esp-bsp.h"
#include "sdkconfig.h"

static const char *TAG = "copilot_voice";

// Conditional logging (uses copilot's Kconfig)
#if CONFIG_COPILOT_LOG_VOICE
#define LOGI_VOICE(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#else
#define LOGI_VOICE(fmt, ...) do {} while(0)
#endif

// ============================================================================
// Configuration
// ============================================================================

#if CONFIG_COPILOT_VOICE_ENABLE

#ifndef CONFIG_COPILOT_VOICE_SAMPLE_RATE
#define CONFIG_COPILOT_VOICE_SAMPLE_RATE 16000
#endif

#ifndef CONFIG_COPILOT_VOICE_TASK_STACK
#define CONFIG_COPILOT_VOICE_TASK_STACK 4096
#endif

#ifndef CONFIG_COPILOT_VOICE_TASK_PRIORITY
#define CONFIG_COPILOT_VOICE_TASK_PRIORITY 5
#endif

#ifndef CONFIG_COPILOT_VOICE_TASK_CORE
#define CONFIG_COPILOT_VOICE_TASK_CORE 0
#endif

#ifndef CONFIG_COPILOT_VOICE_MIC_GAIN
#define CONFIG_COPILOT_VOICE_MIC_GAIN 24
#endif

#ifndef CONFIG_COPILOT_VOICE_SPEAKER_VOLUME
#define CONFIG_COPILOT_VOICE_SPEAKER_VOLUME 80
#endif

// Audio buffer configuration
// ES7210 microphone ADC outputs 2 channels (stereo), we convert to mono for processing
#define MIC_AUDIO_CHANNELS          2       // ES7210 outputs stereo
#define VOICE_AUDIO_CHANNELS        1       // Mono for voice processing
#define VOICE_AUDIO_BITS            16      // 16-bit samples
#define VOICE_AUDIO_FRAME_MS        20      // 20ms frames (standard for WebRTC)
#define VOICE_AUDIO_FRAME_SAMPLES   ((CONFIG_COPILOT_VOICE_SAMPLE_RATE * VOICE_AUDIO_FRAME_MS) / 1000)
#define MIC_AUDIO_FRAME_BYTES       (VOICE_AUDIO_FRAME_SAMPLES * MIC_AUDIO_CHANNELS * (VOICE_AUDIO_BITS / 8))
#define VOICE_AUDIO_FRAME_BYTES     (VOICE_AUDIO_FRAME_SAMPLES * VOICE_AUDIO_CHANNELS * (VOICE_AUDIO_BITS / 8))

// ============================================================================
// State
// ============================================================================

static struct {
    // Microphone codec (voice owns this directly)
    esp_codec_dev_handle_t mic_dev;
    esp_codec_dev_sample_info_t mic_sample_info;

    // Pre-allocated DMA buffers (allocated at init to avoid fragmentation)
    int16_t *mic_dma_buffer;    // DMA-capable buffer for mic read (stereo)
    int16_t *mono_buffer;       // Buffer for mono conversion (not DMA)

    // Tasks
    TaskHandle_t loopback_task;
    TaskHandle_t streaming_task;
    TaskHandle_t voice_rx_task;
    TaskHandle_t voice_tx_task;

    // State
    volatile copilot_voice_state_t state;
    volatile bool loopback_running;
    volatile bool session_active;
    volatile bool streaming_running;

    // Callback
    copilot_voice_event_cb_t event_cb;
    void *event_cb_user_data;

    // Settings
    int mic_gain_db;
    int speaker_volume;

    // WebSocket client initialized flag
    bool ws_client_inited;
} s_voice = {};

// ============================================================================
// Internal helpers
// ============================================================================

static int normalize_core(int core) {
    if (core < 0 || core >= (int)configNUM_CORES) {
        return -1;  // No affinity
    }
    return core;
}

static void notify_state_change(copilot_voice_state_t new_state) {
    if (s_voice.state != new_state) {
        s_voice.state = new_state;
        if (s_voice.event_cb) {
            s_voice.event_cb(new_state, s_voice.event_cb_user_data);
        }
    }
}

// ============================================================================
// Audio Hardware Initialization
// ============================================================================

static bool init_microphone(void) {
    if (s_voice.mic_dev) {
        return true;  // Already initialized
    }

    LOGI_VOICE("Initializing microphone (ES7210)");

    s_voice.mic_dev = bsp_audio_codec_microphone_init();
    if (!s_voice.mic_dev) {
        ESP_LOGE(TAG, "Failed to init microphone codec");
        return false;
    }

    // Configure sample format - ES7210 outputs 2 channels (stereo)
    s_voice.mic_sample_info.sample_rate = CONFIG_COPILOT_VOICE_SAMPLE_RATE;
    s_voice.mic_sample_info.channel = MIC_AUDIO_CHANNELS;  // Stereo input from ES7210
    s_voice.mic_sample_info.bits_per_sample = VOICE_AUDIO_BITS;

    esp_err_t err = esp_codec_dev_open(s_voice.mic_dev, &s_voice.mic_sample_info);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open microphone: %s", esp_err_to_name(err));
        return false;
    }

    // Set microphone gain
    esp_codec_dev_set_in_gain(s_voice.mic_dev, (float)s_voice.mic_gain_db);
    LOGI_VOICE("Microphone initialized: %d Hz, %d ch, %d bit, gain=%d dB",
             s_voice.mic_sample_info.sample_rate,
             s_voice.mic_sample_info.channel,
             s_voice.mic_sample_info.bits_per_sample,
             s_voice.mic_gain_db);

    return true;
}

static bool init_audio_output(void) {
    // Use unified audio output manager (shared with copilot_audio)
    if (copilot_audio_out_is_ready()) {
        LOGI_VOICE("Audio output manager already initialized");
        return true;
    }

    LOGI_VOICE("Initializing audio output manager");

    copilot_audio_out_config_t config = {};
    config.sample_rate = CONFIG_COPILOT_VOICE_SAMPLE_RATE;
    config.speaker_volume = s_voice.speaker_volume;

    if (!copilot_audio_out_init(&config)) {
        ESP_LOGE(TAG, "Failed to init audio output manager");
        return false;
    }

    LOGI_VOICE("Audio output manager ready (rate=%d, volume=%d)",
             config.sample_rate, config.speaker_volume);
    return true;
}

static void deinit_audio(void) {
    if (s_voice.mic_dev) {
        esp_codec_dev_close(s_voice.mic_dev);
        s_voice.mic_dev = nullptr;
    }
    // Note: Don't deinit audio_out as it's shared with copilot_audio
}

// ============================================================================
// Loopback Test Task
// ============================================================================

#if CONFIG_COPILOT_VOICE_LOOPBACK_TEST

static void loopback_task_func(void *arg) {
    (void)arg;

    // Acquire audio output with VOICE priority (highest)
    if (!copilot_audio_out_acquire(AUDIO_SRC_VOICE)) {
        ESP_LOGE(TAG, "Failed to acquire audio output for loopback");
        s_voice.loopback_running = false;
        vTaskDelete(NULL);
        return;
    }

    // Set volume for voice
    copilot_audio_out_set_volume(s_voice.speaker_volume);

    // Use pre-allocated DMA buffers (allocated in init to avoid fragmentation)
    int16_t *mic_stereo_buffer = s_voice.mic_dma_buffer;
    int16_t *mono_buffer = s_voice.mono_buffer;

    if (!mic_stereo_buffer || !mono_buffer) {
        ESP_LOGE(TAG, "Pre-allocated buffers not available (mic=%p, mono=%p)",
                 mic_stereo_buffer, mono_buffer);
        copilot_audio_out_release(AUDIO_SRC_VOICE);
        s_voice.loopback_running = false;
        vTaskDelete(NULL);
        return;
    }

    LOGI_VOICE("Loopback test started: %d Hz, frame=%d samples (%d ms)",
             CONFIG_COPILOT_VOICE_SAMPLE_RATE,
             VOICE_AUDIO_FRAME_SAMPLES,
             VOICE_AUDIO_FRAME_MS);
    LOGI_VOICE("  Mic buffer: %d bytes (stereo), Mono buffer: %d bytes",
             MIC_AUDIO_FRAME_BYTES, VOICE_AUDIO_FRAME_BYTES);

    // Wait for codec to stabilize before reading
    LOGI_VOICE("Waiting for microphone to stabilize...");
    vTaskDelay(pdMS_TO_TICKS(200));

    uint32_t frame_count = 0;
    uint32_t last_log_time = 0;
    uint32_t consecutive_failures = 0;

    while (s_voice.loopback_running) {
        // Read from microphone (stereo from ES7210, DMA transfer)
        // Note: esp_codec_dev_read returns 0 on success, negative on error
        int ret = esp_codec_dev_read(s_voice.mic_dev, mic_stereo_buffer, MIC_AUDIO_FRAME_BYTES);
        if (ret < 0) {
            consecutive_failures++;
            // Only log every 50 failures to reduce spam
            if (consecutive_failures <= 3 || consecutive_failures % 50 == 0) {
                ESP_LOGW(TAG, "Mic read failed: %d (count=%lu)", ret, (unsigned long)consecutive_failures);
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        consecutive_failures = 0;

        // Convert stereo to mono (average L+R channels)
        // On success, all requested bytes were read
        int stereo_samples = MIC_AUDIO_FRAME_BYTES / sizeof(int16_t);  // Total samples (L+R interleaved)
        int mono_samples = stereo_samples / 2;  // Number of mono samples

        for (int i = 0; i < mono_samples; i++) {
            // Average left and right channels
            int32_t left = mic_stereo_buffer[i * 2];
            int32_t right = mic_stereo_buffer[i * 2 + 1];
            mono_buffer[i] = (int16_t)((left + right) / 2);
        }

        // Write mono to audio output (audio_out converts to stereo for speaker)
        int written = copilot_audio_out_write(AUDIO_SRC_VOICE, mono_buffer, mono_samples, 50);
        if (written < mono_samples) {
            // Buffer full - this shouldn't happen in loopback mode
            ESP_LOGW(TAG, "Audio buffer full, wrote %d/%d samples", written, mono_samples);
        }

        frame_count++;

        // Log stats every 5 seconds
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        if (now - last_log_time >= 5000) {
            last_log_time = now;
            int used, free_space;
            copilot_audio_out_get_buffer_status(&used, &free_space);
            LOGI_VOICE("Loopback: %lu frames, buffer=%d/%d bytes",
                     (unsigned long)frame_count, used, used + free_space);
        }

        // Yield to allow other tasks to run (prevents watchdog issues)
        taskYIELD();
    }

    // Release audio output (don't free pre-allocated buffers - owned by voice module)
    copilot_audio_out_release(AUDIO_SRC_VOICE);

    LOGI_VOICE("Loopback test stopped (total frames: %lu)", (unsigned long)frame_count);
    vTaskDelete(NULL);
}

#endif // CONFIG_COPILOT_VOICE_LOOPBACK_TEST

// ============================================================================
// WebSocket Streaming Session
// ============================================================================

#if !CONFIG_COPILOT_VOICE_MODE_LOOPBACK

// Track TTS audio reception for state management
static volatile uint32_t s_last_tts_audio_ms = 0;
static volatile uint32_t s_tts_frames_received = 0;

// Callback for receiving audio from WebSocket server
static void ws_audio_callback(const int16_t *pcm_data, size_t samples, void *user_data) {
    (void)user_data;

    // Write TTS audio to speaker
    if (samples > 0) {
        // Track TTS audio reception
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        s_last_tts_audio_ms = now_ms;
        s_tts_frames_received++;

        // Log first TTS audio reception
        static bool first_tts_logged = false;
        if (!first_tts_logged) {
            ESP_LOGI(TAG, "TTS audio received: %d samples (first frame)", (int)samples);
            first_tts_logged = true;
        }

        // Notify state change to SPEAKING when TTS audio starts
        // This enables mouth animation in the UI
        if (s_voice.state != VOICE_STATE_SPEAKING) {
            notify_state_change(VOICE_STATE_SPEAKING);
            LOGI_VOICE("TTS playback started -> SPEAKING");
        }

        // The audio_out module handles mono-to-stereo conversion
        copilot_audio_out_write(AUDIO_SRC_VOICE, pcm_data, (int)samples, 0);
    }
}

// Callback for WebSocket state changes
static void ws_state_callback(copilot_ws_client_state_t state, void *user_data) {
    (void)user_data;

    switch (state) {
        case WS_CLIENT_STATE_CONNECTED:
            LOGI_VOICE("WebSocket connected");
            break;
        case WS_CLIENT_STATE_STREAMING:
            LOGI_VOICE("Streaming started");
            notify_state_change(VOICE_STATE_LISTENING);
            break;
        case WS_CLIENT_STATE_DISCONNECTING:
        case WS_CLIENT_STATE_IDLE:
            LOGI_VOICE("WebSocket disconnected");
            if (s_voice.session_active) {
                notify_state_change(VOICE_STATE_READY);
            }
            break;
        case WS_CLIENT_STATE_ERROR:
            ESP_LOGE(TAG, "WebSocket error");
            notify_state_change(VOICE_STATE_ERROR);
            break;
        default:
            break;
    }
}

// Streaming task: reads mic and sends to WebSocket
static void streaming_task_func(void *arg) {
    (void)arg;
    LOGI_VOICE("Streaming task started");

    // Use pre-allocated DMA buffers (allocated in init to avoid fragmentation)
    int16_t *mic_stereo_buffer = s_voice.mic_dma_buffer;
    int16_t *mono_buffer = s_voice.mono_buffer;

    if (!mic_stereo_buffer || !mono_buffer) {
        ESP_LOGE(TAG, "Pre-allocated buffers not available (mic=%p, mono=%p)",
                 mic_stereo_buffer, mono_buffer);
        s_voice.streaming_running = false;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Using pre-allocated buffers: mic=%p, mono=%p",
             mic_stereo_buffer, mono_buffer);

    // Acquire audio output for receiving TTS
    if (!copilot_audio_out_acquire(AUDIO_SRC_VOICE)) {
        ESP_LOGE(TAG, "Failed to acquire audio output");
        s_voice.streaming_running = false;
        vTaskDelete(NULL);
        return;
    }

    uint32_t frame_count = 0;
    uint32_t last_log_time = 0;
    uint32_t wait_count = 0;
    uint32_t read_fail_count = 0;
    uint32_t send_fail_count = 0;
    const int64_t frame_interval_us = (int64_t)VOICE_AUDIO_FRAME_MS * 1000;
    int64_t last_frame_us = esp_timer_get_time();

    ESP_LOGI(TAG, "Streaming loop starting, mic_dev=%p", s_voice.mic_dev);

    while (s_voice.streaming_running) {
        // Wait for WebSocket to be streaming
        if (!copilot_ws_client_is_streaming()) {
            wait_count++;
            if (wait_count % 100 == 1) {  // Log every 5 seconds (100 * 50ms)
                ESP_LOGI(TAG, "Waiting for WebSocket streaming... (state=%d, attempts=%lu)",
                         (int)copilot_ws_client_get_state(),
                         (unsigned long)copilot_ws_client_get_connect_attempts());
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            last_frame_us = esp_timer_get_time();  // Reset pacing when not streaming
            continue;
        }

        // Log when streaming becomes active (first time)
        static bool streaming_active_logged = false;
        if (!streaming_active_logged) {
            ESP_LOGI(TAG, "WebSocket now streaming, starting mic capture loop");
            streaming_active_logged = true;
            last_frame_us = esp_timer_get_time();
        }
        wait_count = 0;

        // Debug: log before mic read (to detect if read is blocking)
        static bool first_read_logged = false;
        if (!first_read_logged) {
            ESP_LOGI(TAG, "About to read mic (first time), buffer=%p, size=%d",
                     mic_stereo_buffer, MIC_AUDIO_FRAME_BYTES);
            first_read_logged = true;
        }

        // Read mic data (stereo)
        // Note: esp_codec_dev_read returns 0 on success, negative on error
        int ret = esp_codec_dev_read(s_voice.mic_dev, mic_stereo_buffer, MIC_AUDIO_FRAME_BYTES);

        // Debug: log after first successful read
        static bool first_success_logged = false;
        if (ret >= 0 && !first_success_logged) {
            ESP_LOGI(TAG, "First mic read success! ret=%d", ret);
            first_success_logged = true;
        }

        if (ret < 0) {
            read_fail_count++;
            if (read_fail_count <= 5 || read_fail_count % 100 == 0) {
                ESP_LOGW(TAG, "Mic read failed: %d (count=%lu)", ret, (unsigned long)read_fail_count);
            }
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        // Convert stereo to mono (take left channel only for ES7210 ADC1)
        int mono_samples = VOICE_AUDIO_FRAME_SAMPLES;
        for (int i = 0; i < mono_samples; i++) {
            mono_buffer[i] = mic_stereo_buffer[i * 2];
        }

        // Debug: log first conversion
        static bool first_convert_logged = false;
        if (!first_convert_logged) {
            ESP_LOGI(TAG, "First stereo->mono conversion done, sending %d samples", mono_samples);
            first_convert_logged = true;
        }

        // Send to WebSocket server (for ASR)
#if CONFIG_COPILOT_VOICE_MODE_FULL_DUPLEX || CONFIG_COPILOT_VOICE_MODE_TX_ONLY
        bool sent = copilot_ws_client_send_audio(mono_buffer, mono_samples);

        // Debug: log first send result
        static bool first_send_result_logged = false;
        if (!first_send_result_logged) {
            ESP_LOGI(TAG, "First send_audio returned: %s", sent ? "true" : "false");
            first_send_result_logged = true;
        }

        if (!sent) {
            send_fail_count++;
            if (send_fail_count <= 5 || send_fail_count % 100 == 0) {
                ESP_LOGW(TAG, "Audio send failed (count=%lu)", (unsigned long)send_fail_count);
            }
        }
#endif

        frame_count++;

        // Log stats every 5 seconds
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        if (now - last_log_time >= 5000) {
            last_log_time = now;
            ESP_LOGI(TAG, "Streaming: %lu TX, %lu TTS RX, read_fail=%lu, send_fail=%lu",
                     (unsigned long)frame_count,
                     (unsigned long)s_tts_frames_received,
                     (unsigned long)read_fail_count,
                     (unsigned long)send_fail_count);
        }

        // Check if TTS playback finished (no TTS audio for 300ms)
        // Return to LISTENING state when TTS ends
        if (s_voice.state == VOICE_STATE_SPEAKING && s_last_tts_audio_ms > 0) {
            if (now - s_last_tts_audio_ms > 300) {
                notify_state_change(VOICE_STATE_LISTENING);
                LOGI_VOICE("TTS playback finished -> LISTENING");
            }
        }

        // Pace sends to real-time to avoid bursting and TCP send buffer overflows
        int64_t now_us = esp_timer_get_time();
        int64_t elapsed_us = now_us - last_frame_us;
        if (elapsed_us < frame_interval_us) {
            int64_t wait_us = frame_interval_us - elapsed_us;
            vTaskDelay(pdMS_TO_TICKS((wait_us + 999) / 1000));
        }
        last_frame_us = esp_timer_get_time();
    }

    // Cleanup (don't free pre-allocated buffers - they're owned by voice module)
    copilot_audio_out_release(AUDIO_SRC_VOICE);

    LOGI_VOICE("Streaming task stopped (total frames: %lu)", (unsigned long)frame_count);
    vTaskDelete(NULL);
}

static bool start_streaming_session(void) {
    // Build WebSocket URL from config
#ifndef CONFIG_COPILOT_VOICE_SERVER_URL
    const char *server_url = "ws://192.168.1.100:8080/audio/stream";
#else
    // Convert HTTP URL to WebSocket URL and add path
    char ws_url[256];
    const char *http_url = CONFIG_COPILOT_VOICE_SERVER_URL;

    // Replace http:// with ws://
    if (strncmp(http_url, "http://", 7) == 0) {
        snprintf(ws_url, sizeof(ws_url), "ws://%s/audio/stream", http_url + 7);
    } else if (strncmp(http_url, "https://", 8) == 0) {
        snprintf(ws_url, sizeof(ws_url), "wss://%s/audio/stream", http_url + 8);
    } else {
        snprintf(ws_url, sizeof(ws_url), "ws://%s/audio/stream", http_url);
    }
    const char *server_url = ws_url;
#endif

    // Initialize WebSocket client if not already
    if (!s_voice.ws_client_inited) {
        copilot_ws_client_config_t ws_cfg = {};
        ws_cfg.server_url = server_url;
        ws_cfg.device_id = "esp32_copilot";
        ws_cfg.sample_rate = CONFIG_COPILOT_VOICE_SAMPLE_RATE;
        ws_cfg.on_audio = ws_audio_callback;
        ws_cfg.on_state = ws_state_callback;
        ws_cfg.user_data = nullptr;

        if (!copilot_ws_client_init(&ws_cfg)) {
            ESP_LOGE(TAG, "WebSocket client init failed");
            return false;
        }
        s_voice.ws_client_inited = true;
    }

    // Connect to server
    if (!copilot_ws_client_connect()) {
        ESP_LOGE(TAG, "WebSocket connect failed");
        return false;
    }

    // Start streaming task (use PSRAM for stack since internal RAM is fragmented)
    s_voice.streaming_running = true;

    int core = normalize_core(CONFIG_COPILOT_VOICE_TASK_CORE);
    BaseType_t result;

    if (core >= 0) {
        result = xTaskCreatePinnedToCoreWithCaps(
            streaming_task_func,
            "voice_stream",
            CONFIG_COPILOT_VOICE_TASK_STACK,
            nullptr,
            CONFIG_COPILOT_VOICE_TASK_PRIORITY,
            &s_voice.streaming_task,
            core,
            MALLOC_CAP_SPIRAM);
    } else {
        result = xTaskCreateWithCaps(
            streaming_task_func,
            "voice_stream",
            CONFIG_COPILOT_VOICE_TASK_STACK,
            nullptr,
            CONFIG_COPILOT_VOICE_TASK_PRIORITY,
            &s_voice.streaming_task,
            MALLOC_CAP_SPIRAM);
    }

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create streaming task (SPIRAM free: %lu bytes)",
                 (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        copilot_ws_client_disconnect();
        s_voice.streaming_running = false;
        return false;
    }

    s_voice.session_active = true;
    LOGI_VOICE("Streaming session started");
    return true;
}

static void stop_streaming_session(void) {
    if (!s_voice.streaming_running) {
        return;
    }

    LOGI_VOICE("Stopping streaming session");

    s_voice.streaming_running = false;

    // Disconnect WebSocket
    copilot_ws_client_disconnect();

    // Wait for task to exit
    if (s_voice.streaming_task) {
        int timeout = 50;  // 500ms
        while (eTaskGetState(s_voice.streaming_task) != eDeleted && timeout > 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            timeout--;
        }
        s_voice.streaming_task = NULL;
    }

    s_voice.session_active = false;
}

#endif // !CONFIG_COPILOT_VOICE_MODE_LOOPBACK

// ============================================================================
// Public API Implementation
// ============================================================================

bool copilot_voice_init(void) {
    if (s_voice.state != VOICE_STATE_IDLE) {
        ESP_LOGW(TAG, "Voice module already initialized");
        return true;
    }

    LOGI_VOICE("Initializing voice module");
    LOGI_VOICE("  Sample rate: %d Hz", CONFIG_COPILOT_VOICE_SAMPLE_RATE);
    LOGI_VOICE("  Frame size: %d samples (%d ms, %d bytes)",
             VOICE_AUDIO_FRAME_SAMPLES, VOICE_AUDIO_FRAME_MS, VOICE_AUDIO_FRAME_BYTES);

    // Pre-allocate DMA buffers FIRST, before other allocations fragment memory
    // This is critical for the mic read which requires DMA-capable memory
    ESP_LOGI(TAG, "DMA memory before alloc: free=%u, largest=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));

    s_voice.mic_dma_buffer = (int16_t *)heap_caps_aligned_alloc(
        4, MIC_AUDIO_FRAME_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_voice.mic_dma_buffer) {
        ESP_LOGE(TAG, "Failed to allocate mic DMA buffer (%d bytes)", MIC_AUDIO_FRAME_BYTES);
        return false;
    }

    s_voice.mono_buffer = (int16_t *)heap_caps_malloc(
        VOICE_AUDIO_FRAME_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_voice.mono_buffer) {
        s_voice.mono_buffer = (int16_t *)heap_caps_malloc(
            VOICE_AUDIO_FRAME_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (!s_voice.mono_buffer) {
        ESP_LOGE(TAG, "Failed to allocate mono buffer");
        heap_caps_free(s_voice.mic_dma_buffer);
        s_voice.mic_dma_buffer = NULL;
        return false;
    }

    ESP_LOGI(TAG, "Pre-allocated DMA buffers: mic=%p (%d bytes), mono=%p (%d bytes)",
             s_voice.mic_dma_buffer, MIC_AUDIO_FRAME_BYTES,
             s_voice.mono_buffer, VOICE_AUDIO_FRAME_BYTES);

    // Initialize default settings
    s_voice.mic_gain_db = CONFIG_COPILOT_VOICE_MIC_GAIN;
    s_voice.speaker_volume = CONFIG_COPILOT_VOICE_SPEAKER_VOLUME;

    // IMPORTANT: Initialize audio output FIRST to configure I2S with stereo mode
    // The BSP defaults to mono mode which causes ES7210 mic to fail
    if (!init_audio_output()) {
        ESP_LOGE(TAG, "Audio output init failed");
        notify_state_change(VOICE_STATE_ERROR);
        return false;
    }

    // Initialize microphone (voice module owns this)
    // Must be after audio_out init since I2S needs stereo mode
    if (!init_microphone()) {
        ESP_LOGE(TAG, "Microphone init failed");
        notify_state_change(VOICE_STATE_ERROR);
        return false;
    }

    notify_state_change(VOICE_STATE_READY);
    LOGI_VOICE("Voice module initialized successfully");
    return true;
}

void copilot_voice_deinit(void) {
    LOGI_VOICE("Deinitializing voice module");

    // Stop any active tasks
    copilot_voice_stop_loopback();
    copilot_voice_stop_session();

    // Release microphone
    deinit_audio();

    // Free pre-allocated buffers
    if (s_voice.mic_dma_buffer) {
        heap_caps_free(s_voice.mic_dma_buffer);
        s_voice.mic_dma_buffer = NULL;
    }
    if (s_voice.mono_buffer) {
        heap_caps_free(s_voice.mono_buffer);
        s_voice.mono_buffer = NULL;
    }

    notify_state_change(VOICE_STATE_IDLE);
}

bool copilot_voice_start_session(void) {
    if (s_voice.state != VOICE_STATE_READY) {
        ESP_LOGW(TAG, "Cannot start session: state=%d", s_voice.state);
        return false;
    }

#if CONFIG_COPILOT_VOICE_MODE_LOOPBACK
    // In loopback mode, start_session is not applicable
    ESP_LOGW(TAG, "start_session not applicable in loopback mode");
    return false;
#else
    // Start WebSocket streaming session
    return start_streaming_session();
#endif
}

void copilot_voice_stop_session(void) {
    if (!s_voice.session_active) {
        return;
    }

#if !CONFIG_COPILOT_VOICE_MODE_LOOPBACK
    stop_streaming_session();
#endif

    s_voice.session_active = false;
    notify_state_change(VOICE_STATE_READY);
}

copilot_voice_state_t copilot_voice_get_state(void) {
    return s_voice.state;
}

bool copilot_voice_is_ready(void) {
    return s_voice.state == VOICE_STATE_READY;
}

bool copilot_voice_is_active(void) {
    return s_voice.session_active;
}

bool copilot_voice_is_loopback_running(void) {
#if CONFIG_COPILOT_VOICE_LOOPBACK_TEST
    return s_voice.loopback_running;
#else
    return false;
#endif
}

void copilot_voice_set_event_callback(copilot_voice_event_cb_t cb, void *user_data) {
    s_voice.event_cb = cb;
    s_voice.event_cb_user_data = user_data;
}

bool copilot_voice_start_loopback(void) {
#if CONFIG_COPILOT_VOICE_LOOPBACK_TEST
    if (s_voice.loopback_running) {
        ESP_LOGW(TAG, "Loopback already running");
        return true;
    }

    if (s_voice.state != VOICE_STATE_READY) {
        ESP_LOGE(TAG, "Voice module not ready for loopback");
        return false;
    }

    s_voice.loopback_running = true;
    notify_state_change(VOICE_STATE_SPEAKING);  // Loopback outputs audio -> SPEAKING state

    int core = normalize_core(CONFIG_COPILOT_VOICE_TASK_CORE);
    BaseType_t result;

    if (core >= 0) {
        result = xTaskCreatePinnedToCore(
            loopback_task_func,
            "voice_loopback",
            CONFIG_COPILOT_VOICE_TASK_STACK,
            nullptr,
            CONFIG_COPILOT_VOICE_TASK_PRIORITY,
            &s_voice.loopback_task,
            core);
    } else {
        result = xTaskCreate(
            loopback_task_func,
            "voice_loopback",
            CONFIG_COPILOT_VOICE_TASK_STACK,
            nullptr,
            CONFIG_COPILOT_VOICE_TASK_PRIORITY,
            &s_voice.loopback_task);
    }

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create loopback task");
        s_voice.loopback_running = false;
        return false;
    }

    LOGI_VOICE("Loopback started (core=%d, priority=%d)",
             core, CONFIG_COPILOT_VOICE_TASK_PRIORITY);
    return true;
#else
    ESP_LOGW(TAG, "Loopback test not enabled in config");
    return false;
#endif
}

void copilot_voice_stop_loopback(void) {
#if CONFIG_COPILOT_VOICE_LOOPBACK_TEST
    if (!s_voice.loopback_running) {
        return;
    }

    LOGI_VOICE("Stopping loopback...");
    s_voice.loopback_running = false;

    // Wait for task to exit
    if (s_voice.loopback_task) {
        // Give task time to exit gracefully
        vTaskDelay(pdMS_TO_TICKS(100));
        s_voice.loopback_task = nullptr;
    }

    notify_state_change(VOICE_STATE_READY);  // Back to ready state
#endif
}

void copilot_voice_set_mic_gain(int gain_db) {
    if (gain_db < 0) gain_db = 0;
    if (gain_db > 36) gain_db = 36;

    s_voice.mic_gain_db = gain_db;

    if (s_voice.mic_dev) {
        esp_codec_dev_set_in_gain(s_voice.mic_dev, (float)gain_db);
        LOGI_VOICE("Mic gain set to %d dB", gain_db);
    }
}

void copilot_voice_set_speaker_volume(int volume_percent) {
    if (volume_percent < 0) volume_percent = 0;
    if (volume_percent > 100) volume_percent = 100;

    s_voice.speaker_volume = volume_percent;

    // Use audio output manager for volume control
    ESP_LOGI(TAG, "Setting speaker volume to %d%%", volume_percent);
    copilot_audio_out_set_volume(volume_percent);
}

#else // !CONFIG_COPILOT_VOICE_ENABLE

// Stub implementations when voice module is disabled

bool copilot_voice_init(void) {
    ESP_LOGW(TAG, "Voice module disabled in config");
    return false;
}

void copilot_voice_deinit(void) {}

bool copilot_voice_start_session(void) { return false; }
void copilot_voice_stop_session(void) {}

copilot_voice_state_t copilot_voice_get_state(void) { return VOICE_STATE_IDLE; }
bool copilot_voice_is_ready(void) { return false; }
bool copilot_voice_is_active(void) { return false; }
bool copilot_voice_is_loopback_running(void) { return false; }

void copilot_voice_set_event_callback(copilot_voice_event_cb_t cb, void *user_data) {
    (void)cb;
    (void)user_data;
}

bool copilot_voice_start_loopback(void) { return false; }
void copilot_voice_stop_loopback(void) {}

void copilot_voice_set_mic_gain(int gain_db) { (void)gain_db; }
void copilot_voice_set_speaker_volume(int volume_percent) { (void)volume_percent; }

#endif // CONFIG_COPILOT_VOICE_ENABLE
