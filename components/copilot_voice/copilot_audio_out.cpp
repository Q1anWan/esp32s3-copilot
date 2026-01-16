/**
 * @file copilot_audio_out.cpp
 * @brief Unified Audio Output Manager Implementation
 *
 * Features:
 * - DMA-aligned ring buffer for zero-copy streaming
 * - Priority-based source arbitration
 * - Automatic mono-to-stereo conversion
 * - Lock-free fast path for single writer
 */

#include "copilot_audio_out.h"

#include <string.h>
#include <stdint.h>
#include <math.h>

#include "esp_log.h"
#include "esp_codec_dev.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/idf_additions.h"
#include "driver/i2s_std.h"
#include "bsp/esp-bsp.h"
#include "sdkconfig.h"

// I2S GPIO configuration (must match BSP)
#define AUDIO_I2S_GPIO_CFG       \
    {                            \
        .mclk = BSP_I2S_MCLK,    \
        .bclk = BSP_I2S_SCLK,    \
        .ws = BSP_I2S_LCLK,      \
        .dout = BSP_I2S_DOUT,    \
        .din = BSP_I2S_DSIN,     \
        .invert_flags = {        \
            .mclk_inv = false,   \
            .bclk_inv = false,   \
            .ws_inv = false,     \
        },                       \
    }

static const char *TAG = "audio_out";

// Conditional logging
#if CONFIG_COPILOT_LOG_VOICE
#define LOGI_OUT(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#else
#define LOGI_OUT(fmt, ...) do {} while(0)
#endif

// ============================================================================
// Configuration
// ============================================================================

// Default sample rate (WebRTC standard)
#define DEFAULT_SAMPLE_RATE     16000

// Ring buffer size: ~200ms of stereo audio at 16kHz
// 16000 samples/sec * 0.2 sec * 2 channels * 2 bytes = 12800 bytes
// Round up to 16KB for DMA alignment
#define RING_BUFFER_SIZE        (16 * 1024)

// DMA transfer chunk size (must be power of 2 for efficiency)
#define DMA_CHUNK_SAMPLES       256
#define DMA_CHUNK_BYTES         (DMA_CHUNK_SAMPLES * 2 * sizeof(int16_t))  // stereo

#ifndef CONFIG_COPILOT_AUDIO_CORE
#define CONFIG_COPILOT_AUDIO_CORE 0
#endif

// Output task configuration
#define OUTPUT_TASK_STACK       4096
#define OUTPUT_TASK_PRIORITY    6   // Higher than voice tasks
#define OUTPUT_TASK_CORE        CONFIG_COPILOT_AUDIO_CORE   // Keep audio pipeline on the configured audio core

static int normalize_core(int core) {
    if (core < 0 || core >= (int)configNUM_CORES) {
        return -1;
    }
    return core;
}

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// Ring Buffer (DMA-optimized, lock-free single producer)
// ============================================================================

typedef struct {
    uint8_t *buffer;            // DMA-aligned buffer
    volatile int write_pos;     // Write position (producer)
    volatile int read_pos;      // Read position (consumer)
    int size;                   // Buffer size in bytes
    SemaphoreHandle_t write_sem; // Semaphore for write blocking
    SemaphoreHandle_t data_sem;  // Semaphore for data available
} ring_buffer_t;

static bool ring_init(ring_buffer_t *rb, int size) {
    // Ring buffer is a software buffer, doesn't need DMA capability.
    // Prefer PSRAM to free internal RAM for WiFi and LCD DMA buffers.
    rb->buffer = (uint8_t *)heap_caps_aligned_alloc(
        4, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!rb->buffer) {
        rb->buffer = (uint8_t *)heap_caps_aligned_alloc(
            4, size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (!rb->buffer) {
        ESP_LOGE(TAG, "Failed to allocate ring buffer (%d bytes)", size);
        return false;
    }
    memset(rb->buffer, 0, size);

    rb->write_pos = 0;
    rb->read_pos = 0;
    rb->size = size;
    rb->write_sem = xSemaphoreCreateBinary();
    rb->data_sem = xSemaphoreCreateBinary();

    if (!rb->write_sem || !rb->data_sem) {
        heap_caps_free(rb->buffer);
        if (rb->write_sem) vSemaphoreDelete(rb->write_sem);
        if (rb->data_sem) vSemaphoreDelete(rb->data_sem);
        return false;
    }

    // Initially writable
    xSemaphoreGive(rb->write_sem);
    return true;
}

static void ring_deinit(ring_buffer_t *rb) {
    if (rb->buffer) {
        heap_caps_free(rb->buffer);
        rb->buffer = NULL;
    }
    if (rb->write_sem) {
        vSemaphoreDelete(rb->write_sem);
        rb->write_sem = NULL;
    }
    if (rb->data_sem) {
        vSemaphoreDelete(rb->data_sem);
        rb->data_sem = NULL;
    }
}

static inline int ring_used(const ring_buffer_t *rb) {
    int w = rb->write_pos;
    int r = rb->read_pos;
    if (w >= r) {
        return w - r;
    }
    return rb->size - r + w;
}

static inline int ring_free(const ring_buffer_t *rb) {
    return rb->size - ring_used(rb) - 1;  // -1 to distinguish full from empty
}

// Write to ring buffer (blocking with timeout)
static int ring_write(ring_buffer_t *rb, const uint8_t *data, int len, int timeout_ms) {
    int written = 0;
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(timeout_ms);

    while (written < len) {
        int free_space = ring_free(rb);
        if (free_space <= 0) {
            if (timeout_ms <= 0) {
                break;  // Non-blocking mode: no space available
            }
            // Wait for space
            TickType_t elapsed = xTaskGetTickCount() - start;
            if (elapsed >= timeout) {
                break;  // Timeout
            }
            TickType_t wait = timeout - elapsed;
            xSemaphoreTake(rb->write_sem, wait);
            continue;
        }

        int to_write = len - written;
        if (to_write > free_space) {
            to_write = free_space;
        }

        int w = rb->write_pos;

        // Handle wrap-around
        int to_end = rb->size - w;
        if (to_write <= to_end) {
            memcpy(rb->buffer + w, data + written, to_write);
        } else {
            memcpy(rb->buffer + w, data + written, to_end);
            memcpy(rb->buffer, data + written + to_end, to_write - to_end);
        }

        // Update write position (atomic for single producer)
        rb->write_pos = (w + to_write) % rb->size;
        written += to_write;

        // Signal data available
        xSemaphoreGive(rb->data_sem);
    }

    return written;
}

// Read from ring buffer (blocking with timeout)
static int ring_read(ring_buffer_t *rb, uint8_t *data, int len, int timeout_ms) {
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(timeout_ms);

    while (ring_used(rb) < len) {
        TickType_t elapsed = xTaskGetTickCount() - start;
        if (timeout_ms > 0 && elapsed >= timeout) {
            break;
        }
        TickType_t wait = (timeout_ms > 0) ? (timeout - elapsed) : pdMS_TO_TICKS(10);
        xSemaphoreTake(rb->data_sem, wait);
    }

    int available = ring_used(rb);
    int to_read = (len < available) ? len : available;
    if (to_read <= 0) {
        return 0;
    }

    int r = rb->read_pos;

    // Handle wrap-around
    int to_end = rb->size - r;
    if (to_read <= to_end) {
        memcpy(data, rb->buffer + r, to_read);
    } else {
        memcpy(data, rb->buffer + r, to_end);
        memcpy(data + to_end, rb->buffer, to_read - to_end);
    }

    // Update read position
    rb->read_pos = (r + to_read) % rb->size;

    // Signal space available
    xSemaphoreGive(rb->write_sem);

    return to_read;
}

static void ring_clear(ring_buffer_t *rb) {
    rb->write_pos = 0;
    rb->read_pos = 0;
    xSemaphoreGive(rb->write_sem);
}

// ============================================================================
// Audio Output State
// ============================================================================

static struct {
    // Hardware
    esp_codec_dev_handle_t codec_dev;
    esp_codec_dev_sample_info_t sample_info;

    // Ring buffer
    ring_buffer_t ring;

    // Source management
    volatile copilot_audio_src_t active_src;
    SemaphoreHandle_t src_mutex;

    // Output task
    TaskHandle_t output_task;
    volatile bool running;

    // Configuration
    int sample_rate;
    int volume;

    // DMA output buffer (aligned)
    int16_t *dma_buffer;

    // Tone overlay state
    struct {
        bool active;
        uint32_t samples_left;
        uint32_t total_samples;
        uint32_t sample_index;
        float phase;
        float phase_inc;
        int amplitude;
        int fade_samples;
    } tone;
} s_out = {};

static portMUX_TYPE s_tone_lock = portMUX_INITIALIZER_UNLOCKED;

static inline int16_t copilot_clip_i16(int32_t value) {
    if (value > 32767) {
        return 32767;
    }
    if (value < -32768) {
        return -32768;
    }
    return (int16_t)value;
}

static bool copilot_mix_tone(int16_t *buffer, int frames) {
    if (frames <= 0) {
        return false;
    }

    decltype(s_out.tone) tone = {};
    portENTER_CRITICAL(&s_tone_lock);
    tone = s_out.tone;
    portEXIT_CRITICAL(&s_tone_lock);

    if (!tone.active || tone.samples_left == 0 || tone.total_samples == 0) {
        return false;
    }

    int to_mix = frames;
    if ((uint32_t)to_mix > tone.samples_left) {
        to_mix = (int)tone.samples_left;
    }

    for (int i = 0; i < to_mix; ++i) {
        float env = 1.0f;
        if (tone.fade_samples > 0) {
            if (tone.sample_index < (uint32_t)tone.fade_samples) {
                env = (float)tone.sample_index / (float)tone.fade_samples;
            } else if (tone.sample_index > tone.total_samples - (uint32_t)tone.fade_samples) {
                env = (float)(tone.total_samples - tone.sample_index) / (float)tone.fade_samples;
            }
        }

        float sample_f = sinf(tone.phase) * (float)tone.amplitude * env;
        int32_t sample = (int32_t)sample_f;

        int idx = i * 2;
        buffer[idx] = copilot_clip_i16((int32_t)buffer[idx] + sample);
        buffer[idx + 1] = copilot_clip_i16((int32_t)buffer[idx + 1] + sample);

        tone.phase += tone.phase_inc;
        if (tone.phase > 2.0f * (float)M_PI) {
            tone.phase -= 2.0f * (float)M_PI;
        }

        tone.sample_index++;
    }

    tone.samples_left -= (uint32_t)to_mix;
    if (tone.samples_left == 0) {
        tone.active = false;
    }

    portENTER_CRITICAL(&s_tone_lock);
    s_out.tone = tone;
    portEXIT_CRITICAL(&s_tone_lock);

    return true;
}

// ============================================================================
// Output Task
// ============================================================================

static void output_task_func(void *arg) {
    (void)arg;

    LOGI_OUT("Output task started (rate=%d, chunk=%d samples)",
             s_out.sample_rate, DMA_CHUNK_SAMPLES);

    while (s_out.running) {
        // Read from ring buffer into DMA buffer
        bool tone_active = copilot_audio_out_is_tone_active();
        bool source_active = (s_out.active_src != AUDIO_SRC_NONE);
        bool need_warmup = tone_active || source_active;

        int read_timeout = need_warmup ? 5 : 50;
        int bytes_read = ring_read(&s_out.ring, (uint8_t *)s_out.dma_buffer,
                                   DMA_CHUNK_BYTES, read_timeout);

        if (bytes_read == 0 && !need_warmup) {
            // No data and no active source - yield to prevent watchdog trigger
            // This is critical: without a delay, this task starves IDLE task on CPU1
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        int bytes_to_write = bytes_read;

        // When a source is active or tone is playing, keep writing silence
        // to keep the codec DAC warm and prevent audio popping
        if (need_warmup) {
            if (bytes_read == 0) {
                memset(s_out.dma_buffer, 0, DMA_CHUNK_BYTES);
            } else if (bytes_read < DMA_CHUNK_BYTES) {
                memset(((uint8_t *)s_out.dma_buffer) + bytes_read, 0, DMA_CHUNK_BYTES - bytes_read);
            }
            bytes_to_write = DMA_CHUNK_BYTES;

            // Mix tone if active
            if (tone_active) {
                copilot_mix_tone(s_out.dma_buffer, DMA_CHUNK_SAMPLES);
            }
        }

        if (bytes_to_write > 0) {
            // Write to codec (DMA transfer)
            int ret = esp_codec_dev_write(s_out.codec_dev, s_out.dma_buffer, bytes_to_write);
            if (ret < 0) {
                ESP_LOGW(TAG, "Codec write failed: %d", ret);
            }
        }
    }

    LOGI_OUT("Output task stopped");
    vTaskDelete(NULL);
}

// ============================================================================
// Public API Implementation
// ============================================================================

bool copilot_audio_out_init(const copilot_audio_out_config_t *config) {
    if (s_out.codec_dev) {
        LOGI_OUT("Already initialized");
        return true;
    }

    // Apply configuration
    s_out.sample_rate = config ? config->sample_rate : DEFAULT_SAMPLE_RATE;
    s_out.volume = config ? config->speaker_volume : 80;

    if (s_out.sample_rate <= 0) {
        s_out.sample_rate = DEFAULT_SAMPLE_RATE;
    }

    LOGI_OUT("Initializing audio output (rate=%d, volume=%d)",
             s_out.sample_rate, s_out.volume);

    // Initialize ring buffer
    if (!ring_init(&s_out.ring, RING_BUFFER_SIZE)) {
        return false;
    }

    // Allocate DMA buffer
    s_out.dma_buffer = (int16_t *)heap_caps_aligned_alloc(
        4, DMA_CHUNK_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_out.dma_buffer) {
        ESP_LOGE(TAG, "Failed to allocate DMA buffer");
        ring_deinit(&s_out.ring);
        return false;
    }

    // Create mutex
    s_out.src_mutex = xSemaphoreCreateMutex();
    if (!s_out.src_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        heap_caps_free(s_out.dma_buffer);
        ring_deinit(&s_out.ring);
        return false;
    }

    // Initialize I2C first (required by codecs)
    esp_err_t i2c_err = bsp_i2c_init();
    if (i2c_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2C: %s", esp_err_to_name(i2c_err));
        vSemaphoreDelete(s_out.src_mutex);
        heap_caps_free(s_out.dma_buffer);
        ring_deinit(&s_out.ring);
        return false;
    }

    // Initialize I2S with STEREO mode before codec init
    // This is critical: ES7210 microphone needs stereo I2S mode
    // Default BSP config uses mono which causes mic read to fail
    i2s_std_config_t i2s_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)s_out.sample_rate),
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = AUDIO_I2S_GPIO_CFG,
    };
    esp_err_t i2s_err = bsp_audio_init(&i2s_config);
    if (i2s_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2S: %s", esp_err_to_name(i2s_err));
        vSemaphoreDelete(s_out.src_mutex);
        heap_caps_free(s_out.dma_buffer);
        ring_deinit(&s_out.ring);
        return false;
    }
    LOGI_OUT("I2S initialized with stereo mode at %d Hz", s_out.sample_rate);

    // Initialize codec (I2S already initialized above)
    s_out.codec_dev = bsp_audio_codec_speaker_init();
    if (!s_out.codec_dev) {
        ESP_LOGE(TAG, "Failed to init speaker codec");
        vSemaphoreDelete(s_out.src_mutex);
        heap_caps_free(s_out.dma_buffer);
        ring_deinit(&s_out.ring);
        return false;
    }

    // Configure codec: stereo output at configured sample rate
    s_out.sample_info.sample_rate = s_out.sample_rate;
    s_out.sample_info.channel = 2;  // Stereo output
    s_out.sample_info.bits_per_sample = 16;

    esp_err_t err = esp_codec_dev_open(s_out.codec_dev, &s_out.sample_info);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open codec: %s", esp_err_to_name(err));
        vSemaphoreDelete(s_out.src_mutex);
        heap_caps_free(s_out.dma_buffer);
        ring_deinit(&s_out.ring);
        return false;
    }

    esp_codec_dev_set_out_vol(s_out.codec_dev, s_out.volume);

    // Start output task
    s_out.running = true;
    s_out.active_src = AUDIO_SRC_NONE;

    int core = normalize_core(OUTPUT_TASK_CORE);
    BaseType_t ret;
    if (core >= 0) {
        ret = xTaskCreatePinnedToCoreWithCaps(
            output_task_func,
            "audio_out",
            OUTPUT_TASK_STACK,
            NULL,
            OUTPUT_TASK_PRIORITY,
            &s_out.output_task,
            core,
            MALLOC_CAP_SPIRAM);
        if (ret != pdPASS) {
            ESP_LOGW(TAG, "SPIRAM stack alloc failed, retrying with internal RAM");
            ret = xTaskCreatePinnedToCore(
                output_task_func,
                "audio_out",
                OUTPUT_TASK_STACK,
                NULL,
                OUTPUT_TASK_PRIORITY,
                &s_out.output_task,
                core);
        }
    } else {
        ret = xTaskCreateWithCaps(
            output_task_func,
            "audio_out",
            OUTPUT_TASK_STACK,
            NULL,
            OUTPUT_TASK_PRIORITY,
            &s_out.output_task,
            MALLOC_CAP_SPIRAM);
        if (ret != pdPASS) {
            ESP_LOGW(TAG, "SPIRAM stack alloc failed, retrying with internal RAM");
            ret = xTaskCreate(
                output_task_func,
                "audio_out",
                OUTPUT_TASK_STACK,
                NULL,
                OUTPUT_TASK_PRIORITY,
                &s_out.output_task);
        }
    }

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create output task");
        esp_codec_dev_close(s_out.codec_dev);
        vSemaphoreDelete(s_out.src_mutex);
        heap_caps_free(s_out.dma_buffer);
        ring_deinit(&s_out.ring);
        s_out.codec_dev = NULL;
        return false;
    }

    LOGI_OUT("Audio output initialized successfully (core=%d)", core);
    LOGI_OUT("  Ring buffer: %d bytes", RING_BUFFER_SIZE);
    LOGI_OUT("  DMA chunk: %d samples (%d bytes)", DMA_CHUNK_SAMPLES, DMA_CHUNK_BYTES);

    return true;
}

void copilot_audio_out_deinit(void) {
    if (!s_out.codec_dev) {
        return;
    }

    LOGI_OUT("Deinitializing audio output");

    // Stop output task
    s_out.running = false;
    if (s_out.output_task) {
        // Wake up task if blocked
        xSemaphoreGive(s_out.ring.data_sem);
        vTaskDelay(pdMS_TO_TICKS(100));
        s_out.output_task = NULL;
    }

    // Close codec
    esp_codec_dev_close(s_out.codec_dev);
    s_out.codec_dev = NULL;

    // Free resources
    if (s_out.src_mutex) {
        vSemaphoreDelete(s_out.src_mutex);
        s_out.src_mutex = NULL;
    }

    if (s_out.dma_buffer) {
        heap_caps_free(s_out.dma_buffer);
        s_out.dma_buffer = NULL;
    }

    ring_deinit(&s_out.ring);
}

bool copilot_audio_out_is_ready(void) {
    return s_out.codec_dev != NULL && s_out.running;
}

bool copilot_audio_out_acquire(copilot_audio_src_t src) {
    if (!s_out.src_mutex || src == AUDIO_SRC_NONE) {
        return false;
    }

    xSemaphoreTake(s_out.src_mutex, portMAX_DELAY);

    copilot_audio_src_t current = s_out.active_src;

    // Higher priority can preempt
    if (src > current) {
        if (current != AUDIO_SRC_NONE) {
            LOGI_OUT("Source %d preempts %d", src, current);
        }
        // Always clear buffer when acquiring to avoid playing stale data
        // This fixes first TTS audio popping caused by residual buffer data
        ring_clear(&s_out.ring);
        // Also clear DMA buffer to prevent any leftover audio
        if (s_out.dma_buffer) {
            memset(s_out.dma_buffer, 0, DMA_CHUNK_BYTES);
        }

        // Pre-fill ring buffer with silence to warm up the codec DAC
        // This prevents audio popping when first audio data arrives
        // Write ~20ms of silence (320 stereo samples at 16kHz)
        // Use static buffer to avoid stack allocation
        static int16_t silence[64 * 2] = {0};  // 64 stereo samples per chunk
        for (int i = 0; i < 5; i++) {  // 5 * 64 = 320 samples = 20ms
            ring_write(&s_out.ring, (uint8_t *)silence, sizeof(silence), 0);
        }

        s_out.active_src = src;
        xSemaphoreGive(s_out.src_mutex);
        LOGI_OUT("Source %d acquired", src);
        return true;
    }

    // Same or lower priority
    if (src == current) {
        xSemaphoreGive(s_out.src_mutex);
        return true;  // Already holding
    }

    xSemaphoreGive(s_out.src_mutex);
    LOGI_OUT("Source %d blocked by %d", src, current);
    return false;
}

void copilot_audio_out_release(copilot_audio_src_t src) {
    if (!s_out.src_mutex) {
        return;
    }

    xSemaphoreTake(s_out.src_mutex, portMAX_DELAY);

    if (s_out.active_src == src) {
        LOGI_OUT("Source %d released", src);
        s_out.active_src = AUDIO_SRC_NONE;
    }

    xSemaphoreGive(s_out.src_mutex);
}

copilot_audio_src_t copilot_audio_out_get_active(void) {
    return s_out.active_src;
}

int copilot_audio_out_write(copilot_audio_src_t src,
                            const int16_t *samples,
                            int num_samples,
                            int timeout_ms) {
    if (!s_out.running || !samples || num_samples <= 0) {
        return 0;
    }

    // Check if this source has the output
    if (s_out.active_src != src) {
        return 0;
    }

    // Convert mono to stereo and write
    // Process in chunks to avoid large stack allocation
    const int chunk_size = 128;
    int16_t stereo_chunk[chunk_size * 2];
    int written = 0;

    while (written < num_samples) {
        int to_process = num_samples - written;
        if (to_process > chunk_size) {
            to_process = chunk_size;
        }

        // Mono to stereo conversion
        for (int i = 0; i < to_process; i++) {
            int16_t sample = samples[written + i];
            stereo_chunk[i * 2] = sample;      // Left
            stereo_chunk[i * 2 + 1] = sample;  // Right
        }

        int bytes_to_write = to_process * 2 * sizeof(int16_t);
        int bytes_written = ring_write(&s_out.ring, (uint8_t *)stereo_chunk,
                                       bytes_to_write, timeout_ms);

        written += bytes_written / (2 * sizeof(int16_t));

        if (bytes_written < bytes_to_write) {
            break;  // Buffer full or timeout
        }
    }

    return written;
}

int copilot_audio_out_write_stereo(copilot_audio_src_t src,
                                   const int16_t *samples,
                                   int num_samples,
                                   int timeout_ms) {
    if (!s_out.running || !samples || num_samples <= 0) {
        return 0;
    }

    if (s_out.active_src != src) {
        return 0;
    }

    int bytes_to_write = num_samples * 2 * sizeof(int16_t);
    int bytes_written = ring_write(&s_out.ring, (uint8_t *)samples,
                                   bytes_to_write, timeout_ms);

    return bytes_written / (2 * sizeof(int16_t));
}

bool copilot_audio_out_flush(copilot_audio_src_t src, int timeout_ms) {
    if (!s_out.running || s_out.active_src != src) {
        return false;
    }

    TickType_t start = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(timeout_ms);

    while (ring_used(&s_out.ring) > 0) {
        if (timeout_ms > 0) {
            TickType_t elapsed = xTaskGetTickCount() - start;
            if (elapsed >= timeout) {
                return false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    return true;
}

void copilot_audio_out_set_volume(int volume) {
    if (volume < 0) volume = 0;
    if (volume > 100) volume = 100;

    s_out.volume = volume;

    if (s_out.codec_dev) {
        esp_codec_dev_set_out_vol(s_out.codec_dev, volume);
        LOGI_OUT("Volume set to %d%%", volume);
    }
}

int copilot_audio_out_get_volume(void) {
    return s_out.volume;
}

void copilot_audio_out_get_buffer_status(int *out_used, int *out_free) {
    if (out_used) {
        *out_used = ring_used(&s_out.ring);
    }
    if (out_free) {
        *out_free = ring_free(&s_out.ring);
    }
}

bool copilot_audio_out_play_tone(uint16_t freq_hz, uint16_t duration_ms, uint8_t volume) {
    if (!s_out.running || s_out.sample_rate <= 0 || freq_hz == 0 || duration_ms == 0) {
        return false;
    }

    if (volume > 100) {
        volume = 100;
    }

    uint32_t total_samples = ((uint32_t)duration_ms * (uint32_t)s_out.sample_rate) / 1000;
    if (total_samples == 0) {
        return false;
    }

    int fade_samples = s_out.sample_rate / 100; // 10ms fade
    if (fade_samples * 2 > (int)total_samples) {
        fade_samples = (int)total_samples / 2;
    }

    int amplitude = (12000 * (int)volume) / 100;
    if (amplitude <= 0) {
        return false;
    }

    portENTER_CRITICAL(&s_tone_lock);
    s_out.tone.active = true;
    s_out.tone.samples_left = total_samples;
    s_out.tone.total_samples = total_samples;
    s_out.tone.sample_index = 0;
    s_out.tone.phase = 0.0f;
    s_out.tone.phase_inc = 2.0f * (float)M_PI * (float)freq_hz / (float)s_out.sample_rate;
    s_out.tone.amplitude = amplitude;
    s_out.tone.fade_samples = fade_samples;
    portEXIT_CRITICAL(&s_tone_lock);

#if CONFIG_COPILOT_LOG_AUDIO_OUT
    ESP_LOGI(TAG, "Tone start: %u Hz, %u ms, vol=%u%%",
             (unsigned)freq_hz,
             (unsigned)duration_ms,
             (unsigned)volume);
#endif

    return true;
}

bool copilot_audio_out_is_tone_active(void) {
    bool active = false;
    portENTER_CRITICAL(&s_tone_lock);
    active = s_out.tone.active;
    portEXIT_CRITICAL(&s_tone_lock);
    return active;
}
