#include "copilot_audio.h"

#include <string.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "sdkconfig.h"

// Use unified audio output manager
#include "copilot_audio_out.h"

static const char *TAG = "copilot_audio";

// Conditional logging
#if CONFIG_COPILOT_LOG_AUDIO
#define LOGI_AUDIO(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#else
#define LOGI_AUDIO(fmt, ...) do {} while(0)
#endif

static int copilot_normalize_core(int core) {
    if (core < 0) {
        return -1;
    }
    if (core >= (int)configNUM_CORES) {
        return -1;
    }
    return core;
}

// Audio output manager handles the codec now
static QueueHandle_t s_audio_queue = nullptr;
static TaskHandle_t s_audio_task = nullptr;

// Sample rate must match audio_out (16kHz)
static const int kSampleRate = 16000;

#define AUDIO_TASK_STACK_BYTES (4 * 1024)

struct audio_req_t {
    uint16_t freq_hz;
    uint16_t duration_ms;
    uint8_t volume;
};

static const UBaseType_t kAudioQueueLen = 6;
#if CONFIG_FREERTOS_SUPPORT_STATIC_ALLOCATION
static StaticQueue_t s_audio_queue_struct;
static uint8_t s_audio_queue_storage[kAudioQueueLen * sizeof(audio_req_t)];
#endif

struct tone_desc_t {
    const char *id;
    uint16_t freq_hz;
    uint16_t duration_ms;
    uint8_t volume;
};

static const tone_desc_t kTones[] = {
    {"beep_short", 920, 180, 80},
    {"beep_long", 740, 420, 80},
    {"chime", 1200, 140, 75},
    {"tap", 1400, 60, 70},
};

static bool copilot_audio_lookup(const char *id, audio_req_t *out) {
    if (!id || !out) {
        return false;
    }
    for (size_t i = 0; i < sizeof(kTones) / sizeof(kTones[0]); ++i) {
        if (strcmp(id, kTones[i].id) == 0) {
            out->freq_hz = kTones[i].freq_hz;
            out->duration_ms = kTones[i].duration_ms;
            out->volume = kTones[i].volume;
            return true;
        }
    }
    return false;
}

static void copilot_audio_play_tone(const audio_req_t &req) {
    if (!copilot_audio_out_play_tone(req.freq_hz, req.duration_ms, req.volume)) {
        LOGI_AUDIO("Audio output not ready, skip tone");
        return;
    }

    TickType_t start = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(req.duration_ms + 200);
    while (copilot_audio_out_is_tone_active()) {
        TickType_t elapsed = xTaskGetTickCount() - start;
        if (elapsed >= timeout) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void copilot_audio_task(void *arg) {
    (void)arg;
    audio_req_t req = {};
    while (true) {
        if (xQueueReceive(s_audio_queue, &req, portMAX_DELAY) == pdTRUE) {
            copilot_audio_play_tone(req);
        }
    }
}

void copilot_audio_init(void) {
    if (s_audio_queue) {
        return;
    }

    // Initialize audio output manager (shared with voice module)
    if (!copilot_audio_out_is_ready()) {
        copilot_audio_out_config_t config = {};
        config.sample_rate = kSampleRate;
        config.speaker_volume = 80;
        if (!copilot_audio_out_init(&config)) {
            ESP_LOGE(TAG, "Failed to init audio output manager");
            return;
        }
    }

#if CONFIG_FREERTOS_SUPPORT_STATIC_ALLOCATION
    s_audio_queue = xQueueCreateStatic(kAudioQueueLen, sizeof(audio_req_t),
                                       s_audio_queue_storage, &s_audio_queue_struct);
#else
    s_audio_queue = xQueueCreate(kAudioQueueLen, sizeof(audio_req_t));
#endif
    if (!s_audio_queue) {
        ESP_LOGE(TAG, "Failed to create audio queue");
        return;
    }

    int core = copilot_normalize_core(CONFIG_COPILOT_AUDIO_CORE);
    BaseType_t task_ok = pdFAIL;
#if CONFIG_FREERTOS_SUPPORT_STATIC_ALLOCATION
    BaseType_t affinity = (core >= 0) ? core : tskNO_AFFINITY;
    task_ok = xTaskCreatePinnedToCoreWithCaps(
        copilot_audio_task,
        "copilot_audio",
        AUDIO_TASK_STACK_BYTES,
        nullptr,
        3,
        &s_audio_task,
        affinity,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (task_ok != pdPASS) {
        ESP_LOGW(TAG, "Audio task PSRAM stack alloc failed, fallback to internal");
    }
#endif
    if (task_ok != pdPASS) {
        if (core >= 0) {
            task_ok = xTaskCreatePinnedToCore(copilot_audio_task, "copilot_audio", AUDIO_TASK_STACK_BYTES, nullptr, 3,
                                              &s_audio_task, core);
        } else {
            task_ok = xTaskCreate(copilot_audio_task, "copilot_audio", AUDIO_TASK_STACK_BYTES, nullptr, 3, &s_audio_task);
        }
    }
    if (task_ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create audio task");
    } else {
        LOGI_AUDIO("Audio task core=%d (using unified audio_out)", core);
    }
}

void copilot_audio_play(const char *sound_id) {
    if (!s_audio_queue) {
        return;
    }

    audio_req_t req = {};
    if (!copilot_audio_lookup(sound_id, &req)) {
        req.freq_hz = 880;
        req.duration_ms = 160;
        req.volume = 80;
    }
    xQueueSend(s_audio_queue, &req, 0);
}

bool copilot_audio_is_ready(void) {
    return s_audio_queue && copilot_audio_out_is_ready();
}
