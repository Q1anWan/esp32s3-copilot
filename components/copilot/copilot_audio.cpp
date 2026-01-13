#include "copilot_audio.h"

#include <math.h>
#include <string.h>
#include <stdint.h>

#include "esp_codec_dev.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "bsp/esp-bsp.h"
#include "sdkconfig.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "copilot_audio";

static int copilot_normalize_core(int core) {
    if (core < 0) {
        return -1;
    }
    if (core >= (int)configNUM_CORES) {
        return -1;
    }
    return core;
}

static esp_codec_dev_handle_t s_play_dev = nullptr;
static esp_codec_dev_sample_info_t s_sample_info = {};
static QueueHandle_t s_audio_queue = nullptr;
static TaskHandle_t s_audio_task = nullptr;

struct audio_req_t {
    uint16_t freq_hz;
    uint16_t duration_ms;
    uint8_t volume;
};

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
    if (!s_play_dev) {
        return;
    }

    const int sample_rate = s_sample_info.sample_rate;
    const int channels = 2;
    const int amplitude = 12000;
    const int total_samples = (req.duration_ms * sample_rate) / 1000;
    const int fade_samples = sample_rate / 100; // 10ms fade
    const int chunk_samples = 240;
    int samples_left = total_samples;

    float phase = 0.0f;
    const float phase_inc = 2.0f * (float)M_PI * (float)req.freq_hz / (float)sample_rate;

    int16_t buffer[chunk_samples * channels];
    int sample_index = 0;

    esp_codec_dev_set_out_vol(s_play_dev, req.volume);

    while (samples_left > 0) {
        int now_samples = samples_left > chunk_samples ? chunk_samples : samples_left;
        for (int i = 0; i < now_samples; ++i) {
            float env = 1.0f;
            if (sample_index < fade_samples) {
                env = (float)sample_index / (float)fade_samples;
            } else if (sample_index > total_samples - fade_samples) {
                env = (float)(total_samples - sample_index) / (float)fade_samples;
            }
            float sample_f = sinf(phase) * (float)amplitude * env;
            int16_t sample = (int16_t)sample_f;
            buffer[i * 2] = sample;
            buffer[i * 2 + 1] = sample;

            phase += phase_inc;
            if (phase > 2.0f * (float)M_PI) {
                phase -= 2.0f * (float)M_PI;
            }
            sample_index++;
        }
        esp_codec_dev_write(s_play_dev, buffer, now_samples * channels * sizeof(int16_t));
        samples_left -= now_samples;
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
    if (s_audio_queue || s_play_dev) {
        return;
    }

    s_play_dev = bsp_audio_codec_speaker_init();
    if (!s_play_dev) {
        ESP_LOGE(TAG, "Failed to init speaker codec");
        return;
    }
    s_sample_info.sample_rate = 24000;
    s_sample_info.channel = 2;
    s_sample_info.bits_per_sample = 16;
    esp_codec_dev_open(s_play_dev, &s_sample_info);
    esp_codec_dev_set_out_vol(s_play_dev, 80);

    s_audio_queue = xQueueCreate(6, sizeof(audio_req_t));
    if (!s_audio_queue) {
        return;
    }

    int core = copilot_normalize_core(CONFIG_COPILOT_AUDIO_CORE);
    BaseType_t task_ok;
    // Stack optimized based on high watermark: 720-2288 bytes used -> 2.5KB allocation
    if (core >= 0) {
        task_ok = xTaskCreatePinnedToCore(copilot_audio_task, "copilot_audio", 2560, nullptr, 3, &s_audio_task, core);
    } else {
        task_ok = xTaskCreate(copilot_audio_task, "copilot_audio", 2560, nullptr, 3, &s_audio_task);
    }
    if (task_ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create audio task");
    } else {
        ESP_LOGI(TAG, "Audio task core=%d", core);
    }
}

void copilot_audio_play(const char *sound_id) {
    if (!s_audio_queue || !s_play_dev) {
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
    return s_play_dev && s_audio_queue;
}
