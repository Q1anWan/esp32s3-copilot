#include "copilot_audio.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <sys/stat.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "sdkconfig.h"

// Use unified audio output manager
#include "copilot_audio_out.h"
#include "copilot_mqtt.h"
#include "bsp/esp-bsp.h"

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
static copilot_audio_file_event_cb_t s_file_event_cb = nullptr;
static void *s_file_event_user = nullptr;

// Sample rate must match audio_out (16kHz)
static const int kSampleRate = 16000;

#define AUDIO_TASK_STACK_BYTES (12 * 1024)
#define AUDIO_SCENE_ID_MAX 32
#define AUDIO_SEQUENCE_ID_MAX 96
#define AUDIO_FILE_CHUNK_SAMPLES 128

enum audio_req_kind_t {
    AUDIO_REQ_TONE = 0,
    AUDIO_REQ_FILE,
};

struct audio_req_t {
    audio_req_kind_t kind;
    uint16_t freq_hz;
    uint16_t duration_ms;
    uint8_t volume;
    uint32_t generation;
    char path[COPILOT_AUDIO_PATH_MAX];
    char scene[AUDIO_SCENE_ID_MAX];
    char sequence[AUDIO_SEQUENCE_ID_MAX];
};

static const UBaseType_t kAudioQueueLen = 6;
#if CONFIG_FREERTOS_SUPPORT_STATIC_ALLOCATION
static StaticQueue_t s_audio_queue_struct;
static uint8_t s_audio_queue_storage[kAudioQueueLen * sizeof(audio_req_t)];
#endif

static bool s_sd_mounted = false;
static volatile uint32_t s_audio_generation = 1;
static copilot_audio_status_t s_status = {};

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
    {"speaker_test", 880, 1500, 95},
};

static bool copilot_audio_lookup(const char *id, audio_req_t *out) {
    if (!id || !out) {
        return false;
    }
    for (size_t i = 0; i < sizeof(kTones) / sizeof(kTones[0]); ++i) {
        if (strcmp(id, kTones[i].id) == 0) {
            out->kind = AUDIO_REQ_TONE;
            out->freq_hz = kTones[i].freq_hz;
            out->duration_ms = kTones[i].duration_ms;
            out->volume = kTones[i].volume;
            return true;
        }
    }
    return false;
}

static void copilot_audio_set_error(const char *msg) {
    if (!msg) {
        s_status.last_error[0] = '\0';
        return;
    }
    strncpy(s_status.last_error, msg, sizeof(s_status.last_error) - 1);
    s_status.last_error[sizeof(s_status.last_error) - 1] = '\0';
}

static void copilot_audio_emit_file_event(copilot_audio_file_event_kind_t kind,
                                          const audio_req_t &req) {
    if (!s_file_event_cb) {
        return;
    }
    copilot_audio_file_event_t event = {};
    event.kind = kind;
    event.generation = req.generation;
    event.scene = req.scene;
    event.sequence = req.sequence;
    event.path = req.path;
    event.error = s_status.last_error;
    s_file_event_cb(&event, s_file_event_user);
}

static bool copilot_audio_mount_sd(void) {
    if (s_sd_mounted) {
        return true;
    }
    if (bsp_sdcard) {
        s_sd_mounted = true;
        s_status.sd_mounted = true;
        copilot_audio_set_error("");
        return true;
    }

    esp_err_t err = bsp_sdcard_mount();
    if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
        s_sd_mounted = true;
        s_status.sd_mounted = true;
        copilot_audio_set_error("");
        ESP_LOGI(TAG, "SD card mounted at %s", BSP_SD_MOUNT_POINT);
        return true;
    }

    char err_msg[96];
    snprintf(err_msg, sizeof(err_msg), "sd_mount_failed:%s", esp_err_to_name(err));
    copilot_audio_set_error(err_msg);
    ESP_LOGW(TAG, "SD card mount failed: %s", esp_err_to_name(err));
    return false;
}

static void copilot_audio_unmount_sd(void) {
    if (!s_sd_mounted) {
        return;
    }
    esp_err_t err = bsp_sdcard_unmount();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "SD card unmount returned: %s", esp_err_to_name(err));
    }
    s_sd_mounted = false;
    s_status.sd_mounted = false;
}

static bool copilot_audio_remount_sd(const char *reason) {
    ESP_LOGW(TAG, "Remount SD card (%s)", reason ? reason : "unknown");
    copilot_audio_unmount_sd();
    vTaskDelay(pdMS_TO_TICKS(250));
    return copilot_audio_mount_sd();
}

static void copilot_audio_wait_network_settle(void) {
    copilot_network_status_t net = {};
    for (int i = 0; i < 70; ++i) {
        if (!copilot_mqtt_get_status(&net) || !net.wifi_started ||
            net.wifi_connected || net.ssid[0] == '\0') {
            return;
        }
        if (i == 0) {
            ESP_LOGI(TAG, "Wait for WiFi to settle before SD audio read");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static bool copilot_path_exists(const char *path) {
    struct stat st = {};
    return path && stat(path, &st) == 0 && S_ISREG(st.st_mode);
}

static bool copilot_is_audio_id_char(char c) {
    return (c >= 'a' && c <= 'z') ||
           (c >= 'A' && c <= 'Z') ||
           (c >= '0' && c <= '9') ||
           c == '_' || c == '-';
}

static void copilot_sanitize_audio_id(const char *text, const char *fallback, char *out, size_t out_len) {
    if (!out || out_len == 0) {
        return;
    }
    if (!text || text[0] == '\0') {
        text = fallback && fallback[0] ? fallback : "default";
    }

    size_t w = 0;
    for (size_t i = 0; text[i] != '\0' && w + 1 < out_len; ++i) {
        char c = text[i];
        if (copilot_is_audio_id_char(c)) {
            out[w++] = c;
        } else if (c == ' ' || c == '.') {
            out[w++] = '_';
        }
    }
    if (w == 0) {
        const char *fallback = "default";
        while (*fallback && w + 1 < out_len) {
            out[w++] = *fallback++;
        }
    }
    out[w] = '\0';
}

static bool copilot_parse_u32_exact(const char *text, uint32_t *out) {
    if (!text || text[0] == '\0' || !out) {
        return false;
    }
    uint32_t value = 0;
    for (size_t i = 0; text[i] != '\0'; ++i) {
        if (text[i] < '0' || text[i] > '9') {
            return false;
        }
        uint32_t digit = (uint32_t)(text[i] - '0');
        if (value > (UINT32_MAX - digit) / 10u) {
            return false;
        }
        value = value * 10u + digit;
    }
    *out = value;
    return true;
}

static bool copilot_audio_resolve_scene_path(const char *scene_id, const char *sequence_id,
                                             char *out_path, size_t out_len) {
    if (!out_path || out_len == 0) {
        return false;
    }

    char scene[AUDIO_SCENE_ID_MAX];
    char sequence[AUDIO_SEQUENCE_ID_MAX];
    copilot_sanitize_audio_id(scene_id, "default", scene, sizeof(scene));
    copilot_sanitize_audio_id(sequence_id, "1", sequence, sizeof(sequence));

    char candidates[8][COPILOT_AUDIO_PATH_MAX];
    size_t candidate_count = 0;
    uint32_t numeric_sequence = 0;
    if (copilot_parse_u32_exact(sequence, &numeric_sequence)) {
        snprintf(candidates[candidate_count++], sizeof(candidates[0]), "%s/audio/%s/%03lu.wav",
                 BSP_SD_MOUNT_POINT, scene, (unsigned long)numeric_sequence);
        snprintf(candidates[candidate_count++], sizeof(candidates[0]), "%s/audio/%s/%lu.wav",
                 BSP_SD_MOUNT_POINT, scene, (unsigned long)numeric_sequence);
        snprintf(candidates[candidate_count++], sizeof(candidates[0]), "%s/audio/%s_%03lu.wav",
                 BSP_SD_MOUNT_POINT, scene, (unsigned long)numeric_sequence);
        snprintf(candidates[candidate_count++], sizeof(candidates[0]), "%s/audio/%s_%lu.wav",
                 BSP_SD_MOUNT_POINT, scene, (unsigned long)numeric_sequence);
        snprintf(candidates[candidate_count++], sizeof(candidates[0]), "%s/audio/%s/%03lu.pcm",
                 BSP_SD_MOUNT_POINT, scene, (unsigned long)numeric_sequence);
        snprintf(candidates[candidate_count++], sizeof(candidates[0]), "%s/audio/%s/%lu.pcm",
                 BSP_SD_MOUNT_POINT, scene, (unsigned long)numeric_sequence);
        snprintf(candidates[candidate_count++], sizeof(candidates[0]), "%s/audio/%s_%03lu.pcm",
                 BSP_SD_MOUNT_POINT, scene, (unsigned long)numeric_sequence);
        snprintf(candidates[candidate_count++], sizeof(candidates[0]), "%s/audio/%s_%lu.pcm",
                 BSP_SD_MOUNT_POINT, scene, (unsigned long)numeric_sequence);
    } else {
        snprintf(candidates[candidate_count++], sizeof(candidates[0]), "%s/audio/%s/%s.wav",
                 BSP_SD_MOUNT_POINT, scene, sequence);
        snprintf(candidates[candidate_count++], sizeof(candidates[0]), "%s/audio/%s_%s.wav",
                 BSP_SD_MOUNT_POINT, scene, sequence);
        snprintf(candidates[candidate_count++], sizeof(candidates[0]), "%s/audio/%s/%s.pcm",
                 BSP_SD_MOUNT_POINT, scene, sequence);
        snprintf(candidates[candidate_count++], sizeof(candidates[0]), "%s/audio/%s_%s.pcm",
                 BSP_SD_MOUNT_POINT, scene, sequence);
    }

    if (copilot_audio_mount_sd()) {
        for (size_t i = 0; i < candidate_count; ++i) {
            if (copilot_path_exists(candidates[i])) {
                strncpy(out_path, candidates[i], out_len - 1);
                out_path[out_len - 1] = '\0';
                return true;
            }
        }
    }

    strncpy(out_path, candidates[0], out_len - 1);
    out_path[out_len - 1] = '\0';
    return false;
}

static uint16_t read_le16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t read_le32(const uint8_t *p) {
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static bool read_exact(FILE *fp, void *buf, size_t len) {
    return fp && buf && fread(buf, 1, len, fp) == len;
}

typedef struct {
    uint16_t channels;
    uint16_t bits_per_sample;
    uint32_t sample_rate;
    uint32_t data_offset;
    uint32_t data_size;
} wav_info_t;

static bool parse_wav_header(FILE *fp, wav_info_t *info) {
    if (!fp || !info) {
        return false;
    }

    uint8_t header[12];
    if (!read_exact(fp, header, sizeof(header))) {
        return false;
    }
    if (memcmp(header, "RIFF", 4) != 0 || memcmp(header + 8, "WAVE", 4) != 0) {
        return false;
    }

    bool have_fmt = false;
    bool have_data = false;
    memset(info, 0, sizeof(*info));

    while (!have_data) {
        uint8_t chunk_header[8];
        if (!read_exact(fp, chunk_header, sizeof(chunk_header))) {
            return false;
        }
        uint32_t chunk_size = read_le32(chunk_header + 4);
        long payload_pos = ftell(fp);
        if (payload_pos < 0) {
            return false;
        }

        if (memcmp(chunk_header, "fmt ", 4) == 0) {
            uint8_t fmt[16];
            if (chunk_size < sizeof(fmt) || !read_exact(fp, fmt, sizeof(fmt))) {
                return false;
            }
            uint16_t audio_format = read_le16(fmt);
            info->channels = read_le16(fmt + 2);
            info->sample_rate = read_le32(fmt + 4);
            info->bits_per_sample = read_le16(fmt + 14);
            if (audio_format != 1) {
                ESP_LOGW(TAG, "Unsupported WAV format=%u (need PCM=1)", (unsigned)audio_format);
                return false;
            }
            have_fmt = true;
        } else if (memcmp(chunk_header, "data", 4) == 0) {
            info->data_offset = (uint32_t)payload_pos;
            info->data_size = chunk_size;
            have_data = true;
        }

        if (!have_data) {
            long next_pos = payload_pos + (long)chunk_size + (long)(chunk_size & 1u);
            if (fseek(fp, next_pos, SEEK_SET) != 0) {
                return false;
            }
        }
    }

    return have_fmt && have_data;
}

static int16_t *copilot_audio_alloc_file_buffer(size_t bytes) {
    int16_t *buf = (int16_t *)heap_caps_aligned_alloc(512, bytes,
                                                       MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    return buf;
}

static uint32_t copilot_audio_next_generation(void) {
    uint32_t next = s_audio_generation + 1;
    if (next == 0) {
        next = 1;
    }
    s_audio_generation = next;
    return next;
}

static bool copilot_audio_req_cancelled(uint32_t generation) {
    return generation != 0 && generation != s_audio_generation;
}

static void copilot_audio_drop_pending(void) {
    if (!s_audio_queue) {
        return;
    }
    audio_req_t dropped = {};
    while (xQueueReceive(s_audio_queue, &dropped, 0) == pdTRUE) {
    }
}

static bool copilot_audio_enqueue_file(audio_req_t *req) {
    if (!s_audio_queue || !req) {
        return false;
    }

    req->generation = copilot_audio_next_generation();
    copilot_audio_drop_pending();
    copilot_audio_out_abort(AUDIO_SRC_FILE);

    if (xQueueSendToFront(s_audio_queue, req, 0) == pdTRUE) {
        return true;
    }

    copilot_audio_set_error("audio_queue_full");
    return false;
}

static bool copilot_audio_stream_pcm(FILE *fp, const char *path, uint32_t generation, bool *out_cancelled) {
    int16_t *mono = copilot_audio_alloc_file_buffer(AUDIO_FILE_CHUNK_SAMPLES * sizeof(int16_t));
    if (!mono) {
        copilot_audio_set_error("pcm_buffer_alloc_failed");
        return false;
    }

    bool ok = true;
    while (true) {
        if (copilot_audio_req_cancelled(generation)) {
            if (out_cancelled) {
                *out_cancelled = true;
            }
            ok = false;
            break;
        }
        size_t got = fread(mono, sizeof(int16_t), AUDIO_FILE_CHUNK_SAMPLES, fp);
        if (got > 0) {
            int written = copilot_audio_out_write(AUDIO_SRC_FILE, mono, (int)got, 200);
            if (copilot_audio_req_cancelled(generation)) {
                if (out_cancelled) {
                    *out_cancelled = true;
                }
                ok = false;
                break;
            }
            if (written <= 0 && copilot_audio_out_get_active() != AUDIO_SRC_FILE) {
                ESP_LOGW(TAG, "File playback preempted: %s", path ? path : "");
                ok = false;
                break;
            }
        }
        if (got < AUDIO_FILE_CHUNK_SAMPLES) {
            if (ferror(fp)) {
                copilot_audio_set_error("pcm_read_failed");
                ok = false;
            }
            break;
        }
    }

    heap_caps_free(mono);
    return ok;
}

static bool copilot_audio_stream_wav(FILE *fp, const char *path, const wav_info_t *info,
                                     uint32_t generation, bool *out_cancelled) {
    if (!fp || !info) {
        return false;
    }
    if (info->sample_rate != (uint32_t)kSampleRate || info->bits_per_sample != 16 ||
        (info->channels != 1 && info->channels != 2)) {
        char err_msg[96];
        snprintf(err_msg, sizeof(err_msg), "bad_wav:%luHz:%uch:%ubit",
                 (unsigned long)info->sample_rate,
                 (unsigned)info->channels,
                 (unsigned)info->bits_per_sample);
        copilot_audio_set_error(err_msg);
        ESP_LOGW(TAG, "Unsupported WAV %s (need 16kHz, 16-bit, mono/stereo): rate=%lu ch=%u bits=%u",
                 path ? path : "",
                 (unsigned long)info->sample_rate,
                 (unsigned)info->channels,
                 (unsigned)info->bits_per_sample);
        return false;
    }

    if (fseek(fp, (long)info->data_offset, SEEK_SET) != 0) {
        copilot_audio_set_error("wav_seek_failed");
        return false;
    }

    size_t frame_bytes = info->channels * sizeof(int16_t);
    size_t buf_bytes = AUDIO_FILE_CHUNK_SAMPLES * frame_bytes;
    int16_t *buf = copilot_audio_alloc_file_buffer(buf_bytes);
    if (!buf) {
        copilot_audio_set_error("wav_buffer_alloc_failed");
        return false;
    }

    uint32_t remaining = info->data_size;
    bool ok = true;
    while (remaining > 0) {
        if (copilot_audio_req_cancelled(generation)) {
            if (out_cancelled) {
                *out_cancelled = true;
            }
            ok = false;
            break;
        }
        size_t to_read = remaining < buf_bytes ? remaining : buf_bytes;
        to_read -= to_read % frame_bytes;
        if (to_read == 0) {
            break;
        }

        size_t got = fread(buf, 1, to_read, fp);
        if (got == 0) {
            if (ferror(fp)) {
                copilot_audio_set_error("wav_read_failed");
                ok = false;
            }
            break;
        }

        size_t frames = got / frame_bytes;
        int written = 0;
        if (info->channels == 1) {
            written = copilot_audio_out_write(AUDIO_SRC_FILE, buf, (int)frames, 200);
        } else {
            written = copilot_audio_out_write_stereo(AUDIO_SRC_FILE, buf, (int)frames, 200);
        }
        if (copilot_audio_req_cancelled(generation)) {
            if (out_cancelled) {
                *out_cancelled = true;
            }
            ok = false;
            break;
        }
        if (written <= 0 && copilot_audio_out_get_active() != AUDIO_SRC_FILE) {
            ESP_LOGW(TAG, "File playback preempted: %s", path ? path : "");
            ok = false;
            break;
        }

        remaining -= (uint32_t)got;
        if (got < to_read) {
            break;
        }
    }

    heap_caps_free(buf);
    return ok;
}

static bool has_ext(const char *path, const char *ext) {
    if (!path || !ext) {
        return false;
    }
    const char *dot = strrchr(path, '.');
    if (!dot) {
        return false;
    }
    ++dot;
    while (*dot && *ext) {
        char a = *dot++;
        char b = *ext++;
        if (a >= 'A' && a <= 'Z') {
            a = (char)(a - 'A' + 'a');
        }
        if (b >= 'A' && b <= 'Z') {
            b = (char)(b - 'A' + 'a');
        }
        if (a != b) {
            return false;
        }
    }
    return *dot == '\0' && *ext == '\0';
}

static FILE *copilot_audio_open_file_with_retry(const char *path, int *out_errno) {
    errno = 0;
    FILE *fp = fopen(path, "rb");
    if (fp) {
        setvbuf(fp, nullptr, _IONBF, 0);
        if (out_errno) {
            *out_errno = 0;
        }
        return fp;
    }

    int first_errno = errno;
    if (first_errno == EIO || first_errno == ENODEV) {
        if (copilot_audio_remount_sd("file_open_failed")) {
            errno = 0;
            fp = fopen(path, "rb");
            if (fp) {
                setvbuf(fp, nullptr, _IONBF, 0);
                ESP_LOGI(TAG, "SD file open recovered after remount: %s", path);
                if (out_errno) {
                    *out_errno = 0;
                }
                return fp;
            }
        }
    }

    if (out_errno) {
        *out_errno = errno ? errno : first_errno;
    }
    return nullptr;
}

static void copilot_audio_play_file(const audio_req_t &req) {
    audio_req_t play_req = req;
    const uint32_t generation = play_req.generation;
    copilot_audio_wait_network_settle();
    if (copilot_audio_req_cancelled(generation)) {
        return;
    }
    if (play_req.path[0] == '\0') {
        bool found = copilot_audio_resolve_scene_path(play_req.scene, play_req.sequence,
                                                      play_req.path, sizeof(play_req.path));
        if (!found) {
            ESP_LOGW(TAG, "Audio ID not found yet: scene=%s seq=%s expected=%s",
                     play_req.scene, play_req.sequence, play_req.path);
        }
    }
    if (copilot_audio_req_cancelled(generation)) {
        return;
    }
    if (!copilot_audio_mount_sd()) {
        copilot_audio_emit_file_event(COPILOT_AUDIO_FILE_FAILED, play_req);
        return;
    }
    if (copilot_audio_req_cancelled(generation)) {
        return;
    }
    if (!copilot_audio_out_acquire(AUDIO_SRC_FILE)) {
        copilot_audio_set_error("audio_output_busy");
        ESP_LOGW(TAG, "Audio output busy, skip file: %s", play_req.path);
        copilot_audio_emit_file_event(COPILOT_AUDIO_FILE_FAILED, play_req);
        return;
    }
    if (copilot_audio_req_cancelled(generation)) {
        copilot_audio_out_abort(AUDIO_SRC_FILE);
        return;
    }

    s_status.playing_file = true;
    strncpy(s_status.current_path, play_req.path, sizeof(s_status.current_path) - 1);
    s_status.current_path[sizeof(s_status.current_path) - 1] = '\0';
    copilot_audio_set_error("");

    int open_errno = 0;
    FILE *fp = copilot_audio_open_file_with_retry(play_req.path, &open_errno);
    if (!fp) {
        char err_msg[96];
        snprintf(err_msg, sizeof(err_msg), "open_failed:%d", open_errno);
        copilot_audio_set_error(err_msg);
        ESP_LOGW(TAG, "Failed to open audio file %s (errno=%d)", play_req.path, open_errno);
        s_status.playing_file = false;
        copilot_audio_out_release(AUDIO_SRC_FILE);
        copilot_audio_emit_file_event(COPILOT_AUDIO_FILE_FAILED, play_req);
        return;
    }

    ESP_LOGI(TAG, "Play SD audio: scene=%s seq=%s path=%s",
             play_req.scene[0] ? play_req.scene : "default",
             play_req.sequence[0] ? play_req.sequence : "1",
             play_req.path);
    bool ok = false;
    bool cancelled = false;
    if (has_ext(play_req.path, "pcm")) {
        ok = copilot_audio_stream_pcm(fp, play_req.path, generation, &cancelled);
    } else {
        wav_info_t info = {};
        if (parse_wav_header(fp, &info)) {
            ok = copilot_audio_stream_wav(fp, play_req.path, &info, generation, &cancelled);
        } else {
            copilot_audio_set_error("wav_parse_failed");
            ESP_LOGW(TAG, "Invalid WAV file: %s", play_req.path);
        }
    }
    fclose(fp);

    if (cancelled) {
        ESP_LOGI(TAG, "SD audio cancelled by newer request: %s", play_req.path);
        copilot_audio_out_abort(AUDIO_SRC_FILE);
        s_status.playing_file = false;
        copilot_audio_set_error("");
        return;
    }

    copilot_audio_out_flush(AUDIO_SRC_FILE, 1500);
    copilot_audio_out_release(AUDIO_SRC_FILE);
    s_status.playing_file = false;
    if (ok) {
        s_status.files_played++;
        copilot_audio_set_error("");
        ESP_LOGI(TAG, "SD audio done: %s", play_req.path);
        copilot_audio_emit_file_event(COPILOT_AUDIO_FILE_DONE, play_req);
    } else {
        copilot_audio_emit_file_event(COPILOT_AUDIO_FILE_FAILED, play_req);
    }
}

static void copilot_audio_play_tone(const audio_req_t &req) {
    LOGI_AUDIO("Play tone start freq=%u duration=%u volume=%u",
               (unsigned)req.freq_hz,
               (unsigned)req.duration_ms,
               (unsigned)req.volume);
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
    LOGI_AUDIO("Play tone done freq=%u", (unsigned)req.freq_hz);
}

static void copilot_audio_task(void *arg) {
    (void)arg;
    audio_req_t req = {};
    while (true) {
        if (xQueueReceive(s_audio_queue, &req, portMAX_DELAY) == pdTRUE) {
            if (req.kind == AUDIO_REQ_FILE) {
                copilot_audio_play_file(req);
            } else {
                copilot_audio_play_tone(req);
            }
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
        s_status.ready = true;
        s_status.sd_mounted = s_sd_mounted;
        LOGI_AUDIO("Audio task core=%d (using unified audio_out)", core);
    }
}

void copilot_audio_set_file_event_callback(copilot_audio_file_event_cb_t cb, void *user) {
    s_file_event_cb = cb;
    s_file_event_user = user;
}

void copilot_audio_play(const char *sound_id) {
    if (!s_audio_queue) {
        return;
    }

    audio_req_t req = {};
    if (!copilot_audio_lookup(sound_id, &req)) {
        req.kind = AUDIO_REQ_TONE;
        req.freq_hz = 880;
        req.duration_ms = 160;
        req.volume = 80;
    }
    LOGI_AUDIO("Queue tone id=%s freq=%u duration=%u volume=%u",
               sound_id ? sound_id : "default",
               (unsigned)req.freq_hz,
               (unsigned)req.duration_ms,
               (unsigned)req.volume);
    xQueueSend(s_audio_queue, &req, 0);
}

bool copilot_audio_play_path(const char *path) {
    if (!s_audio_queue || !path || path[0] == '\0') {
        return false;
    }
    audio_req_t req = {};
    req.kind = AUDIO_REQ_FILE;
    strncpy(req.path, path, sizeof(req.path) - 1);
    req.path[sizeof(req.path) - 1] = '\0';
    strncpy(req.scene, "path", sizeof(req.scene) - 1);
    req.sequence[0] = '\0';
    return copilot_audio_enqueue_file(&req);
}

bool copilot_audio_play_scene(const char *scene_id, const char *sequence_id) {
    if (!s_audio_queue) {
        return false;
    }

    audio_req_t req = {};
    req.kind = AUDIO_REQ_FILE;
    copilot_sanitize_audio_id(scene_id, "default", req.scene, sizeof(req.scene));
    copilot_sanitize_audio_id(sequence_id, "1", req.sequence, sizeof(req.sequence));
    req.path[0] = '\0';

    return copilot_audio_enqueue_file(&req);
}

void copilot_audio_stop(void) {
    if (!s_audio_queue) {
        return;
    }

    copilot_audio_next_generation();
    copilot_audio_drop_pending();
    copilot_audio_out_abort(AUDIO_SRC_FILE);
    copilot_audio_out_abort(AUDIO_SRC_TONE);
    s_status.playing_file = false;
    copilot_audio_set_error("");
    ESP_LOGI(TAG, "Audio playback stopped");
}

bool copilot_audio_is_ready(void) {
    return s_audio_queue && copilot_audio_out_is_ready();
}

bool copilot_audio_get_status(copilot_audio_status_t *out_status) {
    if (!out_status) {
        return false;
    }
    if (bsp_sdcard) {
        s_sd_mounted = true;
    }
    s_status.ready = copilot_audio_is_ready();
    s_status.sd_mounted = s_sd_mounted;
    *out_status = s_status;
    return true;
}
