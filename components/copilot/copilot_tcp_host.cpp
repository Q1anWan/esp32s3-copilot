#include "copilot_tcp_host.h"

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"

#include "copilot_app.h"
#include "copilot_mqtt.h"

static const char *TAG = "copilot_tcp";

#ifndef CONFIG_COPILOT_TCP_HOST_PORT
#define CONFIG_COPILOT_TCP_HOST_PORT 7777
#endif

static TaskHandle_t s_tcp_task = nullptr;
static copilot_tcp_cmd_cb s_cmd_cb = nullptr;
static char s_tcp_rx_buf[128];
static char s_tcp_line_buf[512];
static char s_tcp_status_buf[1024];
static char s_tcp_payload_buf[128];
static char s_tcp_ack_buf[128];
#if CONFIG_COPILOT_TCP_AUDIO_ID_ENABLE
static bool s_audio_id_trigger_latched = false;
static char s_audio_id_last_scene[32];
static uint16_t s_audio_id_last_seq = 0;
static TickType_t s_audio_id_last_tick = 0;
#endif

#if CONFIG_COPILOT_TCP_AUDIO_ID_ENABLE
struct audio_id_packet_t {
    double trigger;
    char scene[32];
    uint16_t seq;
};
#endif

static void tcp_send_line(int fd, const char *line) {
    if (fd < 0 || !line) {
        return;
    }
    send(fd, line, strlen(line), 0);
    send(fd, "\n", 1, 0);
}

static void trim_line(char *line) {
    if (!line) {
        return;
    }
    size_t len = strlen(line);
    while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r' ||
                       line[len - 1] == ' ' || line[len - 1] == '\t')) {
        line[--len] = '\0';
    }
    char *start = line;
    while (*start == ' ' || *start == '\t') {
        ++start;
    }
    if (start != line) {
        memmove(line, start, strlen(start) + 1);
    }
}

static bool starts_with_ci(const char *s, const char *prefix) {
    if (!s || !prefix) {
        return false;
    }
    while (*prefix) {
        char a = *s++;
        char b = *prefix++;
        if (a >= 'a' && a <= 'z') {
            a = (char)(a - 'a' + 'A');
        }
        if (b >= 'a' && b <= 'z') {
            b = (char)(b - 'a' + 'A');
        }
        if (a != b) {
            return false;
        }
    }
    return true;
}

#if CONFIG_COPILOT_TCP_AUDIO_ID_ENABLE
static bool is_token_separator(char c) {
    return c == '\0' || c == ',' || c == '\t' || c == ' ' || c == '\r' || c == '\n';
}

static bool is_scene_token_char(char c) {
    return (c >= 'a' && c <= 'z') ||
           (c >= 'A' && c <= 'Z') ||
           (c >= '0' && c <= '9') ||
           c == '_' || c == '-';
}

static bool is_valid_scene_token(const char *token) {
    if (!token || token[0] == '\0') {
        return false;
    }
    for (size_t i = 0; token[i] != '\0'; ++i) {
        if (!is_scene_token_char(token[i])) {
            return false;
        }
    }
    return true;
}

static bool parse_u32_token(const char *token, uint32_t *out) {
    if (!token || token[0] == '\0' || !out) {
        return false;
    }
    char *end = nullptr;
    errno = 0;
    double value = strtod(token, &end);
    if (end == token || *end != '\0' || errno == ERANGE ||
        value < 0.0 || value > (double)UINT32_MAX) {
        return false;
    }
    uint32_t rounded = (uint32_t)(value + 0.5);
    double delta = value - (double)rounded;
    if (delta < -0.0001 || delta > 0.0001) {
        return false;
    }
    *out = rounded;
    return true;
}

static bool parse_double_token(const char *token, double *out) {
    if (!token || token[0] == '\0' || !out) {
        return false;
    }
    char *end = nullptr;
    errno = 0;
    double value = strtod(token, &end);
    if (end == token || *end != '\0' || errno == ERANGE) {
        return false;
    }
    *out = value;
    return true;
}

static bool parse_audio_id_packet(const char *line, audio_id_packet_t *out) {
    if (!line || !out) {
        return false;
    }

    char tokens[3][32] = {};
    int count = 0;
    const char *p = line;
    while (*p != '\0' && count < 3) {
        while (*p == ',' || *p == '\t' || *p == ' ') {
            ++p;
        }
        if (*p == '\0') {
            break;
        }

        size_t len = 0;
        while (!is_token_separator(p[len])) {
            if (len + 1 >= sizeof(tokens[count])) {
                return false;
            }
            ++len;
        }
        if (len == 0) {
            return false;
        }
        memcpy(tokens[count], p, len);
        tokens[count][len] = '\0';
        ++count;
        p += len;

        if (!is_token_separator(*p)) {
            return false;
        }
    }

    while (*p == ',' || *p == '\t' || *p == ' ') {
        ++p;
    }
    if (*p != '\0' || count != 3) {
        return false;
    }

    double trigger = 0.0;
    if (!parse_double_token(tokens[0], &trigger)) {
        return false;
    }

    uint32_t seq = 0;
    if (!parse_u32_token(tokens[2], &seq) || seq == 0 || seq > 65535) {
        return false;
    }

    uint32_t numeric_scene = 0;
    if (parse_u32_token(tokens[1], &numeric_scene)) {
        snprintf(out->scene, sizeof(out->scene), "%s%03lu",
                 CONFIG_COPILOT_TCP_AUDIO_ID_NUMERIC_SCENE_PREFIX,
                 (unsigned long)numeric_scene);
    } else {
        if (!is_valid_scene_token(tokens[1])) {
            return false;
        }
        strncpy(out->scene, tokens[1], sizeof(out->scene) - 1);
        out->scene[sizeof(out->scene) - 1] = '\0';
    }
    out->trigger = trigger;
    out->seq = (uint16_t)seq;
    return true;
}

static bool should_accept_audio_id_packet(const audio_id_packet_t *packet) {
    if (!packet) {
        return false;
    }

    bool active = packet->trigger >= 0.5;
    bool rising = active && !s_audio_id_trigger_latched;
    s_audio_id_trigger_latched = active;
    if (!rising) {
        return false;
    }

    TickType_t now = xTaskGetTickCount();
    TickType_t dedupe = pdMS_TO_TICKS(CONFIG_COPILOT_TCP_AUDIO_ID_DEDUPE_MS);
    if (s_audio_id_last_tick != 0 &&
        now - s_audio_id_last_tick < dedupe &&
        s_audio_id_last_seq == packet->seq &&
        strcmp(s_audio_id_last_scene, packet->scene) == 0) {
        return false;
    }
    strncpy(s_audio_id_last_scene, packet->scene, sizeof(s_audio_id_last_scene) - 1);
    s_audio_id_last_scene[sizeof(s_audio_id_last_scene) - 1] = '\0';
    s_audio_id_last_seq = packet->seq;
    s_audio_id_last_tick = now;
    return true;
}

static void handle_audio_id_packet(int fd, const audio_id_packet_t *packet) {
    if (!packet) {
        return;
    }

    bool accepted = should_accept_audio_id_packet(packet);
    if (accepted && s_cmd_cb) {
        snprintf(s_tcp_payload_buf, sizeof(s_tcp_payload_buf),
                 "{\"type\":\"play\",\"scene\":\"%s\",\"seq\":%u,\"message_id\":\"tcp_audio_id\"}",
                 packet->scene, (unsigned)packet->seq);
        s_cmd_cb(s_tcp_payload_buf, (int)strlen(s_tcp_payload_buf));
    }

    const char *state = accepted ? "accepted" : (packet->trigger >= 0.5 ? "held_or_deduped" : "inactive");
    ESP_LOGI(TAG, "TCP audio ID %s trigger=%.3f scene=%s seq=%u",
             state,
             packet->trigger,
             packet->scene,
             (unsigned)packet->seq);
    snprintf(s_tcp_ack_buf, sizeof(s_tcp_ack_buf),
             "{\"ok\":true,\"cmd\":\"audio_id\",\"trigger\":%.3f,\"scene\":\"%s\",\"seq\":%u,\"accepted\":%s}",
             packet->trigger, packet->scene, (unsigned)packet->seq, accepted ? "true" : "false");
    tcp_send_line(fd, s_tcp_ack_buf);
}
#endif

static void handle_play_text(int fd, const char *line) {
    char scene[32] = {};
    unsigned seq = 0;
    if (sscanf(line, "%*s %31s %u", scene, &seq) != 2 || seq > 65535) {
        tcp_send_line(fd, "{\"ok\":false,\"error\":\"usage: PLAY <scene> <seq>\"}");
        return;
    }

    snprintf(s_tcp_payload_buf, sizeof(s_tcp_payload_buf),
             "{\"type\":\"play\",\"scene\":\"%s\",\"seq\":%u}",
             scene, seq);
    if (s_cmd_cb) {
        s_cmd_cb(s_tcp_payload_buf, (int)strlen(s_tcp_payload_buf));
    }

    snprintf(s_tcp_ack_buf, sizeof(s_tcp_ack_buf),
             "{\"ok\":true,\"cmd\":\"play\",\"scene\":\"%s\",\"seq\":%u}",
             scene, seq);
    tcp_send_line(fd, s_tcp_ack_buf);
}

static void handle_tcp_line(int fd, char *line) {
    trim_line(line);
    if (line[0] == '\0') {
        return;
    }

    if (starts_with_ci(line, "PING")) {
        tcp_send_line(fd, "{\"ok\":true,\"pong\":true}");
        return;
    }

    if (starts_with_ci(line, "STATUS")) {
        if (copilot_app_format_status(s_tcp_status_buf, sizeof(s_tcp_status_buf))) {
            tcp_send_line(fd, s_tcp_status_buf);
        } else {
            tcp_send_line(fd, "{\"ok\":false,\"error\":\"status_failed\"}");
        }
        return;
    }

    if (starts_with_ci(line, "PLAY ")) {
        handle_play_text(fd, line);
        return;
    }

    if (line[0] == '{') {
        if (s_cmd_cb) {
            s_cmd_cb(line, (int)strlen(line));
        }
        tcp_send_line(fd, "{\"ok\":true,\"cmd\":\"json_accepted\"}");
        return;
    }

#if CONFIG_COPILOT_TCP_AUDIO_ID_ENABLE
    audio_id_packet_t packet = {};
    if (parse_audio_id_packet(line, &packet)) {
        handle_audio_id_packet(fd, &packet);
        return;
    }
#endif

    tcp_send_line(fd, "{\"ok\":false,\"error\":\"unknown_command\"}");
}

static void handle_client(int fd) {
    tcp_send_line(fd, "{\"ok\":true,\"device\":\"s3_copilot\",\"commands\":[\"PLAY scene seq\",\"trigger scene seq\",\"STATUS\",\"PING\",\"json\"]}");

    size_t line_len = 0;
    while (true) {
        int got = recv(fd, s_tcp_rx_buf, sizeof(s_tcp_rx_buf), 0);
        if (got <= 0) {
            break;
        }
        for (int i = 0; i < got; ++i) {
            char c = s_tcp_rx_buf[i];
            if (c == '\n') {
                s_tcp_line_buf[line_len] = '\0';
                handle_tcp_line(fd, s_tcp_line_buf);
                line_len = 0;
            } else if (line_len + 1 < sizeof(s_tcp_line_buf)) {
                s_tcp_line_buf[line_len++] = c;
            } else {
                line_len = 0;
                tcp_send_line(fd, "{\"ok\":false,\"error\":\"line_too_long\"}");
            }
        }
    }
    if (line_len > 0) {
        s_tcp_line_buf[line_len] = '\0';
        handle_tcp_line(fd, s_tcp_line_buf);
    }
}

static void tcp_task(void *arg) {
    (void)arg;

    while (true) {
        while (!copilot_mqtt_wifi_is_connected()) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (listen_fd < 0) {
            ESP_LOGE(TAG, "socket failed errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        int yes = 1;
        setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

        struct sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(CONFIG_COPILOT_TCP_HOST_PORT);
        if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
            ESP_LOGE(TAG, "bind port %d failed errno=%d", CONFIG_COPILOT_TCP_HOST_PORT, errno);
            close(listen_fd);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        if (listen(listen_fd, 2) != 0) {
            ESP_LOGE(TAG, "listen failed errno=%d", errno);
            close(listen_fd);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        ESP_LOGI(TAG, "TCP host listening on %s:%d",
                 copilot_mqtt_ip_string()[0] ? copilot_mqtt_ip_string() : "0.0.0.0",
                 CONFIG_COPILOT_TCP_HOST_PORT);

        while (copilot_mqtt_wifi_is_connected()) {
            struct sockaddr_in source_addr = {};
            socklen_t addr_len = sizeof(source_addr);
            int client_fd = accept(listen_fd, (struct sockaddr *)&source_addr, &addr_len);
            if (client_fd < 0) {
                ESP_LOGW(TAG, "accept failed errno=%d", errno);
                continue;
            }

            ESP_LOGI(TAG, "TCP client connected: %s", inet_ntoa(source_addr.sin_addr));
            handle_client(client_fd);
            shutdown(client_fd, SHUT_RDWR);
            close(client_fd);
            ESP_LOGI(TAG, "TCP client disconnected");
        }

        close(listen_fd);
    }
}

void copilot_tcp_host_start(copilot_tcp_cmd_cb cb) {
    s_cmd_cb = cb;
    if (s_tcp_task) {
        return;
    }

    BaseType_t ok = xTaskCreateWithCaps(tcp_task, "copilot_tcp", 6144, nullptr, 4, &s_tcp_task,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ok != pdPASS) {
        ESP_LOGW(TAG, "TCP host PSRAM stack alloc failed, fallback to internal");
        ok = xTaskCreate(tcp_task, "copilot_tcp", 6144, nullptr, 4, &s_tcp_task);
    }
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create TCP host task");
    }
}
