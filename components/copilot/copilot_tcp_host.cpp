#include "copilot_tcp_host.h"

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"

#include "copilot_app.h"
#include "copilot_mqtt.h"

static const char *TAG = "copilot_tcp";

static TaskHandle_t s_tcp_task = nullptr;
static copilot_tcp_cmd_cb s_cmd_cb = nullptr;

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

static void handle_play_text(int fd, const char *line) {
    char scene[32] = {};
    unsigned seq = 0;
    if (sscanf(line, "%*s %31s %u", scene, &seq) != 2 || seq > 65535) {
        tcp_send_line(fd, "{\"ok\":false,\"error\":\"usage: PLAY <scene> <seq>\"}");
        return;
    }

    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"type\":\"play\",\"scene\":\"%s\",\"seq\":%u}",
             scene, seq);
    if (s_cmd_cb) {
        s_cmd_cb(payload, (int)strlen(payload));
    }

    char ack[128];
    snprintf(ack, sizeof(ack),
             "{\"ok\":true,\"cmd\":\"play\",\"scene\":\"%s\",\"seq\":%u}",
             scene, seq);
    tcp_send_line(fd, ack);
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
        char status[768];
        if (copilot_app_format_status(status, sizeof(status))) {
            tcp_send_line(fd, status);
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

    tcp_send_line(fd, "{\"ok\":false,\"error\":\"unknown_command\"}");
}

static void handle_client(int fd) {
    tcp_send_line(fd, "{\"ok\":true,\"device\":\"s3_copilot\",\"commands\":[\"PLAY scene seq\",\"STATUS\",\"PING\", \"json\"]}");

    char rx[128];
    char line[512];
    size_t line_len = 0;
    while (true) {
        int got = recv(fd, rx, sizeof(rx), 0);
        if (got <= 0) {
            break;
        }
        for (int i = 0; i < got; ++i) {
            char c = rx[i];
            if (c == '\n') {
                line[line_len] = '\0';
                handle_tcp_line(fd, line);
                line_len = 0;
            } else if (line_len + 1 < sizeof(line)) {
                line[line_len++] = c;
            } else {
                line_len = 0;
                tcp_send_line(fd, "{\"ok\":false,\"error\":\"line_too_long\"}");
            }
        }
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

    BaseType_t ok = xTaskCreate(tcp_task, "copilot_tcp", 4096, nullptr, 4, &s_tcp_task);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create TCP host task");
    }
}
