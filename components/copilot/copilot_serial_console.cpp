#include "copilot_serial_console.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "copilot_app.h"
#include "copilot_mqtt.h"

static const char *TAG = "copilot_serial";
static TaskHandle_t s_serial_task = nullptr;

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

static void print_status(void) {
    char status[768];
    if (copilot_app_format_status(status, sizeof(status))) {
        printf("COPILOT_STATUS %s\n", status);
    } else {
        printf("COPILOT_ERROR status_failed\n");
    }
    fflush(stdout);
}

static void set_debug(bool enabled) {
    esp_log_level_set("copilot_app", enabled ? ESP_LOG_DEBUG : ESP_LOG_INFO);
    esp_log_level_set("copilot_audio", enabled ? ESP_LOG_DEBUG : ESP_LOG_INFO);
    esp_log_level_set("copilot_tcp", enabled ? ESP_LOG_DEBUG : ESP_LOG_INFO);
    esp_log_level_set("copilot_mqtt", enabled ? ESP_LOG_DEBUG : ESP_LOG_INFO);
    esp_log_level_set("audio_out", enabled ? ESP_LOG_DEBUG : ESP_LOG_INFO);
    printf("COPILOT_DEBUG %s\n", enabled ? "on" : "off");
    fflush(stdout);
}

static void handle_text_command(char *line) {
    trim_line(line);
    if (line[0] == '\0') {
        return;
    }

    if (line[0] == '{') {
        copilot_app_handle_command(line, (int)strlen(line));
        printf("COPILOT_OK json_accepted\n");
        fflush(stdout);
        return;
    }

    if (starts_with_ci(line, "status")) {
        print_status();
        return;
    }

    if (starts_with_ci(line, "help")) {
        printf("COPILOT_HELP commands: status | wifi <ssid> <password> | play <scene> <seq> | debug on/off | reboot | JSON-line\n");
        fflush(stdout);
        return;
    }

    if (starts_with_ci(line, "debug on")) {
        set_debug(true);
        return;
    }
    if (starts_with_ci(line, "debug off")) {
        set_debug(false);
        return;
    }

    if (starts_with_ci(line, "reboot")) {
        printf("COPILOT_OK rebooting\n");
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
    }

    if (starts_with_ci(line, "wifi ")) {
        char ssid[33] = {};
        char password[65] = {};
        int count = sscanf(line, "%*s %32s %64s", ssid, password);
        if (count >= 1 && copilot_mqtt_configure_wifi(ssid, count >= 2 ? password : "")) {
            printf("COPILOT_OK wifi ssid=%s\n", ssid);
        } else {
            printf("COPILOT_ERROR wifi_usage\n");
        }
        fflush(stdout);
        return;
    }

    if (starts_with_ci(line, "play ")) {
        char scene[32] = {};
        unsigned seq = 0;
        if (sscanf(line, "%*s %31s %u", scene, &seq) == 2 && seq <= 65535) {
            char payload[128];
            snprintf(payload, sizeof(payload),
                     "{\"type\":\"play\",\"scene\":\"%s\",\"seq\":%u}",
                     scene, seq);
            copilot_app_handle_command(payload, (int)strlen(payload));
            printf("COPILOT_OK play scene=%s seq=%u\n", scene, seq);
        } else {
            printf("COPILOT_ERROR play_usage\n");
        }
        fflush(stdout);
        return;
    }

    printf("COPILOT_ERROR unknown_command\n");
    fflush(stdout);
}

static void serial_task(void *arg) {
    (void)arg;
    setvbuf(stdin, nullptr, _IONBF, 0);
    setvbuf(stdout, nullptr, _IONBF, 0);

    printf("COPILOT_SERIAL ready\n");
    fflush(stdout);

    char line[512];
    while (true) {
        if (fgets(line, sizeof(line), stdin) != nullptr) {
            handle_text_command(line);
        } else {
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

void copilot_serial_console_start(void) {
    if (s_serial_task) {
        return;
    }
    BaseType_t ok = xTaskCreate(serial_task, "copilot_serial", 4096, nullptr, 2, &s_serial_task);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create serial console task");
    }
}
