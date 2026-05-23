#include "copilot_serial_console.h"

#include <stdio.h>
#include <string.h>
#include <fcntl.h>

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#endif

#include "copilot_app.h"
#include "copilot_mqtt.h"

static const char *TAG = "copilot_serial";
static TaskHandle_t s_serial_task = nullptr;
static bool s_stdio_prepared = false;
static char s_line_buf[512];
static char s_status_buf[1024];

enum class config_cmd_type_t {
    WIFI,
    MQTT,
};

struct config_cmd_request_t {
    config_cmd_type_t type;
    char arg1[128];
    char arg2[65];
    bool ok;
    TaskHandle_t waiter;
};

void copilot_serial_console_prepare(void) {
    if (s_stdio_prepared) {
        return;
    }
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_LF);
    usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_LF);
    fcntl(fileno(stdout), F_SETFL, 0);
    fcntl(fileno(stdin), F_SETFL, 0);
    if (!usb_serial_jtag_is_driver_installed()) {
        usb_serial_jtag_driver_config_t jtag_config = {
            .tx_buffer_size = 1024,
            .rx_buffer_size = 1024,
        };
        esp_err_t err = usb_serial_jtag_driver_install(&jtag_config);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "USB serial JTAG driver install failed: %s", esp_err_to_name(err));
        }
    }
    usb_serial_jtag_vfs_use_driver();
#endif
    s_stdio_prepared = true;
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

static bool is_audio_id_token_char(char c) {
    return (c >= 'a' && c <= 'z') ||
           (c >= 'A' && c <= 'Z') ||
           (c >= '0' && c <= '9') ||
           c == '_' || c == '-';
}

static bool is_valid_audio_id_token(const char *token) {
    if (!token || token[0] == '\0') {
        return false;
    }
    for (size_t i = 0; token[i] != '\0'; ++i) {
        if (!is_audio_id_token_char(token[i])) {
            return false;
        }
    }
    return true;
}

static void print_status(void) {
    if (copilot_app_format_status(s_status_buf, sizeof(s_status_buf))) {
        printf("COPILOT_STATUS %s\n", s_status_buf);
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

static void config_cmd_task(void *arg) {
    config_cmd_request_t *req = static_cast<config_cmd_request_t *>(arg);
    if (!req) {
        vTaskDelete(nullptr);
        return;
    }
    if (req->type == config_cmd_type_t::WIFI) {
        req->ok = copilot_mqtt_configure_wifi(req->arg1, req->arg2);
    } else if (req->type == config_cmd_type_t::MQTT) {
        req->ok = copilot_mqtt_configure_broker(req->arg1);
    }
    if (req->waiter) {
        xTaskNotifyGive(req->waiter);
    }
    vTaskDelete(nullptr);
}

static bool run_config_cmd_internal(config_cmd_type_t type, const char *arg1, const char *arg2) {
    config_cmd_request_t req = {};
    req.type = type;
    strncpy(req.arg1, arg1 ? arg1 : "", sizeof(req.arg1) - 1);
    strncpy(req.arg2, arg2 ? arg2 : "", sizeof(req.arg2) - 1);
    req.waiter = xTaskGetCurrentTaskHandle();

    TaskHandle_t task = nullptr;
    BaseType_t ok = xTaskCreateWithCaps(config_cmd_task, "serial_cfg", 4096, &req, 4, &task,
                                        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create internal config task");
        return false;
    }
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(15000)) == 0) {
        ESP_LOGE(TAG, "Config command timed out");
        return false;
    }
    return req.ok;
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
        printf("COPILOT_HELP commands: status | wifi <ssid> <password> | mqtt <mqtt://host:port> | play <scene> <seq> | debug on/off | reboot | JSON-line\n");
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
        if (count >= 1 && run_config_cmd_internal(config_cmd_type_t::WIFI, ssid, count >= 2 ? password : "")) {
            printf("COPILOT_OK wifi ssid=%s\n", ssid);
        } else {
            printf("COPILOT_ERROR wifi_usage\n");
        }
        fflush(stdout);
        return;
    }

    if (starts_with_ci(line, "mqtt ")) {
        char broker_uri[128] = {};
        int count = sscanf(line, "%*s %127s", broker_uri);
        if (count == 1 && run_config_cmd_internal(config_cmd_type_t::MQTT, broker_uri, "")) {
            printf("COPILOT_OK mqtt broker=%s\n", broker_uri);
        } else {
            printf("COPILOT_ERROR mqtt_usage\n");
        }
        fflush(stdout);
        return;
    }

    if (starts_with_ci(line, "play ")) {
        char scene[32] = {};
        char seq[96] = {};
        if (sscanf(line, "%*s %31s %95s", scene, seq) == 2 &&
            is_valid_audio_id_token(scene) && is_valid_audio_id_token(seq)) {
            char payload[192];
            snprintf(payload, sizeof(payload),
                     "{\"type\":\"play\",\"scene\":\"%s\",\"seq\":\"%s\"}",
                     scene, seq);
            copilot_app_handle_command(payload, (int)strlen(payload));
            printf("COPILOT_OK play scene=%s seq=%s\n", scene, seq);
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
    copilot_serial_console_prepare();
    setvbuf(stdin, nullptr, _IONBF, 0);
    setvbuf(stdout, nullptr, _IONBF, 0);

    printf("COPILOT_SERIAL ready\n");
    fflush(stdout);

    while (true) {
        if (fgets(s_line_buf, sizeof(s_line_buf), stdin) != nullptr) {
            handle_text_command(s_line_buf);
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
    BaseType_t ok = xTaskCreateWithCaps(serial_task, "copilot_serial", 4096, nullptr, 2,
                                        &s_serial_task, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ok != pdPASS) {
        ok = xTaskCreate(serial_task, "copilot_serial", 4096, nullptr, 2, &s_serial_task);
    }
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create serial console task");
    }
}
