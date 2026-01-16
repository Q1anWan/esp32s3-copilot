#include "copilot_mqtt.h"
#include "copilot_voice.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

static const char *TAG = "copilot_mqtt";

// Conditional logging
#if CONFIG_COPILOT_LOG_MQTT
#define LOGI_MQTT(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#else
#define LOGI_MQTT(fmt, ...) do {} while(0)
#endif

static esp_mqtt_client_handle_t s_mqtt = nullptr;
static bool s_mqtt_started = false;
static bool s_mqtt_connected = false;
static bool s_wifi_connected = false;
static bool s_wifi_started = false;
static int s_retry_num = 0;
static copilot_mqtt_cmd_cb s_cmd_cb = nullptr;
static esp_netif_t *s_netif_sta = nullptr;

static char s_device_id[32];
static char s_topic_cmd[96];
static char s_topic_status[96];

static const int kMaxRetry = 6;

static void copilot_mqtt_stop_client(const char *reason) {
    if (!s_mqtt || !s_mqtt_started) {
        s_mqtt_connected = false;
        return;
    }
    ESP_LOGW(TAG, "MQTT stop (%s)", reason ? reason : "unknown");
    esp_mqtt_client_stop(s_mqtt);
    s_mqtt_started = false;
    s_mqtt_connected = false;
}

static void copilot_build_device_id(void) {
    if (CONFIG_COPILOT_DEVICE_ID[0] != '\0') {
        strncpy(s_device_id, CONFIG_COPILOT_DEVICE_ID, sizeof(s_device_id) - 1);
        s_device_id[sizeof(s_device_id) - 1] = '\0';
        LOGI_MQTT( "Device ID (config): %s", s_device_id);
        return;
    }

    uint8_t mac[6] = {};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_device_id, sizeof(s_device_id), "s3_%02X%02X%02X", mac[3], mac[4], mac[5]);
    LOGI_MQTT( "Device ID (mac): %s", s_device_id);
}

static void copilot_build_topics(void) {
    const char *prefix = CONFIG_COPILOT_MQTT_TOPIC_PREFIX;
    if (!prefix || prefix[0] == '\0') {
        prefix = "copilot";
    }
    snprintf(s_topic_cmd, sizeof(s_topic_cmd), "%s/%s/cmd", prefix, s_device_id);
    snprintf(s_topic_status, sizeof(s_topic_status), "%s/%s/status", prefix, s_device_id);
    LOGI_MQTT( "MQTT cmd topic: %s", s_topic_cmd);
    LOGI_MQTT( "MQTT status topic: %s", s_topic_status);
}

static void copilot_mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    (void)handler_args;
    (void)base;
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            LOGI_MQTT( "MQTT connected, subscribing: %s", s_topic_cmd);
            s_mqtt_connected = true;
            esp_mqtt_client_subscribe(s_mqtt, s_topic_cmd, 1);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            s_mqtt_connected = false;
            break;
        case MQTT_EVENT_DATA: {
            if (!s_cmd_cb) {
                break;
            }
            LOGI_MQTT( "MQTT data topic_len=%d payload_len=%d", event->topic_len, event->data_len);
            char *topic = (char *)malloc(event->topic_len + 1);
            char *payload = (char *)malloc(event->data_len + 1);
            if (!topic || !payload) {
                free(topic);
                free(payload);
                break;
            }
            memcpy(topic, event->topic, event->topic_len);
            topic[event->topic_len] = '\0';
            memcpy(payload, event->data, event->data_len);
            payload[event->data_len] = '\0';
            s_cmd_cb(topic, payload, event->data_len);
            free(topic);
            free(payload);
            break;
        }
        default:
            break;
    }
}

static void copilot_mqtt_start_client(void) {
    if (s_mqtt_started) {
        return;
    }

    if (CONFIG_COPILOT_MQTT_BROKER_URI[0] == '\0') {
        ESP_LOGW(TAG, "MQTT broker URI not set");
        return;
    }
    if (!s_wifi_connected) {
        ESP_LOGW(TAG, "WiFi not connected, skip MQTT start");
        return;
    }

    LOGI_MQTT( "MQTT broker: %s", CONFIG_COPILOT_MQTT_BROKER_URI);
    copilot_build_device_id();
    copilot_build_topics();

    if (s_mqtt) {
        LOGI_MQTT( "MQTT restart");
        esp_mqtt_client_start(s_mqtt);
        s_mqtt_started = true;
        return;
    }

    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = CONFIG_COPILOT_MQTT_BROKER_URI;
    mqtt_cfg.credentials.client_id = s_device_id;

    s_mqtt = esp_mqtt_client_init(&mqtt_cfg);
    if (!s_mqtt) {
        ESP_LOGE(TAG, "Failed to init MQTT client");
        return;
    }

    esp_mqtt_client_register_event(s_mqtt, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, copilot_mqtt_event_handler, nullptr);
    esp_mqtt_client_start(s_mqtt);
    s_mqtt_started = true;
}

static void copilot_wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    (void)arg;
    (void)event_base;
    (void)event_data;

    if (event_id == WIFI_EVENT_STA_START) {
        LOGI_MQTT( "WiFi start, connecting...");
        esp_wifi_connect();
    } else if (event_id == WIFI_EVENT_STA_CONNECTED) {
        LOGI_MQTT( "WiFi connected, waiting for IP...");
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "WiFi disconnected, reason=%d", disc ? disc->reason : -1);
        s_wifi_connected = false;
        copilot_mqtt_stop_client("wifi_down");
        if (s_retry_num < kMaxRetry) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "Retry WiFi (%d/%d)", s_retry_num, kMaxRetry);
        } else {
            ESP_LOGE(TAG, "WiFi connect failed");
        }
    }
}

// Timer callback to start voice session (runs in timer task with adequate stack)
#if CONFIG_COPILOT_VOICE_ENABLE && !CONFIG_COPILOT_VOICE_MODE_LOOPBACK
static void voice_session_timer_cb(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "Starting voice streaming session...");
    if (copilot_voice_start_session()) {
        ESP_LOGI(TAG, "Voice streaming session started");
    } else {
        ESP_LOGW(TAG, "Voice streaming session failed to start");
    }
}
#endif

static void copilot_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    (void)arg;
    (void)event_base;
    (void)event_id;
    ip_event_got_ip_t *ip_event = (ip_event_got_ip_t *)event_data;

    s_retry_num = 0;
    s_wifi_connected = true;
    if (ip_event) {
        LOGI_MQTT( "Got IP: " IPSTR, IP2STR(&ip_event->ip_info.ip));
    }
    copilot_mqtt_start_client();

    // Defer voice session start using timer (event handler stack too small)
#if CONFIG_COPILOT_VOICE_ENABLE && !CONFIG_COPILOT_VOICE_MODE_LOOPBACK
    const esp_timer_create_args_t timer_args = {
        .callback = voice_session_timer_cb,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "voice_start",
        .skip_unhandled_events = false,
    };
    esp_timer_handle_t timer;
    if (esp_timer_create(&timer_args, &timer) == ESP_OK) {
        // Start after 100ms delay
        esp_timer_start_once(timer, 100 * 1000);
    }
#endif
}

static void copilot_wifi_init(void) {
    if (CONFIG_COPILOT_WIFI_SSID[0] == '\0') {
        ESP_LOGW(TAG, "WiFi SSID not set, skip WiFi/MQTT (set in menuconfig: Copilot -> WiFi SSID)");
        return;
    }
    if (s_wifi_started) {
        LOGI_MQTT( "WiFi already started");
        return;
    }
    LOGI_MQTT( "WiFi SSID: %s", CONFIG_COPILOT_WIFI_SSID);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_netif_init failed: %d", err);
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "event loop create failed: %d", err);
    }

    if (!s_netif_sta) {
        s_netif_sta = esp_netif_create_default_wifi_sta();
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err == ESP_ERR_NO_MEM) {
        ESP_LOGE(TAG, "esp_wifi_init failed: no memory (try freeing internal RAM before WiFi init)");
        return;
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &copilot_wifi_event_handler, nullptr, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &copilot_ip_event_handler, nullptr, nullptr));

    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid, CONFIG_COPILOT_WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    wifi_config.sta.ssid[sizeof(wifi_config.sta.ssid) - 1] = '\0';
    strncpy((char *)wifi_config.sta.password, CONFIG_COPILOT_WIFI_PASSWORD, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.password[sizeof(wifi_config.sta.password) - 1] = '\0';

    if (CONFIG_COPILOT_WIFI_PASSWORD[0] == '\0') {
        wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    } else {
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    }
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
    s_wifi_started = true;
    LOGI_MQTT( "WiFi init done, connecting...");
}

void copilot_mqtt_start(copilot_mqtt_cmd_cb cb) {
    s_cmd_cb = cb;
    LOGI_MQTT( "MQTT start (callback=%p)", cb);
    copilot_wifi_init();
}

void copilot_mqtt_publish(const char *topic_suffix, const char *payload) {
    if (!s_mqtt || !payload || !topic_suffix) {
        return;
    }
    if (!s_wifi_connected || !s_mqtt_connected) {
        ESP_LOGW(TAG, "MQTT publish skipped (wifi=%d mqtt=%d)", s_wifi_connected ? 1 : 0, s_mqtt_connected ? 1 : 0);
        return;
    }
    char topic[128];
    if (s_device_id[0] == '\0') {
        copilot_build_device_id();
    }
    const char *prefix = CONFIG_COPILOT_MQTT_TOPIC_PREFIX;
    if (!prefix || prefix[0] == '\0') {
        prefix = "copilot";
    }
    snprintf(topic, sizeof(topic), "%s/%s/%s", prefix, s_device_id, topic_suffix);
    LOGI_MQTT( "MQTT publish: %s (%d bytes)", topic, (int)strlen(payload));
    esp_mqtt_client_publish(s_mqtt, topic, payload, 0, 1, 0);
}

const char *copilot_mqtt_cmd_topic(void) {
    return s_topic_cmd;
}

const char *copilot_mqtt_device_id(void) {
    return s_device_id;
}
