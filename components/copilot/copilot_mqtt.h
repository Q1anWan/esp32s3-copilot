#ifndef COPILOT_MQTT_H
#define COPILOT_MQTT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>

typedef void (*copilot_mqtt_cmd_cb)(const char *topic, const char *payload, int payload_len);

typedef struct {
    bool wifi_started;
    bool wifi_connected;
    bool mqtt_started;
    bool mqtt_connected;
    char ssid[33];
    char ip[16];
    char mqtt_broker_uri[128];
    char device_id[32];
    char tcp_host[32];
} copilot_network_status_t;

void copilot_mqtt_start(copilot_mqtt_cmd_cb cb);
void copilot_mqtt_notify_voice_ready(void);
void copilot_mqtt_publish(const char *topic_suffix, const char *payload);
bool copilot_mqtt_configure_broker(const char *broker_uri);
bool copilot_mqtt_configure_wifi(const char *ssid, const char *password);
bool copilot_mqtt_get_status(copilot_network_status_t *out_status);
bool copilot_mqtt_wifi_is_connected(void);
const char *copilot_mqtt_ip_string(void);
const char *copilot_mqtt_cmd_topic(void);
const char *copilot_mqtt_device_id(void);

#ifdef __cplusplus
}
#endif

#endif
