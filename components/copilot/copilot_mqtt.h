#ifndef COPILOT_MQTT_H
#define COPILOT_MQTT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*copilot_mqtt_cmd_cb)(const char *topic, const char *payload, int payload_len);

void copilot_mqtt_start(copilot_mqtt_cmd_cb cb);
void copilot_mqtt_publish(const char *topic_suffix, const char *payload);
const char *copilot_mqtt_cmd_topic(void);
const char *copilot_mqtt_device_id(void);

#ifdef __cplusplus
}
#endif

#endif
