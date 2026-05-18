#ifndef COPILOT_TCP_HOST_H
#define COPILOT_TCP_HOST_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*copilot_tcp_cmd_cb)(const char *payload, int payload_len);

void copilot_tcp_host_start(copilot_tcp_cmd_cb cb);

#ifdef __cplusplus
}
#endif

#endif
