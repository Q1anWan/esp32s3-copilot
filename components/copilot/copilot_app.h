#ifndef COPILOT_APP_H
#define COPILOT_APP_H

#include <stdbool.h>
#include <stdint.h>
#include "lvgl.h"

void copilot_app_network_start(void);
void copilot_app_init(void);
void copilot_app_ui_init(lv_obj_t *root);
void copilot_app_on_touch(uint16_t x, uint16_t y);
void copilot_app_handle_command(const char *payload, int payload_len);
bool copilot_app_format_status(char *out, unsigned out_len);

#endif
