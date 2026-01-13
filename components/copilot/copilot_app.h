#ifndef COPILOT_APP_H
#define COPILOT_APP_H

#include <stdint.h>
#include "lvgl.h"

void copilot_app_init(void);
void copilot_app_ui_init(lv_obj_t *root);
void copilot_app_on_touch(uint16_t x, uint16_t y);

#endif
