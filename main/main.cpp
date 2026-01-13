#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "esp_lvgl_port.h"

#include "copilot_app.h"
#include "copilot_ui.h"

static const char *TAG = "main";

static int copilot_normalize_core(int core) {
    if (core < 0) {
        return -1;
    }
    if (core >= (int)configNUM_CORES) {
        return -1;
    }
    return core;
}

static void copilot_touch_event_cb(lv_event_t *e) {
    (void)e;
    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) {
        return;
    }
    lv_point_t point = {};
    lv_indev_get_point(indev, &point);
    if (point.x < 0 || point.y < 0) {
        return;
    }
    copilot_ui_on_touch((uint16_t)point.x, (uint16_t)point.y);
}

extern "C" void app_main(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    bsp_display_cfg_t disp_cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .trans_size = 0,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
            .buff_dma = false,
            .buff_spiram = true,
        },
    };
    // Optimize LVGL task: reduce from 7168 to 6144 (touch read needs extra stack)
    disp_cfg.lvgl_port_cfg.task_stack = 6144;
    disp_cfg.lvgl_port_cfg.task_affinity = copilot_normalize_core(CONFIG_COPILOT_UI_CORE);
    ESP_LOGI(TAG, "LVGL task affinity=%d", disp_cfg.lvgl_port_cfg.task_affinity);
    lv_display_t *disp = bsp_display_start_with_config(&disp_cfg);
    if (!disp) {
        ESP_LOGE(TAG, "Display init failed");
        return;
    }

    copilot_app_init();

    if (bsp_display_lock(0)) {
        copilot_ui_init(lv_scr_act());

        lv_obj_t *touch_layer = lv_obj_create(lv_scr_act());
        lv_obj_set_size(touch_layer, BSP_LCD_H_RES, BSP_LCD_V_RES);
        lv_obj_set_pos(touch_layer, 0, 0);
        lv_obj_set_style_bg_opa(touch_layer, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(touch_layer, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_clear_flag(touch_layer, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(touch_layer, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(touch_layer, copilot_touch_event_cb, LV_EVENT_PRESSED, nullptr);

        bsp_display_unlock();
    } else {
        ESP_LOGE(TAG, "Failed to acquire LVGL lock");
    }
}
