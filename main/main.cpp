#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "esp_lvgl_port.h"

#include "copilot_app.h"
#include "copilot_audio.h"
#include "copilot_serial_console.h"
#include "copilot_ui.h"

#if CONFIG_COPILOT_SERVO_ENABLE
#include "copilot_servo.h"
#endif
#include "copilot_axp2101.h"

static const char *TAG = "main";
static constexpr uint32_t kDisplayDrawBufferLines = 12;

static int copilot_normalize_core(int core) {
    if (core < 0) {
        return -1;
    }
    if (core >= (int)configNUM_CORES) {
        return -1;
    }
    return core;
}

#if CONFIG_BSP_DISPLAY_TOUCH_ENABLE
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
#endif

extern "C" void app_main(void) {
    copilot_serial_console_prepare();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

#if CONFIG_COPILOT_SERIAL_CONSOLE_ENABLE
    copilot_serial_console_start();
#endif

    bsp_display_cfg_t disp_cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_H_RES * kDisplayDrawBufferLines,
        .trans_size = 0,
        .double_buffer = false,
        .flags = {
            .buff_dma = true,
            .buff_spiram = false,
        },
    };
    // Keep LVGL stack modest; touch is optional and disabled for display/audio builds.
    disp_cfg.lvgl_port_cfg.task_stack = 6144;
    disp_cfg.lvgl_port_cfg.task_stack_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
    disp_cfg.lvgl_port_cfg.task_priority = 15;
    disp_cfg.lvgl_port_cfg.task_affinity = copilot_normalize_core(CONFIG_COPILOT_UI_CORE);
    ESP_LOGI(TAG, "LVGL task affinity=%d", disp_cfg.lvgl_port_cfg.task_affinity);
    lv_display_t *disp = bsp_display_start_with_config(&disp_cfg);
    if (!disp) {
        ESP_LOGE(TAG, "Display init failed");
        return;
    }

#if CONFIG_COPILOT_DISPLAY_ROTATE_180
    lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_180);
    ESP_LOGI(TAG, "Display rotated 180 degrees");
#endif

    copilot_app_init();

#if CONFIG_COPILOT_SERVO_ENABLE
    copilot_servo_init();
#endif

    // Init AXP2101 PMU for battery monitoring
    if (copilot_axp2101_init() == ESP_OK) {
        ESP_LOGI(TAG, "AXP2101 PMU initialized");
    } else {
        ESP_LOGW(TAG, "AXP2101 PMU not available (battery status disabled)");
    }

    if (bsp_display_lock(0)) {
        copilot_ui_init(lv_scr_act());

#if CONFIG_BSP_DISPLAY_TOUCH_ENABLE
        lv_obj_t *touch_layer = lv_obj_create(lv_scr_act());
        lv_obj_set_size(touch_layer, BSP_LCD_H_RES, BSP_LCD_V_RES);
        lv_obj_set_pos(touch_layer, 0, 0);
        lv_obj_set_style_bg_opa(touch_layer, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(touch_layer, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_clear_flag(touch_layer, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(touch_layer, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(touch_layer, copilot_touch_event_cb, LV_EVENT_PRESSED, nullptr);
#else
        ESP_LOGI(TAG, "Touch UI disabled");
#endif

#if CONFIG_COPILOT_STARTUP_SELF_TEST
        ESP_LOGI(TAG, "Startup self-test: speaking head + chime");
        copilot_ui_set_expression(COPILOT_EXPR_SPEAKING, 1800);
        copilot_audio_play("chime");
#endif

        bsp_display_unlock();
    } else {
        ESP_LOGE(TAG, "Failed to acquire LVGL lock");
    }

}
