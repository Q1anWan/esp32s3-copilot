idf_component_register(
  SRCS
    "copilot_face_data.cpp"
    "copilot_app.cpp"
    "copilot_ui.cpp"
    "copilot_audio.cpp"
    "copilot_mqtt.cpp"
    "copilot_perf.cpp"
  INCLUDE_DIRS "."
  PRIV_REQUIRES lvgl esp_wifi esp_event esp_netif nvs_flash mqtt cjson esp_timer
  REQUIRES esp32_s3_touch_amoled_1_75 esp_codec_dev
)
