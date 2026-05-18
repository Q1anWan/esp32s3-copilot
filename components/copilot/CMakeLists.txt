idf_component_register(
  SRCS
    "copilot_face_data.cpp"
    "copilot_app.cpp"
    "copilot_ui.cpp"
    "copilot_voice_ui.cpp"
    "copilot_audio.cpp"
    "copilot_mqtt.cpp"
    "copilot_tcp_host.cpp"
    "copilot_serial_console.cpp"
    "copilot_perf.cpp"
    "copilot_imu.cpp"
    "copilot_servo.cpp"
    "copilot_axp2101.cpp"
  INCLUDE_DIRS "." "../XPowersLib/src" "../XPowersLib/src/REG"
  PRIV_REQUIRES lvgl esp_wifi esp_event esp_netif nvs_flash mqtt cjson esp_timer driver lwip waveshare__qmi8658
  REQUIRES esp32_s3_touch_amoled_1_75 esp_codec_dev copilot_voice XPowersLib
)

target_compile_options(__idf_copilot PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated-enum-enum-conversion>)
