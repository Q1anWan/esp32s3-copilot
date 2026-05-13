# GPIO Pinout — ESP32-S3-Touch-AMOLED-1.75

Last verified: 2026-04-30.

All pin definitions originate from
`components/esp32_s3_touch_amoled_1_75/include/bsp/esp32_s3_touch_amoled_1_75.h`
unless noted otherwise.

## Pin Summary Table

| GPIO | Symbol | Function | Peripheral |
|------|--------|----------|------------|
| 0 | `CONFIG_COPILOT_IMU_RESET_GPIO` | IMU quaternion reset button (BOOT) | GPIO input, pull-up, neg-edge ISR |
| 1 | `BSP_SD_CMD` | SD Card command | SDMMC 1-bit |
| 2 | `BSP_SD_CLK` | SD Card clock | SDMMC 1-bit |
| 3 | `BSP_SD_D0` | SD Card data line 0 | SDMMC 1-bit |
| 4 | `BSP_LCD_DATA0` | Display QSPI data 0 | SPI2 (SH8601 AMOLED) |
| 5 | `BSP_LCD_DATA1` | Display QSPI data 1 | SPI2 |
| 6 | `BSP_LCD_DATA2` | Display QSPI data 2 | SPI2 |
| 7 | `BSP_LCD_DATA3` | Display QSPI data 3 | SPI2 |
| 8 | `BSP_I2S_DOUT` | I2S data out (to ES8311 speaker) | I2S1 |
| 9 | `BSP_I2S_SCLK` | I2S bit clock | I2S1 |
| 10 | `BSP_I2S_DSIN` | I2S data in (from ES7210 mic) | I2S1 |
| 12 | `BSP_LCD_CS` | Display QSPI chip select | SPI2 |
| 14 | `BSP_I2C_SCL` | I2C clock — shared bus | I2C1 |
| 15 | `BSP_I2C_SDA` | I2C data — shared bus | I2C1 |
| 17 | `SERVO_PITCH_GPIO` | Pitch servo PWM (UP/DOWN) | LEDC Timer1 CH0 |
| 18 | `SERVO_YAW_GPIO` | Yaw servo PWM (LEFT/RIGHT) | LEDC Timer1 CH1 |
| 38 | `BSP_LCD_PCLK` | Display QSPI pixel clock | SPI2 |
| 39 | `BSP_LCD_RST` | Display panel reset | GPIO output |
| 40 | `BSP_LCD_TOUCH_RST` | Touch controller reset (CST9217) | GPIO output |
| 42 | `BSP_I2S_MCLK` | I2S master clock | I2S1 |
| 43 | UART0 TX | Console UART transmit | UART0 (2,000,000 baud) |
| 44 | UART0 RX | Console UART receive | UART0 |
| 45 | `BSP_I2S_LCLK` | I2S word select (LR clock) | I2S1 |
| 46 | `BSP_POWER_AMP_IO` | ES8311 power amplifier enable | GPIO output |

**24 GPIO pins in use.** GPIOs 0–10, 12, 14–15, 17–18, 38–40, 42–46.

## Not Connected

| Symbol | Function | Note |
|--------|----------|------|
| `BSP_LCD_BACKLIGHT` | Display backlight | `GPIO_NUM_NC` — brightness via LCD cmd 0x51 |
| `BSP_LCD_TOUCH_INT` | Touch interrupt | `GPIO_NUM_NC` — touch is polled over I2C |
| SD D1–D7 | SD Card data lines | `GPIO_NUM_NC` — 1-bit mode only |
| SD CD/WP | SD Card detect / write protect | `GPIO_NUM_NC` |

## Per-Bus Breakdown

### QSPI Display (SPI2_HOST) — SH8601 AMOLED 466×466

| GPIO | Signal |
|------|--------|
| 4 | DATA0 |
| 5 | DATA1 |
| 6 | DATA2 |
| 7 | DATA3 |
| 12 | CS |
| 38 | PCLK |
| 39 | RST |

### Shared I2C Bus (I2C1)

GPIO 14 = SCL, GPIO 15 = SDA. All devices share this bus:

| Device | Address | Function |
|--------|---------|----------|
| ES8311 | default | Speaker codec |
| ES7210 | default | Microphone codec |
| QMI8658 | `0x6B` | IMU sensor |
| CST9217 | — | Touch controller |
| TCA9554 | `0x20` | IO expander |

### I2S Audio (I2S1)

| GPIO | Signal | Direction |
|------|--------|-----------|
| 8 | DOUT | ESP32 → ES8311 (speaker) |
| 9 | SCLK | Bit clock |
| 10 | DSIN | ES7210 → ESP32 (microphone) |
| 42 | MCLK | Master clock |
| 45 | LCLK | Word select / LR clock |
| 46 | PA enable | ES8311 power amp on/off |

### SDMMC (1-bit mode)

| GPIO | Signal |
|------|--------|
| 1 | CMD |
| 2 | CLK |
| 3 | D0 |

### Console UART0

| GPIO | Signal |
|------|--------|
| 43 | TX |
| 44 | RX |

### Servo PWM (LEDC Timer 1, 50 Hz)

| GPIO | Signal | LEDC Channel | Function | Source |
|------|--------|-------------|----------|--------|
| 17 | Pitch | CH0 | Head up/down servo | `components/copilot/Kconfig:374-375` |
| 18 | Yaw | CH1 | Head left/right servo | `components/copilot/Kconfig:381-382` |

14-bit duty resolution (0–16383), 20 ms period (50 Hz). Center pulse = 1500 µs,
range = 500–2500 µs (±45° default). Configurable via Kconfig (angles, pulses,
scale factors, task priority/core).

### Other GPIO

| GPIO | Function | Source |
|------|----------|--------|
| 0 | BOOT button (also IMU quaternion reset) | `components/copilot/Kconfig:172-179` |
