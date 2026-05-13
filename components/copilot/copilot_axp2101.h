#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Battery/power status snapshot
typedef struct {
    int      battery_percent;    // 0-100, -1 if no battery
    uint16_t batt_voltage_mv;    // mV
    uint16_t vbus_voltage_mv;    // mV, 0 if USB not connected
    uint16_t system_voltage_mv;  // mV
    float    temperature_c;      // PMU internal temp (Celsius)
    bool     is_charging;
    bool     is_vbus_in;
    bool     is_battery_connect;
    bool     is_discharge;
} copilot_power_status_t;

// Init AXP2101 on the shared BSP I2C bus (GPIO14/15).
// Must be called after bsp_i2c_init().
// Returns ESP_OK on success.
int copilot_axp2101_init(void);

// Get current power status. Returns true if PMU is initialized.
bool copilot_axp2101_get_status(copilot_power_status_t *status);

// Check if PMU is initialized and ready
bool copilot_axp2101_is_ready(void);

#ifdef __cplusplus
}
#endif
