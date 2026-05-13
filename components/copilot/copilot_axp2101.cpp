#include "copilot_axp2101.h"

#include <string.h>
#include "esp_log.h"
#include "sdkconfig.h"
#include "bsp/esp-bsp.h"
#include "driver/i2c_master.h"

#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"

static const char *TAG = "axp2101";

static XPowersPMU     *s_pmu = nullptr;
static i2c_master_dev_handle_t s_dev_handle = nullptr;
static bool            s_ready = false;

// --- I2C read/write callbacks using new I2C driver ---

static int axp_i2c_read(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len) {
    if (!s_dev_handle || !data || len == 0) return -1;
    esp_err_t ret = i2c_master_transmit_receive(s_dev_handle, &regAddr, 1, data, len, 50);
    return (ret == ESP_OK) ? 0 : -1;
}

static int axp_i2c_write(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len) {
    if (!s_dev_handle || !data || len == 0) return -1;
    // Write: [regAddr] + data
    uint8_t buf[len + 1];
    buf[0] = regAddr;
    memcpy(buf + 1, data, len);
    esp_err_t ret = i2c_master_transmit(s_dev_handle, buf, len + 1, 50);
    return (ret == ESP_OK) ? 0 : -1;
}

// --- Public API ---

int copilot_axp2101_init(void) {
    if (s_ready) return ESP_OK;

    i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
    if (!bus) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_FAIL;
    }

    // Add AXP2101 device on the shared I2C bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address   = 0x34,
        .scl_speed_hz     = 100000,
    };
    esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add AXP2101 to I2C bus: %d", ret);
        return ret;
    }

    s_pmu = new XPowersPMU();
    if (!s_pmu->begin(0x34, axp_i2c_read, axp_i2c_write)) {
        ESP_LOGE(TAG, "AXP2101 begin() failed");
        delete s_pmu;
        s_pmu = nullptr;
        i2c_master_bus_rm_device(s_dev_handle);
        s_dev_handle = nullptr;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "AXP2101 initialized (chip ID: 0x%02X)", s_pmu->getChipID());

    // Enable measurements
    s_pmu->enableVbusVoltageMeasure();
    s_pmu->enableBattVoltageMeasure();
    s_pmu->enableSystemVoltageMeasure();
    s_pmu->enableTemperatureMeasure();
    s_pmu->disableTSPinMeasure();

    // Set charging parameters
    s_pmu->setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_50MA);
    s_pmu->setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_200MA);
    s_pmu->setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_25MA);
    s_pmu->setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V1);

    s_pmu->clearIrqStatus();

    s_ready = true;
    ESP_LOGI(TAG, "AXP2101 ready. Battery: %d%%, %dmV",
             s_pmu->getBatteryPercent(), s_pmu->getBattVoltage());
    return ESP_OK;
}

bool copilot_axp2101_get_status(copilot_power_status_t *status) {
    if (!s_ready || !s_pmu || !status) return false;

    memset(status, 0, sizeof(*status));
    status->battery_percent   = s_pmu->getBatteryPercent();
    status->batt_voltage_mv   = s_pmu->getBattVoltage();
    status->vbus_voltage_mv   = s_pmu->getVbusVoltage();
    status->system_voltage_mv = s_pmu->getSystemVoltage();
    status->temperature_c     = s_pmu->getTemperature();
    status->is_charging       = s_pmu->isCharging();
    status->is_vbus_in        = s_pmu->isVbusIn();
    status->is_battery_connect = s_pmu->isBatteryConnect();
    status->is_discharge      = s_pmu->isDischarge();
    return true;
}

bool copilot_axp2101_is_ready(void) {
    return s_ready;
}
