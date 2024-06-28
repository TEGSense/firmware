#include "ppg.hpp"

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "MAX30101.hpp"
#include "command.hpp"
#include "flash.hpp"
#include "max3010x.h"

LOG_MODULE_REGISTER(ppg);

static const struct device* dev = NULL;
static struct gpio_dt_spec en_3v3;

MAX30101 ppg = MAX30101();
PPGConfig ppg_config;

bool ppg_read_config() {
    // Read configuration from flash.
    LOG_INF("Reading configuration from flash...");
    uint8_t value;
    if (!flash_read(0, "adc_range", value, 0x20))
        return false;
    ppg_config.adc_range = value;
    if (!flash_read(1, "sample_rate", value, 0x04))
        return false;
    ppg_config.sample_rate = value;
    if (!flash_read(2, "pulse_width", value, 0x02))
        return false;
    ppg_config.pulse_width = value;
    if (!flash_read(3, "sample_avg", value, 0x00))
        return false;
    ppg_config.sample_avg = value;
    if (!flash_read(4, "ir_led_pa", value, 0x1F))
        return false;
    ppg_config.ir_led_pa = value;
    if (!flash_read(5, "red_led_pa", value, 0x1F))
        return false;
    ppg_config.red_led_pa = value;
    if (!flash_read(6, "collection_mode", value, 0x00))
        return false;
    ppg_config.collection_mode = value;

    return true;
}

bool ppg_write_config() {
    // Write configuration to flash.
    LOG_INF("Writing configuration to flash...");
    if (!flash_write(0, "adc_range", ppg_config.adc_range))
        return false;
    if (!flash_write(1, "sample_rate", ppg_config.sample_rate))
        return false;
    if (!flash_write(2, "pulse_width", ppg_config.pulse_width))
        return false;
    if (!flash_write(3, "sample_avg", ppg_config.sample_avg))
        return false;
    if (!flash_write(4, "ir_led_pa", ppg_config.ir_led_pa))
        return false;
    if (!flash_write(5, "red_led_pa", ppg_config.red_led_pa))
        return false;
    if (!flash_write(6, "collection_mode", ppg_config.collection_mode))
        return false;

    return true;
}

bool ppg_configure() {
    // Configure 3.3V enable GPIO.
    LOG_INF("Configuring 3V3 enable GPIO...");
    en_3v3 = GPIO_DT_SPEC_GET(DT_NODELABEL(gpio_en_3v3), gpios);
    if (!device_is_ready(en_3v3.port)) {
        LOG_ERR("3v3 EN GPIO not ready!");
        return false;
    }
    gpio_pin_configure_dt(&en_3v3, GPIO_OUTPUT);
    gpio_pin_set_dt(&en_3v3, 0);

    LOG_INF("Configuring PPG device...");

    // Configure params.
    dev = DEVICE_DT_GET_ANY(maxim_max3010x);

    if (dev == NULL) {
        LOG_ERR("Could not get max3010x device\n");
        return false;
    }
    if (!device_is_ready(dev)) {
        LOG_ERR("max30101 device %s is not ready\n", dev->name);
        return false;
    }

    // Wrap class since the Zephyr driver sucks.
    if (!ppg.begin(dev)) {
        LOG_ERR("Could not begin PPG device...");
        return false;
    }

    // Read configuration from flash.
    if (!ppg_read_config())
        return false;
    ppg.setupSpO2(
        ppg_config.ir_led_pa, ppg_config.red_led_pa, ppg_config.sample_avg,
        ppg_config.sample_rate, ppg_config.pulse_width, ppg_config.adc_range);

    // Power-down mode.
    ppg_sleep();

    return true;
}

void ppg_sleep() {
    LOG_INF("Sleeping PPG device...");

    // Power-down mode.
    gpio_pin_set_dt(&en_3v3, 0);

    ppg.shutDown();
}

void ppg_wakeup() {
    LOG_INF("Waking PPG device...");

    // Power-up mode.
    gpio_pin_set_dt(&en_3v3, 1);
    ppg.wakeUp();
    k_sleep(K_MSEC(50));

    LOG_INF("  -> Clearing out sensor FIFO");
    ppg.clearFIFO();
}

uint32_t ppg_read_samples(
    uint32_t* const red_samples,
    uint32_t* const ir_samples,
    uint32_t sample_i,
    const uint32_t N) {
    if (!ppg.available())
        ppg.check();
    uint8_t available = ppg.available();

    int max_i = N - sample_i;
    if (max_i > available)
        max_i = available;
    for (int i = 0; i < max_i; ++i) {
        ir_samples[sample_i] = ppg.getFIFOIR();
        red_samples[sample_i] = ppg.getFIFORed();
        ppg.nextSample();

        ++sample_i;
    }
    return sample_i;
}

uint16_t ppg_startup_timeout_ms() {
    uint8_t startup_timeout =
        (ppg_config.collection_mode & CollectionMode_StartupTimeout_Mask) >> 4;
    return startup_timeout * CollectionMode_StartupTimeout_Scale * 1000;
}

uint16_t ppg_collection_period_ms() {
    uint8_t collection_period =
        ppg_config.collection_mode & CollectionMode_CollectionPeriod_Mask;
    return collection_period * CollectionMode_CollectionPeriod_Scale;
}
