/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT maxim_max3010x

#include <zephyr/logging/log.h>

#include "max3010x.h"

LOG_MODULE_REGISTER(MAX3010x, CONFIG_SENSOR_LOG_LEVEL);

static int max3010x_sample_fetch(
    const struct device* dev, enum sensor_channel chan) {
    struct max3010x_data* data = dev->data;
    const struct max3010x_config* config = dev->config;
    uint8_t buffer[MAX3010x_MAX_BYTES_PER_SAMPLE];
    uint32_t fifo_data;
    int fifo_chan;
    int num_bytes;
    int i;

    /* Read all the active channels for one sample */
    num_bytes = data->num_channels * MAX3010x_BYTES_PER_CHANNEL;
    if (i2c_burst_read_dt(
            &config->i2c, MAX3010x_REG_FIFO_DATA, buffer, num_bytes)) {
        LOG_ERR("Could not fetch sample");
        return -EIO;
    }

    fifo_chan = 0;
    for (i = 0; i < num_bytes; i += 3) {
        /* Each channel is 18-bits */
        fifo_data = (buffer[i] << 16) | (buffer[i + 1] << 8) | (buffer[i + 2]);
        fifo_data &= MAX3010x_FIFO_DATA_MASK;

        /* Save the raw data */
        data->raw[fifo_chan++] = fifo_data;
    }

    return 0;
}

static int max3010x_channel_get(
    const struct device* dev,
    enum sensor_channel chan,
    struct sensor_value* val) {
    struct max3010x_data* data = dev->data;
    enum max3010x_led_channel led_chan;
    int fifo_chan;

    switch (chan) {
        case SENSOR_CHAN_RED:
            led_chan = MAX3010x_LED_CHANNEL_RED;
            break;

        case SENSOR_CHAN_IR:
            led_chan = MAX3010x_LED_CHANNEL_IR;
            break;

        case SENSOR_CHAN_GREEN:
            led_chan = MAX3010x_LED_CHANNEL_GREEN;
            break;

        default:
            LOG_ERR("Unsupported sensor channel");
            return -ENOTSUP;
    }

    /* Check if the led channel is active by looking up the associated fifo
     * channel. If the fifo channel isn't valid, then the led channel
     * isn't active.
     */
    fifo_chan = data->map[led_chan];
    if (fifo_chan >= MAX3010x_MAX_NUM_CHANNELS) {
        LOG_ERR("Inactive sensor channel");
        return -ENOTSUP;
    }

    /* TODO: Scale the raw data to standard units */
    val->val1 = data->raw[fifo_chan];
    val->val2 = 0;

    return 0;
}

static const struct sensor_driver_api max3010x_driver_api = {
    .sample_fetch = max3010x_sample_fetch,
    .channel_get = max3010x_channel_get,
};

static int max3010x_init(const struct device* dev) {
    const struct max3010x_config* config = dev->config;
    struct max3010x_data* data = dev->data;
    uint8_t part_id;
    uint8_t mode_cfg;
    uint32_t led_chan;
    int fifo_chan;

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("Bus device is not ready");
        return -ENODEV;
    }

    /* Check the part id to make sure this is MAX3010x */
    if (i2c_reg_read_byte_dt(&config->i2c, MAX3010x_REG_PART_ID, &part_id)) {
        LOG_ERR("Could not get Part ID");
        return -EIO;
    }
    if (part_id != MAX3010x_PART_ID) {
        LOG_ERR(
            "Got Part ID 0x%02x, expected 0x%02x", part_id, MAX3010x_PART_ID);
        return -EIO;
    }

    /* Reset the sensor */
    if (i2c_reg_write_byte_dt(
            &config->i2c, MAX3010x_REG_MODE_CFG,
            MAX3010x_MODE_CFG_RESET_MASK)) {
        return -EIO;
    }

    /* Wait for reset to be cleared */
    do {
        if (i2c_reg_read_byte_dt(
                &config->i2c, MAX3010x_REG_MODE_CFG, &mode_cfg)) {
            LOG_ERR("Could read mode cfg after reset");
            return -EIO;
        }
    } while (mode_cfg & MAX3010x_MODE_CFG_RESET_MASK);

    /* Write the FIFO configuration register */
    if (i2c_reg_write_byte_dt(
            &config->i2c, MAX3010x_REG_FIFO_CFG, config->fifo)) {
        return -EIO;
    }

    /* Write the mode configuration register */
    if (i2c_reg_write_byte_dt(
            &config->i2c, MAX3010x_REG_MODE_CFG, config->mode)) {
        return -EIO;
    }

    /* Write the SpO2 configuration register */
    if (i2c_reg_write_byte_dt(
            &config->i2c, MAX3010x_REG_SPO2_CFG, config->spo2)) {
        return -EIO;
    }

    /* Write the LED pulse amplitude registers */
    if (i2c_reg_write_byte_dt(
            &config->i2c, MAX3010x_REG_LED1_PA, config->led_pa[0])) {
        return -EIO;
    }
    if (i2c_reg_write_byte_dt(
            &config->i2c, MAX3010x_REG_LED2_PA, config->led_pa[1])) {
        return -EIO;
    }
    if (i2c_reg_write_byte_dt(
            &config->i2c, MAX3010x_REG_LED3_PA, config->led_pa[2])) {
        return -EIO;
    }

#ifdef CONFIG_MAX3010x_MULTI_LED_MODE
    uint8_t multi_led[2];

    /* Write the multi-LED mode control registers */
    multi_led[0] = (config->slot[1] << 4) | (config->slot[0]);
    multi_led[1] = (config->slot[3] << 4) | (config->slot[2]);

    if (i2c_reg_write_byte_dt(
            &config->i2c, MAX3010x_REG_MULTI_LED, multi_led[0])) {
        return -EIO;
    }
    if (i2c_reg_write_byte_dt(
            &config->i2c, MAX3010x_REG_MULTI_LED + 1, multi_led[1])) {
        return -EIO;
    }
#endif

    /* Initialize the channel map and active channel count */
    data->num_channels = 0U;
    for (led_chan = 0U; led_chan < MAX3010x_MAX_NUM_CHANNELS; led_chan++) {
        data->map[led_chan] = MAX3010x_MAX_NUM_CHANNELS;
    }

    /* Count the number of active channels and build a map that translates
     * the LED channel number (red/ir/green) to the fifo channel number.
     */
    for (fifo_chan = 0; fifo_chan < MAX3010x_MAX_NUM_CHANNELS; fifo_chan++) {
        led_chan = (config->slot[fifo_chan] & MAX3010x_SLOT_LED_MASK) - 1;
        if (led_chan < MAX3010x_MAX_NUM_CHANNELS) {
            data->map[led_chan] = fifo_chan;
            data->num_channels++;
        }
    }

    return 0;
}

int max3010x_set_shutdown(
    const struct device* dev, const enum max3010x_power shutdown) {
    const struct max3010x_config* config = dev->config;

    /* Set mode */
    if (shutdown == MAX3010x_SHUTDOWN) {
        if (i2c_reg_write_byte_dt(
                &config->i2c, MAX3010x_REG_MODE_CFG,
                MAX3010x_MODE_CFG_SHDN_MASK)) {
            return -EIO;
        }
    } else {
        if (i2c_reg_write_byte_dt(
                &config->i2c, MAX3010x_REG_MODE_CFG, config->mode)) {
            return -EIO;
        }
    }

    return 0;
}

int max3010x_update_led_pa(
    const struct device* dev, const uint8_t pa1, const uint8_t pa2) {
    const struct max3010x_config* config = dev->config;

    /* Write the LED pulse amplitude registers */
    if (i2c_reg_write_byte_dt(&config->i2c, MAX3010x_REG_LED1_PA, pa1)) {
        return -EIO;
    }
    if (i2c_reg_write_byte_dt(&config->i2c, MAX3010x_REG_LED2_PA, pa2)) {
        return -EIO;
    }

    return 0;
}

static uint8_t getWritePointer(const struct device* dev) {
    const struct max3010x_config* config = dev->config;
    uint8_t fifo_wr;
    if (!i2c_reg_read_byte_dt(&config->i2c, MAX3010x_REG_FIFO_WR, &fifo_wr)) {
        return 0;
    }
    return fifo_wr;
}

static uint8_t getReadPointer(const struct device* dev) {
    const struct max3010x_config* config = dev->config;
    uint8_t fifo_rd;
    if (!i2c_reg_read_byte_dt(&config->i2c, MAX3010x_REG_FIFO_RD, &fifo_rd)) {
        return 0;
    }
    return fifo_rd;
}

uint8_t max3010x_samples_in_fifo(const struct device* dev) {
    const struct max3010x_config* config = dev->config;
    uint8_t num_samples = 0;

    uint16_t fifo_wr = getWritePointer(dev);
    uint16_t fifo_rd = getReadPointer(dev);

    if (fifo_wr != fifo_rd) {
        // Calculate the number of readings we need to get from sensor
        int numberOfSamples = fifo_wr - fifo_rd;
        if (numberOfSamples < 0)
            numberOfSamples += 32;  // Wrap condition
        num_samples = numberOfSamples;
    }

    return num_samples;
}

static struct max3010x_config max3010x_config = {
    .i2c = I2C_DT_SPEC_INST_GET(0),
    .fifo = (CONFIG_MAX3010x_SMP_AVE << MAX3010x_FIFO_CFG_SMP_AVE_SHIFT) |
#ifdef CONFIG_MAX3010x_FIFO_ROLLOVER_EN
            MAX3010x_FIFO_CFG_ROLLOVER_EN_MASK |
#endif
            (CONFIG_MAX3010x_FIFO_A_FULL << MAX3010x_FIFO_CFG_FIFO_FULL_SHIFT),

#if defined(CONFIG_MAX3010x_HEART_RATE_MODE)
    .mode = MAX3010x_MODE_HEART_RATE,
    .slot[0] = MAX3010x_SLOT_RED_LED1_PA,
    .slot[1] = MAX3010x_SLOT_DISABLED,
    .slot[2] = MAX3010x_SLOT_DISABLED,
    .slot[3] = MAX3010x_SLOT_DISABLED,
#elif defined(CONFIG_MAX3010x_SPO2_MODE)
    .mode = MAX3010x_MODE_SPO2,
    .slot[0] = MAX3010x_SLOT_RED_LED1_PA,
    .slot[1] = MAX3010x_SLOT_IR_LED2_PA,
    .slot[2] = MAX3010x_SLOT_DISABLED,
    .slot[3] = MAX3010x_SLOT_DISABLED,
#else
    .mode = MAX3010x_MODE_MULTI_LED,
    .slot[0] = CONFIG_MAX3010x_SLOT1,
    .slot[1] = CONFIG_MAX3010x_SLOT2,
    .slot[2] = CONFIG_MAX3010x_SLOT3,
    .slot[3] = CONFIG_MAX3010x_SLOT4,
#endif

    .spo2 = (CONFIG_MAX3010x_ADC_RGE << MAX3010x_SPO2_ADC_RGE_SHIFT) |
            (CONFIG_MAX3010x_SR << MAX3010x_SPO2_SR_SHIFT) |
            (CONFIG_MAX3010x_PW << MAX3010x_SPO2_PW_SHIFT),

    .led_pa[0] = CONFIG_MAX3010x_LED1_PA,
    .led_pa[1] = CONFIG_MAX3010x_LED2_PA,
    .led_pa[2] = CONFIG_MAX3010x_LED3_PA,
};

static struct max3010x_data max3010x_data;

DEVICE_DT_INST_DEFINE(
    0,
    max3010x_init,
    NULL,
    &max3010x_data,
    &max3010x_config,
    POST_KERNEL,
    CONFIG_SENSOR_INIT_PRIORITY,
    &max3010x_driver_api);
