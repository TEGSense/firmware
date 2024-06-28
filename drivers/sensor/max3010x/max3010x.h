/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>

#define MAX3010x_REG_INT_STS1 0x00
#define MAX3010x_REG_INT_STS2 0x01
#define MAX3010x_REG_INT_EN1 0x02
#define MAX3010x_REG_INT_EN2 0x03
#define MAX3010x_REG_FIFO_WR 0x04
#define MAX3010x_REG_FIFO_OVF 0x05
#define MAX3010x_REG_FIFO_RD 0x06
#define MAX3010x_REG_FIFO_DATA 0x07
#define MAX3010x_REG_FIFO_CFG 0x08
#define MAX3010x_REG_MODE_CFG 0x09
#define MAX3010x_REG_SPO2_CFG 0x0a
#define MAX3010x_REG_LED1_PA 0x0c
#define MAX3010x_REG_LED2_PA 0x0d
#define MAX3010x_REG_LED3_PA 0x0e
#define MAX3010x_REG_PILOT_PA 0x10
#define MAX3010x_REG_MULTI_LED 0x11
#define MAX3010x_REG_TINT 0x1f
#define MAX3010x_REG_TFRAC 0x20
#define MAX3010x_REG_TEMP_CFG 0x21
#define MAX3010x_REG_PROX_INT 0x30
#define MAX3010x_REG_REV_ID 0xfe
#define MAX3010x_REG_PART_ID 0xff

#define MAX3010x_INT_PPG_MASK (1 << 6)

#define MAX3010x_FIFO_CFG_SMP_AVE_SHIFT 5
#define MAX3010x_FIFO_CFG_FIFO_FULL_SHIFT 0
#define MAX3010x_FIFO_CFG_ROLLOVER_EN_MASK (1 << 4)

#define MAX3010x_MODE_CFG_SHDN_MASK (1 << 7)
#define MAX3010x_MODE_CFG_RESET_MASK (1 << 6)

#define MAX3010x_SPO2_ADC_RGE_SHIFT 5
#define MAX3010x_SPO2_SR_SHIFT 2
#define MAX3010x_SPO2_PW_SHIFT 0

#define MAX3010x_PART_ID 0x15

#define MAX3010x_BYTES_PER_CHANNEL 3
#define MAX3010x_MAX_NUM_CHANNELS 3
#define MAX3010x_MAX_BYTES_PER_SAMPLE \
    (MAX3010x_MAX_NUM_CHANNELS * MAX3010x_BYTES_PER_CHANNEL)

#define MAX3010x_SLOT_LED_MASK 0x03

#define MAX3010x_FIFO_DATA_BITS 18
#define MAX3010x_FIFO_DATA_MASK ((1 << MAX3010x_FIFO_DATA_BITS) - 1)

enum max3010x_mode {
    MAX3010x_MODE_HEART_RATE = 2,
    MAX3010x_MODE_SPO2 = 3,
    MAX3010x_MODE_MULTI_LED = 7,
};

enum max3010x_slot {
    MAX3010x_SLOT_DISABLED = 0,
    MAX3010x_SLOT_RED_LED1_PA,
    MAX3010x_SLOT_IR_LED2_PA,
    MAX3010x_SLOT_GREEN_LED3_PA,
    MAX3010x_SLOT_RED_PILOT_PA,
    MAX3010x_SLOT_IR_PILOT_PA,
    MAX3010x_SLOT_GREEN_PILOT_PA,
};

enum max3010x_led_channel {
    MAX3010x_LED_CHANNEL_RED = 0,
    MAX3010x_LED_CHANNEL_IR,
    MAX3010x_LED_CHANNEL_GREEN,
};

enum max3010x_pw {
    MAX3010x_PW_15BIT_69us = 0x00,
    MAX3010x_PW_16BIT_118us = 0x01,
    MAX3010x_PW_17BIT_215us = 0x02,
    MAX3010x_PW_18BIT_411us = 0x03,
};

enum max3010x_power {
    MAX3010x_SHUTDOWN = 0,
    MAX3010x_ON,
};

struct max3010x_config {
        struct i2c_dt_spec i2c;
        uint8_t fifo;
        uint8_t spo2;
        uint8_t led_pa[MAX3010x_MAX_NUM_CHANNELS];
        enum max3010x_mode mode;
        enum max3010x_slot slot[4];
};

struct max3010x_data {
        uint32_t raw[MAX3010x_MAX_NUM_CHANNELS];
        uint8_t map[MAX3010x_MAX_NUM_CHANNELS];
        uint8_t num_channels;
};

int max3010x_set_shutdown(
    const struct device* dev, const enum max3010x_power shutdown);

int max3010x_update_led_pa(
    const struct device* dev, const uint8_t pa1, const uint8_t pa2);

uint8_t max3010x_samples_in_fifo(const struct device* dev);

#ifdef __cplusplus
}
#endif
