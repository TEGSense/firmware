#pragma once

#include <stdint.h>

#include "MAX30101.hpp"

extern MAX30101 ppg;

struct PPGConfig {
        uint8_t adc_range;
        uint8_t sample_rate;
        uint8_t pulse_width;
        uint8_t sample_avg;
        uint8_t ir_led_pa;
        uint8_t red_led_pa;
        uint8_t collection_mode;
};
extern PPGConfig ppg_config;

bool ppg_read_config();
bool ppg_write_config();
bool ppg_configure();
void ppg_sleep();
void ppg_wakeup();
uint16_t ppg_startup_timeout_ms();
uint16_t ppg_collection_period_ms();

uint32_t ppg_read_samples(
    uint32_t* const red_samples,
    uint32_t* const ir_samples,
    uint32_t sample_i,
    const uint32_t N);
