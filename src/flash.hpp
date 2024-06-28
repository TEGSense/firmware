#pragma once

#include <stdint.h>

bool flash_init();
bool flash_write(const int id, const char* const name, const uint8_t value);
bool flash_read(
    const int id,
    const char* const name,
    uint8_t& value,
    const uint8_t default_value);
