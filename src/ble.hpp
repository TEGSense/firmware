#pragma once

#include <stdint.h>

bool ble_configure();
void ble_sleep();
void ble_wakeup();
bool ble_await_connection(const uint32_t timeout_ms);
uint16_t ble_send(
    const uint8_t* src_buffer,
    const uint16_t src_size,
    const bool immediate = false);
