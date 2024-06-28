#pragma once

#include <stdint.h>

// Command codes.
constexpr uint8_t NoOp = 0x00;
constexpr uint8_t ADCRange = 0x01;
constexpr uint8_t SampleRate = 0x02;
constexpr uint8_t PulseWidth = 0x04;
constexpr uint8_t SampleAvg = 0x08;
constexpr uint8_t IRLEDPA = 0x10;
constexpr uint8_t RedLEDPA = 0x20;
constexpr uint8_t Reboot = 0x40;
constexpr uint8_t CollectionMode = 0x80;

// CollectionMode modes.
constexpr uint8_t CollectionMode_CollectionPeriod_Mask = 0x0F;
constexpr uint16_t CollectionMode_CollectionPeriod_Scale = 500;

constexpr uint8_t CollectionMode_StartupTimeout_Mask = 0xF0;
constexpr uint16_t CollectionMode_StartupTimeout_Scale = 10;

constexpr uint16_t CommandSize = 2;

struct Command {
    public:
        uint8_t command;
        uint8_t data;
};

extern uint8_t commands;
extern uint8_t values[8];

// Decode a command.
bool decode_command(const uint8_t bytes[CommandSize], Command& cmd);
bool push_command(const Command& cmd);
bool push_command(const uint8_t bytes[CommandSize]);

typedef void (*CommandCallback)(const uint8_t cmd, const uint8_t data);
int process_commands(CommandCallback cb);
