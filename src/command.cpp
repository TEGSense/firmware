#include "command.hpp"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(command);

K_MUTEX_DEFINE(command_mutex);

uint8_t commands;
uint8_t values[8];

static const char* CommandNames[] = {
    "ADCRange", "SampleRate", "PulseWidth", "SampleAvg",
    "IRLEDPA",  "RedLEDPA",   "Reboot",     "CollectionMode",
};

// Decode a command.
bool decode(const uint8_t bytes[CommandSize], Command& cmd) {
    cmd = {.command = bytes[0], .data = bytes[1]};

    switch (cmd.command) {
        case NoOp:
        case ADCRange:
        case SampleRate:
        case PulseWidth:
        case SampleAvg:
        case IRLEDPA:
        case RedLEDPA:
        case Reboot:
        case CollectionMode:
            break;
        default:
            return false;
    }

    return true;
}

bool push_command(const Command& cmd) {
    if (k_mutex_lock(&command_mutex, K_MSEC(10))) {
        LOG_ERR("Failed to lock command mutex.");
        return false;
    }

    commands |= cmd.command;

    // Set the command data.
    for (uint8_t i = 0; i < 8; ++i) {
        uint8_t test_cmd = (1 << i);
        if (test_cmd == cmd.command) {
            values[i] = cmd.data;
            break;
        }
    }

    k_mutex_unlock(&command_mutex);

    return true;
}

bool push_command(const uint8_t bytes[CommandSize]) {
    Command cmd;
    if (!decode(bytes, cmd)) {
        LOG_ERR("Failed to decode command.");
        return false;
    }

    return push_command(cmd);
}

int process_commands(CommandCallback cb) {
    if (k_mutex_lock(&command_mutex, K_MSEC(10))) {
        LOG_ERR("Failed to lock command mutex.");
        return 0;
    }

    // Make a copy of the commands and values so we don't need to hold the
    // mutex.
    const uint8_t cmds = commands;
    uint8_t vals[8];
    for (uint8_t i = 0; i < 8; ++i) {
        vals[i] = values[i];
    }

    // And clear the command flags.
    commands = 0;

    k_mutex_unlock(&command_mutex);

    // Call the callback for all active commands.
    int commands_processed = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        uint8_t cmd = (1 << i);
        if (cmds & cmd) {
            ++commands_processed;
            LOG_INF(
                "CMD %s (%02X) -> %d (%02X).", CommandNames[i], cmd, vals[i],
                vals[i]);
            cb(cmd, vals[i]);
        }
    }

    return commands_processed;
}
