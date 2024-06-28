#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/pm/pm.h>
#include <zephyr/sys/reboot.h>

#include "ble.hpp"
#include "command.hpp"
#include "flash.hpp"
#include "ppg.hpp"

LOG_MODULE_REGISTER(main);

static uint16_t packet_id = 0;

static uint8_t next_cfg = 0;
static uint8_t next_fifo_cfg = 0;
static uint8_t next_cp_cfg = 0;
static uint8_t next_ir_pa = 0;
static uint8_t next_red_pa = 0;

const uint16_t MaxSamples = 50;
struct ppg_buffer {
        const uint32_t syncword = 0xdeadbeef;
        uint32_t time = 0;
        uint16_t packet_id = 0;
        uint8_t cfg = next_cfg;
        uint8_t fifo_cfg = next_fifo_cfg;
        uint8_t cp_cfg = next_cp_cfg;
        uint8_t red_pa = next_red_pa;
        uint8_t ir_pa = next_ir_pa;
        uint16_t sample_i = 0;
        uint32_t red[MaxSamples];
        uint32_t ir[MaxSamples];
};

const uint16_t MaxBuffers = 20;
static ppg_buffer buf[MaxBuffers];
static uint16_t write_buf = 0;
static uint16_t read_buf = 0;

static uint32_t collection_cutoff_ms = 0;
static bool collecting = true;

ppg_buffer& get_write_buffer() {
    return buf[write_buf];
}

bool write_buffer_full() {
    auto& buffer = get_write_buffer();
    return buffer.sample_i == MaxSamples;
}

bool read_buffer_ready() {
    return read_buf != write_buf;
}

ppg_buffer& get_read_buffer() {
    return buf[read_buf];
}

void inc_write_buffer() {
    LOG_DBG(
        "Moving WRITE from buffer %d to %d", write_buf,
        (write_buf + 1) % MaxBuffers);
    write_buf = (write_buf + 1) % MaxBuffers;

    auto& buffer = get_write_buffer();
    buffer.packet_id = ++packet_id;
    buffer.time = k_cyc_to_ms_floor32(sys_clock_tick_get());
    buffer.sample_i = 0;

    buffer.cfg = next_cfg;
    buffer.fifo_cfg = next_fifo_cfg;
    buffer.cp_cfg = next_cp_cfg;
    buffer.ir_pa = next_ir_pa;
    buffer.red_pa = next_red_pa;
}

void inc_read_buffer() {
    LOG_DBG(
        "Moving READ from buffer %d to %d", read_buf,
        (read_buf + 1) % MaxBuffers);
    read_buf = (read_buf + 1) % MaxBuffers;
}

void process_cb(const uint8_t cmd, const uint8_t data) {
    switch (cmd) {
        case NoOp:
            break;
        case ADCRange:
            ppg.setADCRange(data);
            break;
        case SampleRate:
            ppg.setSampleRate(data);
            break;
        case PulseWidth:
            ppg.setPulseWidth(data);
            break;
        case SampleAvg:
            ppg.setFIFOAverage(data);
            break;
        case IRLEDPA:
            ppg.setPulseAmplitudeIR(data);
            break;
        case RedLEDPA:
            ppg.setPulseAmplitudeRed(data);
            break;
        case Reboot:
            LOG_WRN("System reboot was triggered.");
            LOG_INF("Storing PPG configuration parameters...");
            ppg_config.adc_range = next_cfg & ~0x9F;
            ppg_config.sample_rate = next_cfg & ~0xE3;
            ppg_config.pulse_width = next_cfg & ~0xFC;
            ppg_config.sample_avg = next_fifo_cfg & 0xE0;
            ppg_config.ir_led_pa = next_ir_pa;
            ppg_config.red_led_pa = next_red_pa;
            if (!ppg_write_config())
                LOG_ERR("Failed to write PPG configuration to flash.");

            LOG_INF("Flushing log buffer then rebooting in 1 second...");
            // Flush the log buffer before rebooting.
            if (IS_ENABLED(CONFIG_LOG_MODE_DEFERRED))
                while (log_process())
                    ;
            k_sleep(K_MSEC(1000));

            sys_reboot(SYS_REBOOT_COLD);
            break;
        case CollectionMode:
            ppg_config.collection_mode = data;
            next_cp_cfg = data;
            LOG_INF("Setting collection mode: %d (0x%02x)", data, data);
            LOG_INF(
                "  -> Collection period: %d ms", ppg_collection_period_ms());
            LOG_INF("  -> Startup timeout: %d ms", ppg_startup_timeout_ms());
            LOG_INF("Will only take effect after reboot.");
            break;
        default:
            break;
    }
}

void collect_data() {
    if (!collecting)
        return;
    else if (k_cyc_to_ms_floor32(sys_clock_tick_get()) > collection_cutoff_ms) {
        LOG_INF(
            "Collection period has expired, going to sleep after transmitting "
            "data...");
        collecting = false;
        ppg_sleep();
        return;
    }

    auto& buffer = get_write_buffer();
    buffer.sample_i =
        ppg_read_samples(buffer.red, buffer.ir, buffer.sample_i, MaxSamples);
}

void process_messages() {
    // While collecting data, only process during buffer transitions since
    // we don't want to change the sensor configuration in the middle of
    // a buffer.
    if (collecting && !write_buffer_full())
        return;
    if (process_commands(process_cb) > 0) {
        // Update PPG configuration parameters for telemetry.
        next_cfg = ppg.getParticleConfig();
        next_fifo_cfg = ppg.getFIFOConfig();
        next_ir_pa = ppg.getPAIR();
        next_red_pa = ppg.getPARed();

        // Clear FIFO.
        ppg.clearFIFO();
    }
}

void telemeter_data() {
    if (read_buffer_ready() && ble_await_connection(10)) {
        auto& buffer = get_read_buffer();
        uint16_t size = sizeof(ppg_buffer);
        LOG_DBG(
            "Sending %d bytes of packet id %d (read buffer %d)...", size,
            buffer.packet_id, read_buf);
        ble_send(reinterpret_cast<const uint8_t*>(&buffer), size, true);
        inc_read_buffer();
    }
}

int main() {
    if (!flash_init()) {
        LOG_ERR("Failed to initialize flash, going to reboot...");
        // Flush the log buffer before rebooting.
        if (IS_ENABLED(CONFIG_LOG_MODE_DEFERRED))
            while (log_process())
                ;
        k_sleep(K_MSEC(1000));

        sys_reboot(SYS_REBOOT_COLD);

        return 1;
    }
    if (!ppg_configure()) {
        LOG_ERR("Failed to configure PPG, going to reboot...");
        // Flush the log buffer before rebooting.
        if (IS_ENABLED(CONFIG_LOG_MODE_DEFERRED))
            while (log_process())
                ;
        k_sleep(K_MSEC(1000));

        sys_reboot(SYS_REBOOT_COLD);

        return 1;
    }
    if (!ble_configure()) {
        LOG_ERR("Failed to configure BLE, going to reboot...");
        // Flush the log buffer before rebooting.
        if (IS_ENABLED(CONFIG_LOG_MODE_DEFERRED))
            while (log_process())
                ;
        k_sleep(K_MSEC(1000));

        sys_reboot(SYS_REBOOT_COLD);

        return 1;
    }

    // Store PPG configuration parameters.
    next_cfg = ppg.getParticleConfig();
    next_fifo_cfg = ppg.getFIFOConfig();
    next_ir_pa = ppg.getPAIR();
    next_red_pa = ppg.getPARed();

    // And collection mode.
    next_cp_cfg = ppg_config.collection_mode;

    // Check for a startup timeout.
    uint16_t startup_timeout = ppg_startup_timeout_ms();
    if (startup_timeout > 0) {
        LOG_INF("Sleeping for %d ms to allow for charging...", startup_timeout);
        k_sleep(K_MSEC(startup_timeout));
    } else {
        LOG_INF("Startup timeout is disabled, continuing immediately.");
    }

    // Setup the collection cutoff time.
    uint16_t collection_period = ppg_collection_period_ms();
    if (collection_period > 0) {
        collection_cutoff_ms =
            k_cyc_to_ms_floor32(sys_clock_tick_get()) + collection_period;
        LOG_INF(
            "Collection period is %d ms, will stop collecting at %d ms.",
            collection_period, collection_cutoff_ms);
    } else {
        LOG_INF("Collection period is disabled, will collect indefinitely.");
        collection_cutoff_ms = 0xFFFFFFFF;
    }

    // Wakeup BLE and PPG.
    ble_wakeup();
    ppg_wakeup();

    // Initialize first buffer.
    auto& buffer = get_write_buffer();
    buffer.time = k_cyc_to_ms_floor32(sys_clock_tick_get());
    buffer.cfg = next_cfg;
    buffer.fifo_cfg = next_fifo_cfg;
    buffer.cp_cfg = next_cp_cfg;
    buffer.ir_pa = next_ir_pa;
    buffer.red_pa = next_red_pa;

    // Forever.
    static uint32_t delay_ms = 10;
    // int iterations = 0;
    while (true) {
        k_sleep(K_MSEC(delay_ms));
        collect_data();
        process_messages();
        if (write_buffer_full())
            inc_write_buffer();
        telemeter_data();

        // When we stop collecting data, we can increase the loop delay.
        if (!collecting && !read_buffer_ready() && delay_ms < 500) {
            LOG_INF("No more data to send, increasing loop delay.");
            delay_ms = 500;
        }

        // Chirp to keep the BLE link alive.
        if (delay_ms > 10) {
            static const char* data = "idle";
            ble_send(reinterpret_cast<const uint8_t*>(data), 4);
        }
    }

    // At this point we'll just look for commands, maybe a reboot.
    int iterations = 0;
    while (true) {
        k_sleep(K_MSEC(delay_ms));
        if (iterations++ % (1000 / delay_ms) == 0) {
            static const char* data = "idle";
            ble_send(reinterpret_cast<const uint8_t*>(data), 4);
        }
        if (process_commands(process_cb) > 0) {
            // Update PPG configuration parameters for telemetry.
            next_cfg = ppg.getParticleConfig();
            next_fifo_cfg = ppg.getFIFOConfig();
            next_ir_pa = ppg.getPAIR();
            next_red_pa = ppg.getPARed();

            // Clear FIFO.
            ppg.clearFIFO();
        }
        k_sleep(K_MSEC(delay_ms));
    }

    return 0;
}
