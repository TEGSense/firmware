#include "ble.hpp"

#include <bluetooth/services/nus.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>

#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include "command.hpp"

LOG_MODULE_REGISTER(ble);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static struct bt_data ad[2] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static bool ble_initialized = false;
static struct bt_conn* connection = NULL;

static const uint16_t nus_buffer_size = 244;
static uint8_t nus_buffer[nus_buffer_size];
static uint16_t used = 0;

static uint32_t connection_time = 0;

K_SEM_DEFINE(ble_connection_semaphore, 0, 1);

static void connected(struct bt_conn* conn, uint8_t conn_err) {
    char addr[BT_ADDR_LE_STR_LEN];

    LOG_INF("BLE::connected");

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    if (conn_err) {
        LOG_INF(
            "Failed to connect to %s (%d - %s)", addr, conn_err,
            strerror(conn_err));

        return;
    }

    LOG_INF("Connected to %s", addr);
    if ((connection != NULL) && (connection != conn)) {
        LOG_WRN("  -> Previous connection will be released.");
        bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        bt_conn_unref(connection);
    }
    connection = bt_conn_ref(conn);

    connection_time = k_cyc_to_ms_floor32(sys_clock_tick_get());
    LOG_INF("Connected at %d", connection_time);
    k_sem_give(&ble_connection_semaphore);
}

static void disconnected(struct bt_conn* conn, uint8_t reason) {
    char addr[BT_ADDR_LE_STR_LEN];

    LOG_INF("BLE::disconnected");
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    // Determine which slot is being disconnected.
    if (conn == connection) {
        // err can mean either of the following:
        // - BT_HCI_ERR_UNKNOWN_CONN_ID
        // Creating the connection started by bt_conn_le_create
        // canceled either by the user through bt_conn_disconnect
        // or by the timeout in the host through bt_conn_le_create_param
        // timeout parameter, which defaults to
        // @kconfig{CONFIG_BT_CREATE_CONN_TIMEOUT} seconds.
        // - BT_HCI_ERR_ADV_TIMEOUT
        // High duty cycle directed connectable advertiser started by
        // bt_le_adv_start failed to be connected within the timeout.
        const char* reason_str = "Unknown";
        switch (reason) {
            case BT_HCI_ERR_UNKNOWN_CONN_ID:
                reason_str = "Unknown connection ID";
                break;
            case BT_HCI_ERR_CONN_FAIL_TO_ESTAB:
                reason_str = "Connection failed to establish";
                break;
            case BT_HCI_ERR_ADV_TIMEOUT:
                reason_str = "Advertising timeout";
                break;
            case BT_HCI_ERR_CONN_TIMEOUT:
                reason_str = "Connection timeout";
                break;
        }
        LOG_WRN(
            "Disconnected from %s (reason: %x - %s)", addr, reason, reason_str);
        bt_conn_unref(connection);
        connection = NULL;

        k_sem_reset(&ble_connection_semaphore);
    }
}

static bool le_param_req(struct bt_conn* conn, struct bt_le_conn_param* param) {
    LOG_INF("Connection parameters update request received.");
    LOG_INF(
        "  - Connection interval: [%d, %d]", param->interval_min,
        param->interval_max);
    LOG_INF("  - Latency: %d", param->latency);
    LOG_INF("  - Timeout: %d", param->timeout);

    return true;
}

static void le_param_updated(
    struct bt_conn* conn,
    uint16_t interval,
    uint16_t latency,
    uint16_t timeout) {
    LOG_INF("Connection parameters updated:");
    LOG_INF(
        "  - Interval: %d; Latency: %d; Timeout: %d", interval, latency,
        timeout);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
    .le_param_req = le_param_req,
    .le_param_updated = le_param_updated,
};

static void auth_cancel(struct bt_conn* conn) {
    char addr[BT_ADDR_LE_STR_LEN];

    LOG_INF("BLE::auth_cancel");
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Pairing cancelled: %s", addr);
}

static void pairing_complete(struct bt_conn* conn, bool bonded) {
    char addr[BT_ADDR_LE_STR_LEN];

    LOG_INF("BLE::pairing_complete");
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}

static void pairing_failed(struct bt_conn* conn, enum bt_security_err reason) {
    char addr[BT_ADDR_LE_STR_LEN];

    LOG_INF("BLE::pairing_failed");
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_WRN("Pairing failed conn: %s, reason %d", addr, reason);
}

static void nus_received(
    struct bt_conn* conn, const uint8_t* const data, uint16_t len) {
    // We assume only a single command per packet.
    if (len != CommandSize) {
        LOG_WRN("Received invalid command size: %d", len);
        return;
    }

    LOG_INF(
        "Received command: %d (0x%02X) - %d (0x%02X)", data[0], data[0],
        data[1], data[1]);

    // And we'll just push it right through. When commands are processed, only
    // the most recent value for each is taken, so it doesn't matter how many
    // are received.
    if (!push_command(data))
        LOG_WRN("Failed to push command.");
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
    .cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
    .pairing_complete = pairing_complete, .pairing_failed = pairing_failed};

static struct bt_nus_cb nus_cb = {.received = nus_received};

int init_nus() {
    int err = 0;

    LOG_INF("Initializing BLE and NUS...");

    err = bt_conn_auth_cb_register(&conn_auth_callbacks);
    if (err) {
        LOG_ERR(
            "Failed to register authorization callbacks. %d - %s", err,
            strerror(err));
        return err;
    }

    err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
    if (err) {
        LOG_ERR(
            "Failed to register authorization info callbacks. %d - %s", err,
            strerror(err));
        return err;
    }

    err = bt_nus_init(&nus_cb);
    if (err) {
        LOG_ERR(
            "Failed to initialize NUS callbacks. %d - %s", err, strerror(err));
        return err;
    }

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Error bt_enable: %d - %s", err, strerror(err));
        return err;
    }

    LOG_INF("Bluetooth initialized");

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    return 0;
}

bool ble_configure() {
    LOG_INF("BLE Configure...");
    int err = init_nus();
    ble_initialized = !err;
    return ble_initialized;
}

void ble_sleep() {
    LOG_INF("Sleeping BLE... Stopping advertising and closing connections...");
    bt_le_adv_stop();
    k_sem_reset(&ble_connection_semaphore);
    if (connection != NULL) {
        bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        bt_conn_unref(connection);
        connection = NULL;
    }

    // Reset NUS buffer.
    used = 0;
}

void ble_wakeup() {
    LOG_INF("Waking BLE... Starting advertising...");
    const auto& params = BT_LE_ADV_CONN;
    int err = bt_le_adv_start(params, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
    }

    // Reset NUS buffer.
    used = 0;
}

bool ble_await_connection(const uint32_t timeout_ms) {
    if (k_sem_count_get(&ble_connection_semaphore) > 0) {
        const uint32_t now = k_cyc_to_ms_floor32(sys_clock_tick_get());
        return (now - connection_time) > 500;
    }
    return false;
    // return k_sem_take(&ble_connection_semaphore, K_FOREVER) == 0;
}

uint16_t add_to_nus_buffer(
    const uint8_t* src_buffer,
    const uint16_t src_size,
    const uint16_t src_remaining) {
    uint16_t free = nus_buffer_size - used;
    uint16_t to_write = free < src_remaining ? free : src_remaining;
    uint16_t src_remaining_after_write = src_remaining - to_write;

    uint16_t src_si = src_size - src_remaining;
    for (uint16_t di = 0; di < to_write; ++di)
        nus_buffer[used + di] = src_buffer[src_si + di];

    used += to_write;
    return src_remaining_after_write;
}

bool nus_buffer_full() {
    return used == nus_buffer_size;
}

void transmit_nus_buffer() {
    if (connection == NULL)
        return;
    bt_nus_send(connection, nus_buffer, used);
    used = 0;
}

uint16_t ble_send(
    const uint8_t* src_buffer, const uint16_t src_size, const bool immediate) {
    if (IS_ENABLED(CONFIG_NUS_BUFFERING)) {
        uint16_t remaining = src_size;
        while (remaining > 0) {
            remaining = add_to_nus_buffer(src_buffer, src_size, remaining);
            if (immediate || nus_buffer_full())
                transmit_nus_buffer();
        }
    } else {
        bt_nus_send(connection, src_buffer, src_size);
    }
    return src_size;
}
