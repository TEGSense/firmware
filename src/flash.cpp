#include "flash.hpp"

#include <zephyr/drivers/flash.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/storage/flash_map.h>

#include <zephyr/logging/log.h>

#define LOG_MODULE_NAME flash
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define NVS_PARTITION storage_partition
#define NVS_PARTITION_DEVICE FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET FIXED_PARTITION_OFFSET(NVS_PARTITION)

static struct nvs_fs fs;

bool flash_init() {
    int rc;
    struct flash_pages_info info;

    // Initialize flash.
    fs.flash_device = NVS_PARTITION_DEVICE;
    if (!device_is_ready(fs.flash_device)) {
        LOG_ERR("Flash device %s is not ready", fs.flash_device->name);
        return false;
    }
    fs.offset = NVS_PARTITION_OFFSET;
    rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
    if (rc) {
        LOG_ERR("Unable to get page info");
        return false;
    }
    fs.sector_size = info.size;
    fs.sector_count = 3U;

    rc = nvs_mount(&fs);
    if (rc) {
        LOG_ERR("Flash Init failed (%d)", rc);
        return false;
    }

    return true;
}

bool flash_write(const int id, const char* const name, const uint8_t value) {
    int rc = nvs_write(&fs, id, &value, 1);
    // Will return zero bytes written if the value is unchanged.
    if (rc >= 0)
        LOG_INF("-> set config[%s] = %d (0x%02x)", name, value, value);
    else
        LOG_ERR("-> set config[%s]: Failed to write!!", name);
    return rc >= 0;
}

bool flash_read(
    const int id,
    const char* const name,
    uint8_t& value,
    const uint8_t default_value) {
    int rc = nvs_read(&fs, id, &value, 1);
    if (rc > 0)
        LOG_INF("config[%s]: %d (0x%02x)", name, value, value);
    else {
        LOG_ERR("config[%s]: Not found!!", name);
        value = default_value;
        flash_write(id, name, value);
    }
    return rc > 0;
}
