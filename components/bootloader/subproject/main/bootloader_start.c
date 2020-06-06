// Copyright 2015-2019 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>
#include <sys/param.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "bootloader_init.h"
#include "bootloader_utility.h"
#include "bootloader_common.h"

#if CONFIG_IDF_TARGET_ESP32
#include "esp32/rom/cache.h"
#include "esp32/rom/efuse.h"
#include "esp32/rom/ets_sys.h"
#include "esp32/rom/spi_flash.h"
#include "esp32/rom/crc.h"
#include "esp32/rom/rtc.h"
#include "esp32/rom/uart.h"
#include "esp32/rom/gpio.h"
#include "esp32/rom/secure_boot.h"
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/rom/cache.h"
#include "esp32s2/rom/efuse.h"
#include "esp32s2/rom/ets_sys.h"
#include "esp32s2/rom/spi_flash.h"
#include "esp32s2/rom/crc.h"
#include "esp32s2/rom/rtc.h"
#include "esp32s2/rom/uart.h"
#include "esp32s2/rom/gpio.h"
#include "esp32s2/rom/secure_boot.h"
#include "soc/extmem_reg.h"
#include "soc/cache_memory.h"
#else
#error "Unsupported IDF_TARGET"
#endif

// #include "soc/soc.h"
// #include "soc/cpu.h"
// #include "soc/rtc.h"
// #include "soc/dport_reg.h"
// #include "soc/gpio_periph.h"
// #include "soc/efuse_periph.h"
// #include "soc/rtc_periph.h"
// #include "soc/timer_periph.h"

// #include "sdkconfig.h"
#include "esp_image_format.h"
#include "esp_flash_partitions.h"
#include "bootloader_flash.h"
#include "bootloader_random.h"
// #include "bootloader_config.h"
// #include "bootloader_common.h"
// #include "bootloader_utility.h"
// #include "bootloader_sha.h"
// #include "esp_efuse.h"


static const char *TAG = "boot";
/* Reduce literal size for some generic string literals */
#define MAP_ERR_MSG "Image contains multiple %s segments. Only the last one will be mapped."

/**
 * @brief Software reset the ESP32
 *
 * Bootloader code should call this in the case that it cannot proceed.
 *
 * It is not recommended to call this function from an app (if called, the app will abort).
 */
__attribute__((noreturn)) static void bootloader_reset(void)
{
    uart_tx_flush(0);    /* Ensure any buffered log output is displayed */
    uart_tx_flush(1);
    ets_delay_us(1000); /* Allow last byte to leave FIFO */
    REG_WRITE(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_SW_SYS_RST);
    while (1) { }       /* This line will never be reached, used to keep gcc happy */
}

/* Return true if a partition has a valid app image that was successfully loaded */
static bool try_load_partition(uint32_t partition_offset, uint32_t partition_size, esp_image_metadata_t *data)
{
    if (partition_size == 0) {
        ESP_LOGD(TAG, "Can't boot from zero-length partition");
        return false;
    }

    if (image_load(ESP_IMAGE_LOAD, partition_offset, partition_size, data) == ESP_OK) {
        ESP_LOGI(TAG, "Loaded app from partition at offset 0x%x",
                 partition_offset);
        return true;
    }

    return false;
}

static void set_cache_and_start_app(
    uint32_t drom_addr,
    uint32_t drom_load_addr,
    uint32_t drom_size,
    uint32_t irom_addr,
    uint32_t irom_load_addr,
    uint32_t irom_size,
    uint32_t entry_addr)
{
    int rc;
    ESP_LOGD(TAG, "configure drom and irom and start");
#if CONFIG_IDF_TARGET_ESP32
    Cache_Read_Disable(0);
    Cache_Flush(0);
#elif CONFIG_IDF_TARGET_ESP32S2
    uint32_t autoload = Cache_Suspend_ICache();
    Cache_Invalidate_ICache_All();
#endif

    /* Clear the MMU entries that are already set up,
       so the new app only has the mappings it creates.
    */
#if CONFIG_IDF_TARGET_ESP32
    for (int i = 0; i < DPORT_FLASH_MMU_TABLE_SIZE; i++) {
        DPORT_PRO_FLASH_MMU_TABLE[i] = DPORT_FLASH_MMU_TABLE_INVALID_VAL;
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    for (int i = 0; i < FLASH_MMU_TABLE_SIZE; i++) {
        FLASH_MMU_TABLE[i] = MMU_TABLE_INVALID_VAL;
    }
#endif
    uint32_t drom_load_addr_aligned = drom_load_addr & MMU_FLASH_MASK;
    uint32_t drom_page_count = bootloader_cache_pages_to_map(drom_size, drom_load_addr);
    ESP_LOGV(TAG, "d mmu set paddr=%08x vaddr=%08x size=%d n=%d",
             drom_addr & MMU_FLASH_MASK, drom_load_addr_aligned, drom_size, drom_page_count);
#if CONFIG_IDF_TARGET_ESP32
    rc = cache_flash_mmu_set(0, 0, drom_load_addr_aligned, drom_addr & MMU_FLASH_MASK, 64, drom_page_count);
#elif CONFIG_IDF_TARGET_ESP32S2
    rc = Cache_Ibus_MMU_Set(MMU_ACCESS_FLASH, drom_load_addr & 0xffff0000, drom_addr & 0xffff0000, 64, drom_page_count, 0);
#endif
    ESP_LOGV(TAG, "rc=%d", rc);
#if CONFIG_IDF_TARGET_ESP32
    rc = cache_flash_mmu_set(1, 0, drom_load_addr_aligned, drom_addr & MMU_FLASH_MASK, 64, drom_page_count);
    ESP_LOGV(TAG, "rc=%d", rc);
#endif
    uint32_t irom_load_addr_aligned = irom_load_addr & MMU_FLASH_MASK;
    uint32_t irom_page_count = bootloader_cache_pages_to_map(irom_size, irom_load_addr);
    ESP_LOGV(TAG, "i mmu set paddr=%08x vaddr=%08x size=%d n=%d",
             irom_addr & MMU_FLASH_MASK, irom_load_addr_aligned, irom_size, irom_page_count);
#if CONFIG_IDF_TARGET_ESP32
    rc = cache_flash_mmu_set(0, 0, irom_load_addr_aligned, irom_addr & MMU_FLASH_MASK, 64, irom_page_count);
#elif CONFIG_IDF_TARGET_ESP32S2
    uint32_t iram1_used = 0;
    if (irom_load_addr + irom_size > IRAM1_ADDRESS_LOW) {
        iram1_used = 1;
    }
    if (iram1_used) {
        rc = Cache_Ibus_MMU_Set(MMU_ACCESS_FLASH, IRAM0_ADDRESS_LOW, 0, 64, 64, 1);
        rc = Cache_Ibus_MMU_Set(MMU_ACCESS_FLASH, IRAM1_ADDRESS_LOW, 0, 64, 64, 1);
        REG_CLR_BIT(EXTMEM_PRO_ICACHE_CTRL1_REG, EXTMEM_PRO_ICACHE_MASK_IRAM1);
    }
    rc = Cache_Ibus_MMU_Set(MMU_ACCESS_FLASH, irom_load_addr & 0xffff0000, irom_addr & 0xffff0000, 64, irom_page_count, 0);
#endif
    ESP_LOGV(TAG, "rc=%d", rc);
#if CONFIG_IDF_TARGET_ESP32
    rc = cache_flash_mmu_set(1, 0, irom_load_addr_aligned, irom_addr & MMU_FLASH_MASK, 64, irom_page_count);
    ESP_LOGV(TAG, "rc=%d", rc);
    DPORT_REG_CLR_BIT( DPORT_PRO_CACHE_CTRL1_REG,
                       (DPORT_PRO_CACHE_MASK_IRAM0) | (DPORT_PRO_CACHE_MASK_IRAM1 & 0) |
                       (DPORT_PRO_CACHE_MASK_IROM0 & 0) | DPORT_PRO_CACHE_MASK_DROM0 |
                       DPORT_PRO_CACHE_MASK_DRAM1 );
    DPORT_REG_CLR_BIT( DPORT_APP_CACHE_CTRL1_REG,
                       (DPORT_APP_CACHE_MASK_IRAM0) | (DPORT_APP_CACHE_MASK_IRAM1 & 0) |
                       (DPORT_APP_CACHE_MASK_IROM0 & 0) | DPORT_APP_CACHE_MASK_DROM0 |
                       DPORT_APP_CACHE_MASK_DRAM1 );
#elif CONFIG_IDF_TARGET_ESP32S2
    REG_CLR_BIT( EXTMEM_PRO_ICACHE_CTRL1_REG, (EXTMEM_PRO_ICACHE_MASK_IRAM0) | (EXTMEM_PRO_ICACHE_MASK_IRAM1 & 0) | EXTMEM_PRO_ICACHE_MASK_DROM0 );
#endif
#if CONFIG_IDF_TARGET_ESP32
    Cache_Read_Enable(0);
#elif CONFIG_IDF_TARGET_ESP32S2
    Cache_Resume_ICache(autoload);
#endif
    // Application will need to do Cache_Flush(1) and Cache_Read_Enable(1)

    ESP_LOGD(TAG, "start: 0x%08x", entry_addr);
    typedef void (*entry_t)(void) __attribute__((noreturn));
    entry_t entry = ((entry_t) entry_addr);

    // TODO: we have used quite a bit of stack at this point.
    // use "movsp" instruction to reset stack back to where ROM stack starts.
    (*entry)();
}

static void unpack_load_app(const esp_image_metadata_t *data)
{
    uint32_t drom_addr = 0;
    uint32_t drom_load_addr = 0;
    uint32_t drom_size = 0;
    uint32_t irom_addr = 0;
    uint32_t irom_load_addr = 0;
    uint32_t irom_size = 0;

    // Find DROM & IROM addresses, to configure cache mappings
    for (int i = 0; i < data->image.segment_count; i++) {
        const esp_image_segment_header_t *header = &data->segments[i];
        if (header->load_addr >= SOC_DROM_LOW && header->load_addr < SOC_DROM_HIGH) {
            if (drom_addr != 0) {
                ESP_LOGE(TAG, MAP_ERR_MSG, "DROM");
            } else {
                ESP_LOGD(TAG, "Mapping segment %d as %s", i, "DROM");
            }
            drom_addr = data->segment_data[i];
            drom_load_addr = header->load_addr;
            drom_size = header->data_len;
        }
        if (header->load_addr >= SOC_IROM_LOW && header->load_addr < SOC_IROM_HIGH) {
            if (irom_addr != 0) {
                ESP_LOGE(TAG, MAP_ERR_MSG, "IROM");
            } else {
                ESP_LOGD(TAG, "Mapping segment %d as %s", i, "IROM");
            }
            irom_addr = data->segment_data[i];
            irom_load_addr = header->load_addr;
            irom_size = header->data_len;
        }
    }

    ESP_LOGD(TAG, "calling set_cache_and_start_app");
    set_cache_and_start_app(drom_addr,
                            drom_load_addr,
                            drom_size,
                            irom_addr,
                            irom_load_addr,
                            irom_size,
                            data->image.entry_addr);
}

// Copy loaded segments to RAM, set up caches for mapped segments, and start application.
static void load_image(const esp_image_metadata_t *image_data)
{
    ESP_LOGI(TAG, "Disabling RNG early entropy source...");
    bootloader_random_disable();

    // copy loaded segments to RAM, set up caches for mapped segments, and start application
    unpack_load_app(image_data);
}

/**
 * @brief Load the selected partition and start application.
 *
 * Start from partition 'start_index', if not bootable then work backwards to FACTORY_INDEX
 * (ie try any OTA slots in descending order and then the factory partition).
 * If still nothing, start from 'start_index + 1' and work up to highest numbered OTA partition.
 * If still nothing, try TEST_APP_INDEX.
 * Everything this function calls must be located in the iram_loader_seg segment.
 *
 * @param[in] bs Bootloader state structure.
 * @param[in] start_index The index from which the search for images begins.
 */
__attribute__((noreturn)) static void bootloader_utility_load_boot_image(uint32_t partition_offset, uint32_t partition_size)
{
    esp_image_metadata_t image_data;

    ESP_LOGD(TAG, "Trying partition offs 0x%x size 0x%x", partition_offset, partition_size);
    if (try_load_partition(partition_offset, partition_size, &image_data)) {
        load_image(&image_data);
    }

    ESP_LOGE(TAG, "Factory app partition is not bootable");
    ESP_LOGE(TAG, "No bootable app partitions in the partition table");
    bzero(&image_data, sizeof(esp_image_metadata_t));
    bootloader_reset();
}

/*
 * We arrive here after the ROM bootloader finished loading this second stage bootloader from flash.
 * The hardware is mostly uninitialized, flash cache is down and the app CPU is in reset.
 * We do have a stack, so we can do the initialization in C.
 */
void __attribute__((noreturn)) call_start_cpu0(void)
{
    // 1. Hardware initialization
    if (bootloader_init() != ESP_OK) {
        bootloader_reset();
    }

    // 2. Load the app image for booting
    const uint32_t PARTITION_OFFSET = 0x10000; // 64 KiB
    const uint32_t PARTITION_SIZE = 0x200000; // 2 MiB
    bootloader_utility_load_boot_image(PARTITION_OFFSET, PARTITION_SIZE);
}

// Return global reent struct if any newlib functions are linked to bootloader
struct _reent *__getreent(void)
{
    return _GLOBAL_REENT;
}
