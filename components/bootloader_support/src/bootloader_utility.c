// Copyright 2018 Espressif Systems (Shanghai) PTE LTD
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
#include <string.h>
#include <stdint.h>
#include <limits.h>
#include <sys/param.h>

#include "esp_attr.h"
#include "esp_log.h"

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

#include "soc/soc.h"
#include "soc/cpu.h"
#include "soc/rtc.h"
#include "soc/dport_reg.h"
#include "soc/gpio_periph.h"
#include "soc/efuse_periph.h"
#include "soc/rtc_periph.h"
#include "soc/timer_periph.h"

#include "sdkconfig.h"
#include "esp_flash_partitions.h"
#include "bootloader_flash.h"
#include "bootloader_random.h"
#include "bootloader_config.h"
#include "bootloader_common.h"
#include "bootloader_utility.h"
#include "bootloader_sha.h"
#include "esp_efuse.h"

static const char *TAG = "boot";

esp_err_t bootloader_sha256_flash_contents(uint32_t flash_offset, uint32_t len, uint8_t *digest)
{
    if (digest == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Handling firmware images larger than MMU capacity */
    uint32_t mmu_free_pages_count = bootloader_mmap_get_free_pages();
    bootloader_sha256_handle_t sha_handle = NULL;

    sha_handle = bootloader_sha256_start();
    if (sha_handle == NULL) {
        return ESP_ERR_NO_MEM;
    }

    while (len > 0) {
        uint32_t mmu_page_offset = ((flash_offset & MMAP_ALIGNED_MASK) != 0) ? 1 : 0; /* Skip 1st MMU Page if it is already populated */
        uint32_t partial_image_len = MIN(len, ((mmu_free_pages_count - mmu_page_offset) * SPI_FLASH_MMU_PAGE_SIZE)); /* Read the image that fits in the free MMU pages */

        const void * image = bootloader_mmap(flash_offset, partial_image_len);
        if (image == NULL) {
            bootloader_sha256_finish(sha_handle, NULL);
            return ESP_FAIL;
        }
        bootloader_sha256_data(sha_handle, image, partial_image_len);
        bootloader_munmap(image);

        flash_offset += partial_image_len;
        len -= partial_image_len;
    }
    bootloader_sha256_finish(sha_handle, digest);
    return ESP_OK;
}
