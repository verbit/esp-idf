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
#pragma once

#include "bootloader_config.h"
#include "esp_image_format.h"
#include "bootloader_config.h"

/** @brief Generates the digest of the data between offset & offset+length.
 *
 * This function should be used when the size of the data is larger than 3.2MB.
 * The MMU capacity is 3.2MB (50 pages - 64KB each). This function generates the SHA-256
 * of the data in chunks of 3.2MB, considering the MMU capacity.
 *
 * @param[in]  flash_offset  Offset of the data in flash.
 * @param[in]  len           Length of data in bytes.
 * @param[out] digest        Pointer to buffer where the digest is written, if ESP_OK is returned.
 *
 * @return ESP_OK if secure boot digest is generated successfully.
 */
esp_err_t bootloader_sha256_flash_contents(uint32_t flash_offset, uint32_t len, uint8_t *digest);
