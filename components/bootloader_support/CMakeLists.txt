set(srcs
    "src/bootloader_clock.c"
    "src/bootloader_common.c"
    "src/bootloader_flash.c"
    "src/bootloader_mem.c"
    "src/bootloader_random.c"
    "src/esp_image_format.c"
    "src/flash_partitions.c"
    "src/flash_qio_mode.c"
    "src/bootloader_flash_config_${IDF_TARGET}.c"
    "src/bootloader_efuse_${IDF_TARGET}.c"
    )

if(BOOTLOADER_BUILD)
    set(include_dirs "include" "include_bootloader")
    set(priv_requires spi_flash efuse)
    list(APPEND srcs
    "src/bootloader_init.c"
    "src/${IDF_TARGET}/bootloader_sha.c"
    "src/${IDF_TARGET}/bootloader_${IDF_TARGET}.c"
    )
else()
    list(APPEND srcs
        "src/idf/bootloader_sha.c")
    set(include_dirs "include")
    set(priv_include_dirs "include_bootloader")
    set(priv_requires spi_flash mbedtls efuse)
endif()

set(requires soc) #unfortunately the header directly uses SOC registers

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "${include_dirs}"
                    PRIV_INCLUDE_DIRS "${priv_include_dirs}"
                    REQUIRES "${requires}"
                    PRIV_REQUIRES "${priv_requires}")
