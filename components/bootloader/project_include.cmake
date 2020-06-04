set(BOOTLOADER_OFFSET 0x1000)

# Do not generate flash file when building bootloader
if(BOOTLOADER_BUILD OR NOT CONFIG_APP_BUILD_BOOTLOADER)
    return()
endif()

# Glue to build the bootloader subproject binary as an external
# cmake project under this one
#
#
idf_build_get_property(build_dir BUILD_DIR)
set(BOOTLOADER_BUILD_DIR "${build_dir}/bootloader")
set(bootloader_binary_files
    "${BOOTLOADER_BUILD_DIR}/bootloader.elf"
    "${BOOTLOADER_BUILD_DIR}/bootloader.bin"
    "${BOOTLOADER_BUILD_DIR}/bootloader.map"
    )

idf_build_get_property(project_dir PROJECT_DIR)

set(sign_key_arg)
set(ver_key_arg)

idf_build_get_property(idf_path IDF_PATH)
idf_build_get_property(idf_target IDF_TARGET)
idf_build_get_property(sdkconfig SDKCONFIG)
idf_build_get_property(python PYTHON)
idf_build_get_property(extra_cmake_args EXTRA_CMAKE_ARGS)

externalproject_add(bootloader
    SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/subproject"
    BINARY_DIR "${BOOTLOADER_BUILD_DIR}"
    CMAKE_ARGS  -DSDKCONFIG=${sdkconfig} -DIDF_PATH=${idf_path} -DIDF_TARGET=${idf_target}
                -DPYTHON_DEPS_CHECKED=1 -DPYTHON=${python}
                -DEXTRA_COMPONENT_DIRS=${CMAKE_CURRENT_LIST_DIR}
                ${sign_key_arg} ${ver_key_arg}
                # LEGACY_INCLUDE_COMMON_HEADERS has to be passed in via cache variable since
                # the bootloader common component requirements depends on this and
                # config variables are not available before project() call.
                -DLEGACY_INCLUDE_COMMON_HEADERS=${CONFIG_LEGACY_INCLUDE_COMMON_HEADERS}
                ${extra_cmake_args}
    INSTALL_COMMAND ""
    BUILD_ALWAYS 1  # no easy way around this...
    BUILD_BYPRODUCTS ${bootloader_binary_files}
    )

if(CONFIG_SECURE_SIGNED_APPS)
    add_dependencies(bootloader gen_secure_boot_keys)
endif()

# this is a hack due to an (annoying) shortcoming in cmake, it can't
# extend the 'clean' target to the external project
# see thread: https://cmake.org/pipermail/cmake/2016-December/064660.html
#
# So for now we just have the top-level build remove the final build products...
set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND PROPERTY
    ADDITIONAL_MAKE_CLEAN_FILES
    ${bootloader_binary_files})
