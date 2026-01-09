# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ProjectSlave_esp32)

# Paths
set(SLAVE_BINARIES_DIR "${CMAKE_SOURCE_DIR}/../../../slave_init_bo/binaries")
set(SPIFFS_IMAGE_DIR "${CMAKE_SOURCE_DIR}/spiffs_image_master")

# Ensure SPIFFS directory exists
file(MAKE_DIRECTORY "${SPIFFS_IMAGE_DIR}")

# Define the target file path in SPIFFS
set(SPIFFS_FW_SOURCE "${SLAVE_BINARIES_DIR}/version1.bin")
set(SPIFFS_FW_DEST "${SPIFFS_IMAGE_DIR}/slave.bin")
set(COPY_STAMP "${CMAKE_CURRENT_BINARY_DIR}/.copy_fw_stamp")

# Create custom command to copy firmware when it changes
add_custom_command(
    OUTPUT "${COPY_STAMP}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${SPIFFS_FW_SOURCE}" "${SPIFFS_FW_DEST}"
    COMMAND ${CMAKE_COMMAND} -E touch "${COPY_STAMP}"
    DEPENDS "${SPIFFS_FW_SOURCE}"
    COMMENT "Copiando version1.bin a SPIFFS"
    VERBATIM
)

# Create custom target
add_custom_target(copy_fw_to_spiffs ALL DEPENDS "${COPY_STAMP}")

# Create SPIFFS partition image
spiffs_create_partition_image(storage ${SPIFFS_IMAGE_DIR} FLASH_IN_PROJECT)

# Ensure copy happens before SPIFFS image is created
add_dependencies(spiffs_storage_bin copy_fw_to_spiffs)