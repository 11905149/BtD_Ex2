# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Set project components and additional component directories based on the target
if("${IDF_TARGET}" STREQUAL "linux")
    set(COMPONENTS main)
endif()
set(EXTRA_COMPONENT_DIRS components)

# Include ESP-IDF project cmake configuration
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
include_directories(./components/st7789)
include_directories(./components/I2Cdev)
include_directories(./components/MPU6050)

# Define your project name
project(tcp_client)


# Configure SPIFFS partition images for fonts and images
spiffs_create_partition_image(storage1 fonts FLASH_IN_PROJECT)
spiffs_create_partition_image(storage2 images FLASH_IN_PROJECT)
