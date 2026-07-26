# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(IDF_TARGET "esp32s3" CACHE STRING "ESP-IDF target")
set(COMPONENTS main esp_psram)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(navigation_robot)
