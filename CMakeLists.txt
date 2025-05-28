# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "D:/Documents/ESP_projects/ESP-IDF_5.3.2/genre-classification-esp32/components/u8g2")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
#set(COMPONENTS main)
project(test_fft)
