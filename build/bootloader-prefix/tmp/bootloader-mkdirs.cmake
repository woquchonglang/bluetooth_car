# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/software/esp32_5.0/Espressif/frameworks/esp-idf-v5.0.4/components/bootloader/subproject"
  "H:/ESP32/engine/1/gatt_server/build/bootloader"
  "H:/ESP32/engine/1/gatt_server/build/bootloader-prefix"
  "H:/ESP32/engine/1/gatt_server/build/bootloader-prefix/tmp"
  "H:/ESP32/engine/1/gatt_server/build/bootloader-prefix/src/bootloader-stamp"
  "H:/ESP32/engine/1/gatt_server/build/bootloader-prefix/src"
  "H:/ESP32/engine/1/gatt_server/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "H:/ESP32/engine/1/gatt_server/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "H:/ESP32/engine/1/gatt_server/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
