# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/software/esp32_5.0/Espressif/frameworks/esp-idf-v5.0.4/components/bootloader/subproject"
  "H:/engine/scnu_robot/gatt_server/build/bootloader"
  "H:/engine/scnu_robot/gatt_server/build/bootloader-prefix"
  "H:/engine/scnu_robot/gatt_server/build/bootloader-prefix/tmp"
  "H:/engine/scnu_robot/gatt_server/build/bootloader-prefix/src/bootloader-stamp"
  "H:/engine/scnu_robot/gatt_server/build/bootloader-prefix/src"
  "H:/engine/scnu_robot/gatt_server/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "H:/engine/scnu_robot/gatt_server/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "H:/engine/scnu_robot/gatt_server/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
