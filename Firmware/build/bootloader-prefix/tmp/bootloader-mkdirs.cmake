# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/ESP/v5.3/esp-idf/components/bootloader/subproject"
  "D:/Cambridge_Sensor_CDT/SensorCDT_Microfluidic_Project/Firmware/build/bootloader"
  "D:/Cambridge_Sensor_CDT/SensorCDT_Microfluidic_Project/Firmware/build/bootloader-prefix"
  "D:/Cambridge_Sensor_CDT/SensorCDT_Microfluidic_Project/Firmware/build/bootloader-prefix/tmp"
  "D:/Cambridge_Sensor_CDT/SensorCDT_Microfluidic_Project/Firmware/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Cambridge_Sensor_CDT/SensorCDT_Microfluidic_Project/Firmware/build/bootloader-prefix/src"
  "D:/Cambridge_Sensor_CDT/SensorCDT_Microfluidic_Project/Firmware/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Cambridge_Sensor_CDT/SensorCDT_Microfluidic_Project/Firmware/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Cambridge_Sensor_CDT/SensorCDT_Microfluidic_Project/Firmware/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
