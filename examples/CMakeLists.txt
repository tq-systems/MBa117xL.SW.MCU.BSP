set(MCUX_DEVICE "MIMXRT1176")
set(MCPU_SUFFIX "m7")
set(CORE_TYPE "MIMXRT1176DVMAA")
set(CONFIG_CORE c${MCPU_SUFFIX}f)
set(CONFIG_DEVICE ${MCUX_DEVICE})
set(CONFIG_DEVICE_ID ${MCUX_DEVICE}xxxxx)
set(CONFIG_CORE_ID c${MCPU_SUFFIX})

set(BoardDirPath ${CMAKE_CURRENT_SOURCE_DIR}/board)
list(APPEND CMAKE_MODULE_PATH "${BoardDirPath}/cmake")

set(LINKER_FILE_RAM "${BoardDirPath}/MIMXRT1176xxxxx_cm7_ram.ld")
set(LINKER_FILE_FLASH "${BoardDirPath}/MIMXRT1176xxxxx_cm7_flexspi_nor.ld")
set(LINKER_FILE_SDRAM "${BoardDirPath}/MIMXRT1176xxxxx_cm7_flexspi_nor_sdram.ld")

include(target)

if(bootDisk STREQUAL "SD-Ram-Flash")
 set(BOARD_SRCS ${BOARD_SRCS}
    "${BoardDirPath}/dcd.c"
    "${BoardDirPath}/dcd.h"
 )
endif()

if(bootDisk STREQUAL "Flash" OR bootDisk STREQUAL "SD-Ram-Flash")
    set(BOARD_SRCS ${BOARD_SRCS} 
    "${BoardDirPath}/TQMa117xL_flexspi_nor_config.c"
    "${BoardDirPath}/TQMa117xL_flexspi_nor_config.h"
    )
endif()

add_subdirectory(demo_hello_world)
add_subdirectory(demo_flexcan)
add_subdirectory(demo_gpio_led)
add_subdirectory(demo_digital_io)
add_subdirectory(demo_afe)
add_subdirectory(demo_SE97BTP)
add_subdirectory(demo_rs485)
add_subdirectory(demo_sdcard)
add_subdirectory(demo_RTC)
add_subdirectory(demo_eeprom)
add_subdirectory(demo_iot)
add_subdirectory(demo_usb)
add_subdirectory(demo_PMIC)
add_subdirectory(demo_spi_flash)
add_subdirectory(demo_ethernet_1g)
add_subdirectory(demo_ethernet_qos)
if(bootDisk STREQUAL "SD-Ram-Flash")
    add_subdirectory(demo_display_dsi)
    add_subdirectory(demo_display_lvds)
endif()
