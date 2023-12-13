set(CMAKE_BINARY_DIR "${CMAKE_BINARY_DIR}/dist/${CMAKE_BUILD_TYPE}/${bootDisk}")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

message("BINARY PATH: ${EXECUTABLE_OUTPUT_PATH}")

list(APPEND CMAKE_MODULE_PATH
    ${TQ_DRIVERS_PATH}/NXP_API/i2c_api
    ${TQ_DRIVERS_PATH}/NXP_API/flexcan_api
    ${TQ_DRIVERS_PATH}/NXP_API/ecspi_api
    ${TQ_DRIVERS_PATH}/NXP_API/lpi2c_api
    ${TQ_DRIVERS_PATH}/NXP_API/lpspi_api
    ${TQ_DRIVERS_PATH}/NXP_API/lpuart_api
    ${TQ_DRIVERS_PATH}/NXP_API/utils
    ${MCUXSDK_ROOT}/core/
    ${MCUXSDK_ROOT}/core/drivers/common
    ${MCUXSDK_ROOT}/core/drivers/enet
    ${MCUXSDK_ROOT}/core/drivers/enet_qos
    ${MCUXSDK_ROOT}/core/components/lists
    ${MCUXSDK_ROOT}/core/components/osa
    ${MCUXSDK_ROOT}/core/components/phy
    ${MCUXSDK_ROOT}/core/components/phy/device/phyrtl8201
    ${MCUXSDK_ROOT}/core/components/phy/device/phyrtl8211f
    ${MCUXSDK_ROOT}/core/components/phy/mdio/enet
    ${MCUXSDK_ROOT}/core/components/phy/mdio/enet_qos
    ${MCUXSDK_ROOT}/core/components/serial_manager
    ${MCUXSDK_ROOT}/core/components/silicon_id
    ${MCUXSDK_ROOT}/core/components/uart
    ${MCUXSDK_ROOT}/core/components/gpio
    ${MCUXSDK_ROOT}/core/CMSIS/Core/Include
    ${MCUXSDK_ROOT}/core/drivers/lpspi
    ${MCUXSDK_ROOT}/core/drivers/ecspi
    ${MCUXSDK_ROOT}/core/drivers/flexcan
    ${MCUXSDK_ROOT}/core/drivers/igpio
    ${MCUXSDK_ROOT}/core/drivers/ii2c
    ${MCUXSDK_ROOT}/core/drivers/irtc
    ${MCUXSDK_ROOT}/core/drivers/iuart
    ${MCUXSDK_ROOT}/core/drivers/lpi2c
    ${MCUXSDK_ROOT}/core/drivers/lpuart
    ${MCUXSDK_ROOT}/core/drivers/mu
    ${MCUXSDK_ROOT}/core/drivers/pit
    ${MCUXSDK_ROOT}/core/drivers/port
    ${MCUXSDK_ROOT}/core/drivers/rdc
    ${MCUXSDK_ROOT}/core/drivers/sim
    ${MCUXSDK_ROOT}/core/drivers/smc
    ${MCUXSDK_ROOT}/core/drivers/snvs_lp
    ${MCUXSDK_ROOT}/core/drivers/snvs_hp
    ${MCUXSDK_ROOT}/core/drivers/usdhc
    ${MCUXSDK_ROOT}/core/utilities
    ${MCUXSDK_ROOT}/core/utilities/assert
    ${MCUXSDK_ROOT}/core/utilities/debug_console
    ${MCUXSDK_ROOT}/core/utilities/misc_utilities
    ${MCUXSDK_ROOT}/middleware/lwip
    ${MCUXSDK_ROOT}/middleware/multicore
    ${MCUXSDK_ROOT}/middleware/sdmmc
    ${MCUXSDK_ROOT}/middleware/usb
    ${MCUXSDK_ROOT}/rtos/freertos/freertos-kernel
)

list(APPEND CMAKE_MODULE_PATH
    ${MCUXSDK_ROOT}/core/devices/${MCUX_DEVICE}
    ${MCUXSDK_ROOT}/core/devices/${MCUX_DEVICE}/drivers
    ${MCUXSDK_ROOT}/core/devices/${MCUX_DEVICE}/utilities
    ${MCUXSDK_ROOT}/core/devices/${MCUX_DEVICE}/drivers/c${MCPU_SUFFIX}
    ${MCUXSDK_ROOT}/core/devices/${MCUX_DEVICE}/xip
)

set(BOARD_SRCS
    "${BoardDirPath}/pin_mux.c"
    "${BoardDirPath}/pin_mux.h"
    "${BoardDirPath}/board.c"
    "${BoardDirPath}/board.h"
    "${BoardDirPath}/clock_config.c"
    "${BoardDirPath}/clock_config.h"
)
