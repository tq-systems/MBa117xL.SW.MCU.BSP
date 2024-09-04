project(demo_ethernet_1g)

set(MCUX_SDK_PROJECT_NAME ${BOARD_NAME}_${PROJECT_NAME}.elf)

include(flags)

add_executable(${MCUX_SDK_PROJECT_NAME} 
    ${BOARD_SRCS}
    "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.c"
    "${TQ_DRIVERS_PATH}/NXP_API/DP83867.c"
    "${TQ_DRIVERS_PATH}/NXP_API/DP83867.h"
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${BoardDirPath}
    ${BoardDirPath}/ethernet/
    ${TQ_DRIVERS_PATH}/NXP_API/
)

#global inclusion

include(global_include)

# extra modules

list(APPEND CMAKE_MODULE_PATH ${MCUXSDK_ROOT}/core/components/silicon_id/socs/imxrt1170)

# include modules

include(component_silicon_id)

include(driver_common)

include(component_silicon_id_rt1170)

include(middleware_lwip_contrib_ping)

include(middleware_lwip_enet_ethernetif_MIMXRT1176_cm7)

include(middleware_lwip)

include(driver_phy-common)

include(driver_enet)

include(driver_clock)

include(device_CMSIS)

include(utility_debug_console)

include(component_lpuart_adapter)

include(component_serial_manager)

include(component_lists)

include(component_serial_manager_uart_MIMXRT1176_cm7)

include(driver_lpuart)

include(device_startup_MIMXRT1176_cm7)

include(driver_iomuxc)

include(utility_assert)

include(driver_igpio)

include(component_igpio_adapter)

include(driver_xip_device)

include(driver_pmu_1)

include(driver_dcdc_soc)

include(driver_cache_armv7_m7)

include(driver_anatop_ai)

include(CMSIS_Include_core_cm)

include(driver_memory)

include(utilities_misc_utilities)

include(device_system_MIMXRT1176_cm7)


if(NOT DEFINED TARGET_LINK_SYSTEM_LIBRARIES)  
    set(TARGET_LINK_SYSTEM_LIBRARIES "-lm -lc -lgcc -lnosys")  
endif()  

target_link_libraries(${MCUX_SDK_PROJECT_NAME}  PRIVATE -Wl,--start-group)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE ${TARGET_LINK_SYSTEM_LIBRARIES})

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--end-group)

add_custom_command(TARGET ${MCUX_SDK_PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_OBJCOPY}
-Obinary $<TARGET_FILE:${MCUX_SDK_PROJECT_NAME}> $<TARGET_FILE_DIR:${MCUX_SDK_PROJECT_NAME}>/${BOARD_NAME}_${PROJECT_NAME}.bin)
