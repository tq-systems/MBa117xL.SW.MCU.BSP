project(demo_usb)

set(MCUX_SDK_PROJECT_NAME ${BOARD_NAME}_${PROJECT_NAME}.elf)

include(flags)

add_executable(${MCUX_SDK_PROJECT_NAME} 
${BOARD_SRCS}
"${CMAKE_CURRENT_SOURCE_DIR}/host_msd_command.c"
"${CMAKE_CURRENT_SOURCE_DIR}/host_msd_command.h"
"${CMAKE_CURRENT_SOURCE_DIR}/usb_host_config.h"
"${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.c"
"${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOSConfig.h"
"${TQ_DRIVERS_PATH}/PCA9555BS.c"
"${TQ_DRIVERS_PATH}/PCA9555BS.h"
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/demo_apps/
    ${BoardDirPath}
    ${TQ_DRIVERS_PATH}
)

#global inclusion

include(global_include)

# include modules
include(middleware_usb_host_ehci_MIMXRT1176_cm7)

include(middleware_usb_phy)

include(middleware_freertos-kernel_heap_4)

include(middleware_freertos-kernel_MIMXRT1176_cm7)

include(middleware_usb_host_msd)

include(driver_clock)

include(driver_common)

include(device_CMSIS)

include(utility_debug_console)

include(driver_igpio)

include(component_lpuart_adapter)

include(component_serial_manager)

include(component_lists)

include(component_serial_manager_uart_MIMXRT1176_cm7)

include(driver_lpuart)

include(device_startup_MIMXRT1176_cm7)

include(driver_iomuxc)

include(utility_assert)

include(driver_igpio)

include(driver_xip_device)

include(middleware_usb_host_stack_MIMXRT1176_cm7)

include(component_osa)

include(driver_pmu_1)

include(driver_dcdc_soc)

include(driver_cache_armv7_m7)

include(middleware_usb_common_header)

include(driver_anatop_ai)

include(CMSIS_Include_core_cm)

include(driver_memory)

include(middleware_usb_host_common_header)

include(middleware_freertos-kernel_extension)

include(utilities_misc_utilities)

include(device_system_MIMXRT1176_cm7)

include (driver_lpi2c)

include (lpi2c_api)


if(NOT DEFINED TARGET_LINK_SYSTEM_LIBRARIES)  
    set(TARGET_LINK_SYSTEM_LIBRARIES "-lm -lc -lgcc -lnosys")  
endif()  

target_link_libraries(${MCUX_SDK_PROJECT_NAME}  PRIVATE -Wl,--start-group)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE ${TARGET_LINK_SYSTEM_LIBRARIES})

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--end-group)

add_custom_command(TARGET ${MCUX_SDK_PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_OBJCOPY}
-Obinary $<TARGET_FILE:${MCUX_SDK_PROJECT_NAME}> $<TARGET_FILE_DIR:${MCUX_SDK_PROJECT_NAME}>/${BOARD_NAME}_${PROJECT_NAME}.bin)
