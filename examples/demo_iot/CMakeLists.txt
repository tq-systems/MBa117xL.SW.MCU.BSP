project(demo_iot)

set(MCUX_SDK_PROJECT_NAME ${BOARD_NAME}_${PROJECT_NAME}.elf)

include(flags)

add_executable(${MCUX_SDK_PROJECT_NAME} 
"${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.c"
${BOARD_SRCS}
"${TQ_DRIVERS_PATH}/PCA9555BS.c"
"${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOSConfig.h"
"${CMAKE_CURRENT_SOURCE_DIR}/iot.h"
"${CMAKE_CURRENT_SOURCE_DIR}/iot.c"
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${BoardDirPath}
    ${TQ_DRIVERS_PATH}
)

#global inclusion

include(global_include)

# include modules
include(tq_utils)

include(lpuart_api)

include (driver_lpi2c)

include (lpi2c_api)

include(utility_debug_console_lite)

include(utility_assert_lite)

include(driver_flexcan)

include(driver_clock)

include(driver_common)

include(device_CMSIS)

include(component_lpuart_adapter)

include(component_lists)

include(driver_dcdc_soc)

include(driver_cache_armv7_m7)

include(driver_xip_device)

include(driver_lpuart)

include(device_startup_MIMXRT1176_cm7)

include(driver_iomuxc)

include(driver_igpio)

include(driver_lpuart_freertos)

include(middleware_freertos-kernel_heap_4)

include(middleware_freertos-kernel_MIMXRT1176_cm7)

include(middleware_freertos-kernel_extension)

include(driver_pmu_1)

include(driver_anatop_ai)

include(CMSIS_Include_core_cm)

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
