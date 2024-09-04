project(demo_eeprom)

set(MCUX_SDK_PROJECT_NAME ${BOARD_NAME}_${PROJECT_NAME}.elf)

include(flags)

add_executable(${MCUX_SDK_PROJECT_NAME} 
${BOARD_SRCS}
"${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.c"
"${TQ_DRIVERS_PATH}/M24C64.c"
"${TQ_DRIVERS_PATH}/M24C64.h"
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/demo_apps
    ${TQ_DRIVERS_PATH}
    ${BoardDirPath}
)

#global inclusion

include(global_include)

# include modules

include(tq_utils)

include(lpi2c_api)

include(utility_debug_console_lite)

include(driver_igpio)

include(utility_assert_lite)

include(driver_lpi2c)

include(driver_clock)

include(driver_common)

include(driver_rdc)

include(device_CMSIS)

include(driver_xip_device)

include(driver_pmu_1)

include(driver_dcdc_soc)

include(driver_cache_armv7_m7)

include(driver_anatop_ai)

include(driver_iomuxc)

include(component_lpuart_adapter)

include(component_lists)

include(driver_lpuart)

include(device_startup_MIMXRT1176_cm7)

include(CMSIS_Include_core_cm)

include(utilities_misc_utilities_MIMX8ML8)

include(device_system_MIMXRT1176_cm7)


if(NOT DEFINED TARGET_LINK_SYSTEM_LIBRARIES)  
    set(TARGET_LINK_SYSTEM_LIBRARIES "-lm -lc -lgcc -lnosys")  
endif()  

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--start-group)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE ${TARGET_LINK_SYSTEM_LIBRARIES})

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--end-group)

add_custom_command(TARGET ${MCUX_SDK_PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_OBJCOPY}
-Obinary $<TARGET_FILE:${MCUX_SDK_PROJECT_NAME}> $<TARGET_FILE_DIR:${MCUX_SDK_PROJECT_NAME}>/${BOARD_NAME}_${PROJECT_NAME}.bin)
