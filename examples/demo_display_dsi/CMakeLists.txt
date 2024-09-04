project(demo_display_dsi)

set(MCUX_SDK_PROJECT_NAME ${BOARD_NAME}_${PROJECT_NAME}.elf)

include(flags)
include(global_config)

add_executable(${MCUX_SDK_PROJECT_NAME} 
    ${BOARD_SRCS}
    "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/display_support.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOSConfig.h"
    "${TQ_DRIVERS_PATH}/PCA9555BS.c"
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${TQ_DRIVERS_PATH}
    ${BoardDirPath}
)

set_source_files_properties("${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOSConfig.h" PROPERTIES COMPONENT_CONFIG_FILE "middleware_freertos-kernel_template")

#global inclusion

include(global_include)

include(all_lib_device_${MCUX_DEVICE}_c${MCPU_SUFFIX})
include(all_lib_device)

include(lpi2c_api)

IF(NOT DEFINED TARGET_LINK_SYSTEM_LIBRARIES)  
    SET(TARGET_LINK_SYSTEM_LIBRARIES "-lm -lc -lgcc -lnosys")  
ENDIF()  

TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--start-group)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE ${TARGET_LINK_SYSTEM_LIBRARIES})

TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--end-group)

set_target_properties(${MCUX_SDK_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES "output.map")

add_custom_command(TARGET ${MCUX_SDK_PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_OBJCOPY}
-Obinary $<TARGET_FILE:${MCUX_SDK_PROJECT_NAME}> $<TARGET_FILE_DIR:${MCUX_SDK_PROJECT_NAME}>/${BOARD_NAME}_${PROJECT_NAME}.bin)
