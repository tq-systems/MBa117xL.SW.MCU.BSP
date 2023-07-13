set(BOARD_EXTENSION_SRCS
    "${BoardDirPath}/PMIC/PMIC.c"
)

set(BOARD_EXTENSION_HEADERS
    "${BoardDirPath}/PMIC/PMIC.h"
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
    ${BoardDirPath}/PMIC
)


target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE ${BOARD_EXTENSION_SRCS})
target_sources(${MCUX_SDK_PROJECT_NAME} PUBLIC ${BOARD_EXTENSION_HEADERS})

message("Added board extensions.")

if(EXISTS "${CMAKE_SOURCE_DIR}/version.h")
    add_compile_definitions(-DVERSION_INFORMATION=1)
    target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
    ${CMAKE_SOURCE_DIR})
    target_sources(${MCUX_SDK_PROJECT_NAME} PUBLIC "${CMAKE_SOURCE_DIR}/version.h")
    message("version.h included.")
endif()

include(driver_lpi2c)
