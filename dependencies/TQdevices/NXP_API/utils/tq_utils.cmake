#Description: API for NXP's device, user_visible: True
include_guard(DIRECTORY)
message("TQ utils")

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/TQ_utils.h
    ${CMAKE_CURRENT_LIST_DIR}/TQ_utils.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/.
)
