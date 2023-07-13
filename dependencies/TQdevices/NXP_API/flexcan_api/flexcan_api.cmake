#Description: API for NXP's device, user_visible: True
include_guard(DIRECTORY)
message("TQ - flexcan_api.")

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/.
)
