#set flags regarding the board
include(global_config)

# config flags
EN_FRTOS_FLAG()
EN_SRL_UART()

# config to select component, the format is CONFIG_USE_${component}
set(CONFIG_USE_driver_lpuart true)
