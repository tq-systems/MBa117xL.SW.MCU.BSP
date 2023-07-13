#set flags regarding the board
include(global_config)

# config application flags
EN_SRL_UART()
EN_FLOAT_ADVANCED_PRINT(0)
# config to select component, the format is CONFIG_USE_${component}
set(CONFIG_USE_driver_lpuart true)
