#set flags regarding the board
include(global_config)

# config application flags
EN_SRL_UART()
EN_FLOAT_ADVANCED_PRINT(1)

# config to select component, the format is CONFIG_USE_${component}
set(CONFIG_USE_component_silicon_id_rt1170 true)
set(CONFIG_USE_driver_enet true)
set(CONFIG_USE_middleware_lwip_enet_ethernetif_MIMXRT1176_cm7 true)
set(CONFIG_USE_component_serial_manager_uart_MIMXRT1176_cm7 true)
set(CONFIG_USE_driver_lpuart true)
