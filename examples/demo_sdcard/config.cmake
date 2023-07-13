#set flags regarding the board
include(global_config)

# config application flags
EN_SRL_UART()
EN_FLOAT_ADVANCED_PRINT(1)
SD_ENABLE()
# config to select component, the format is CONFIG_USE_${component}
set(CONFIG_USE_middleware_sdmmc_host_usdhc true)
set(CONFIG_USE_middleware_sdmmc_host_usdhc_interrupt_MIMXRT1176_cm7 true)
set(CONFIG_USE_driver_lpuart true)
