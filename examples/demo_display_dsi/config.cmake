# config to select component, the format is CONFIG_USE_${component}
# Please refer to cmake files below to get available components:
#  ${SdkRootDirPath}/devices/MIMXRT1176/all_lib_device.cmake

# Device Configuration
set(CONFIG_USE_component_silicon_id true)
set(CONFIG_USE_component_silicon_id_rt1170 true)
set(CONFIG_USE_device_MIMXRT1176_CMSIS true)
set(CONFIG_USE_device_MIMXRT1176_system true)
set(CONFIG_USE_device_MIMXRT1176_startup true)
set(CONFIG_BOARD evkbmimxrt1170)
set(CONFIG_KIT evkbmimxrt1170)

# Driver Configuration
set(CONFIG_USE_driver_common true)
set(CONFIG_USE_driver_clock true)
set(CONFIG_USE_driver_memory true)
set(CONFIG_USE_driver_anatop_ai true)
set(CONFIG_USE_driver_pmu_1 true)
set(CONFIG_USE_driver_video-common true)
set(CONFIG_USE_driver_display-common true)
set(CONFIG_USE_driver_display-rm68191 true)
set(CONFIG_USE_driver_mipi_dsi_split true)
set(CONFIG_USE_driver_display-mipi-dsi-cmd true)
set(CONFIG_USE_driver_display-rm68200 true)
set(CONFIG_USE_driver_soc_src true)
set(CONFIG_USE_driver_lcdifv2 true)
set(CONFIG_USE_driver_dc-fb-lcdifv2 true)
set(CONFIG_USE_driver_elcdif true)
set(CONFIG_USE_driver_dc-fb-elcdif true)
set(CONFIG_USE_driver_display-hx8394 true)
set(CONFIG_USE_driver_fbdev true)
set(CONFIG_USE_driver_iomuxc true)
set(CONFIG_USE_driver_igpio true)
set(CONFIG_USE_driver_xip_device true)
set(CONFIG_USE_driver_xip_board_evkbmimxrt1170 true)
set(CONFIG_USE_driver_dcdc_soc true)
set(CONFIG_USE_driver_cache_armv7_m7 true)
set(CONFIG_USE_driver_xmcd_evkbmimxrt1170 true)
set(CONFIG_USE_driver_soc_mipi_csi2rx true)
set(CONFIG_USE_driver_soc_mipi_dsi true)
set(CONFIG_USE_driver_dc-fb-common true)
set(CONFIG_USE_driver_lpuart true)

# Middleware Configuration
set(CONFIG_USE_middleware_freertos-kernel true)
set(CONFIG_USE_middleware_freertos-kernel_heap_4 true)
set(CONFIG_USE_middleware_freertos-kernel_template true)
set(CONFIG_USE_middleware_freertos-kernel_extension true)

# Utility Configuration
set(CONFIG_USE_utility_debug_console true)
set(CONFIG_USE_utility_assert true)
set(CONFIG_USE_utilities_misc_utilities true)
set(CONFIG_USE_utility_str true)

# Component Configuration
set(CONFIG_USE_component_lists true)
set(CONFIG_USE_component_lpuart_adapter true)
set(CONFIG_USE_component_serial_manager_uart true)
set(CONFIG_USE_component_serial_manager true)

# CMSIS Include Configuration
set(CONFIG_USE_CMSIS_Include_core_cm true)

# Other Configuration
set(CONFIG_USE_COMPONENT_CONFIGURATION true)


add_compile_definitions(SERIAL_PORT_TYPE_UART=1)
add_compile_definitions(USE_TQ_LPI2C)
