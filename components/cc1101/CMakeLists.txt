idf_component_register(
    SRCS
        "cc1101.c"
        "cc1101_presets.c"
    INCLUDE_DIRS
        "include"
    PRIV_REQUIRES
        esp_driver_spi esp_driver_gpio
)