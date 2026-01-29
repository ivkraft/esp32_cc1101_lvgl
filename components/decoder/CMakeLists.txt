idf_component_register(
    SRCS "decoder.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_rmt esp_timer freertos
)