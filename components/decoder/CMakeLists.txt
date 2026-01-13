idf_component_register(
    SRCS "decoder.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer freertos
)