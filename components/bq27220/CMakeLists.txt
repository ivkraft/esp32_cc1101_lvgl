idf_component_register(
    SRCS
        "bq27220.c"
    INCLUDE_DIRS
        "include"
    PRIV_REQUIRES
        esp_driver_i2c
)