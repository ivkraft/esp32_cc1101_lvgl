idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_lcd esp_lvgl_port lvgl knob button esp_driver_spi cc1101 decoder bq27220 bq25896 RTC)
