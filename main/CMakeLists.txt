idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_lcd esp_lvgl_port lvgl knob button driver cc1101 decoder bq27220 bq25896)
