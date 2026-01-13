#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lvgl_port.h"
#include "driver/pulse_cnt.h"



#include "cc1101.h"
#include "cc1101_presets.h"
#include "cc1101_regs.h"

#include "decoder.h"


static const char *TAG = "main";

// TTGO T-Display Pin Definitions
#define LCD_HOST SPI2_HOST
#define PIN_NUM_SCLK GPIO_NUM_11
#define PIN_NUM_MOSI GPIO_NUM_9
#define PIN_NUM_MISO GPIO_NUM_10
#define PIN_NUM_DC GPIO_NUM_16
#define PIN_NUM_RST GPIO_NUM_40
#define PIN_NUM_CS GPIO_NUM_41
#define PIN_NUM_BK_LIGHT GPIO_NUM_21

#define PIN_POWER_EN 15

#define LCD_H_RES 170
#define LCD_V_RES 320

// Ваши пины для T-Embed
#define ENCODER_A GPIO_NUM_4
#define ENCODER_B GPIO_NUM_5
#define ENCODER_KEY GPIO_NUM_0
#define KEY_ESC GPIO_NUM_6

#define PIN_CC_CS 12  // manual CS
#define PIN_CC_GDO0 3
#define PIN_CC_GDO2 38

static pcnt_unit_handle_t pcnt_unit = NULL;
static lv_group_t *menu_group;


// Храним “шаги” (detents), а не сырые counts
static int last_steps = 0;

// Если окажется, что направление инвертировано — поставь 1
static const int invert_dir = 1;



void cc1101_power_on(bool on) {

    gpio_config_t pwr_cfg = {
        .pin_bit_mask = 1ULL << PIN_POWER_EN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&pwr_cfg);
    gpio_set_level(PIN_POWER_EN, on); // ВКЛЮЧАЕМ ПИТАНИЕ cc1101
    vTaskDelay(pdMS_TO_TICKS(50));
}

static void encoder_gpio_init(void)
{
    gpio_config_t io = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    io.pin_bit_mask = (1ULL << ENCODER_A) | (1ULL << ENCODER_B) | (1ULL << ENCODER_KEY);
    gpio_config(&io);
}


// Инициализация аппаратного счетчика PCNT для энкодера

void init_hardware_encoder(void)
{
    encoder_gpio_init();

    // 1) PCNT unit
    pcnt_unit_config_t unit_config = {
        .low_limit  = -32768,
        .high_limit =  32767,
        // .accum_count = false, // можно включить, если хочешь расширение
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    // 2) Glitch filter (в NS). Ставить ДО enable. :contentReference[oaicite:1]{index=1}
    pcnt_glitch_filter_config_t flt = {
        .max_glitch_ns = 1500, // старт: 1.5us; если всё ещё “дёргается” — увеличь до 2000–3000
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &flt));

    // 3) Channel 0: edge=A, level=B
    pcnt_chan_config_t ch0_cfg = {
        .edge_gpio_num  = ENCODER_A,
        .level_gpio_num = ENCODER_B,
    };
    pcnt_channel_handle_t ch0 = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &ch0_cfg, &ch0));

    // 4) Channel 1: edge=B, level=A
    pcnt_chan_config_t ch1_cfg = {
        .edge_gpio_num  = ENCODER_B,
        .level_gpio_num = ENCODER_A,
    };
    pcnt_channel_handle_t ch1 = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &ch1_cfg, &ch1));

    // 5) Действия на фронты: считаем и по rising, и по falling => 2X на канал
    // В сумме 2 канала => 4X на “щелчок”
    //
    // Логика: level_action “INVERSE” инвертирует действие при определённом уровне второй фазы.
    // Это стандартный способ собрать quadrature decoder из PCNT. :contentReference[oaicite:2]{index=2}
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch0,
                    PCNT_CHANNEL_EDGE_ACTION_INCREASE,   // rising
                    PCNT_CHANNEL_EDGE_ACTION_DECREASE)); // falling
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch0,
                    PCNT_CHANNEL_LEVEL_ACTION_KEEP,      // level low
                    PCNT_CHANNEL_LEVEL_ACTION_INVERSE)); // level high

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch1,
                    PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                    PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    // Для второго канала обычно делается противоположная полярность level-action
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch1,
                    PCNT_CHANNEL_LEVEL_ACTION_INVERSE,   // level low
                    PCNT_CHANNEL_LEVEL_ACTION_KEEP));    // level high

    // 6) Запуск
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    last_steps = 0;
}



// Функция чтения для LVGL
void encoder_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    (void)drv;

    int count = 0;
    pcnt_unit_get_count(pcnt_unit, &count);

    // В 4X режиме один “щелчок” часто даёт 4 counts => делим на 4
    // Это ровно тот подход, который Espressif/сообщество рекомендует для UI. :contentReference[oaicite:3]{index=3}
    int steps = count / 4;

    int diff = steps - last_steps;
    last_steps = steps;

    if (invert_dir) diff = -diff;

    // LVGL любит, когда enc_diff небольшой
    if (diff > 8) diff = 8;
    if (diff < -8) diff = -8;

    data->enc_diff = diff;

    // Кнопка (active-low)
    data->state = (gpio_get_level(ENCODER_KEY) == 0) ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}


// Глобальная переменная для группы управления
static lv_group_t *menu_group;


typedef struct {
    lv_coord_t x, y, w, h;
} card_geom_t;

static void card_focus_cb(lv_event_t * e)
{
    lv_obj_t *card = lv_event_get_target(e);
    card_geom_t *g = (card_geom_t *)lv_event_get_user_data(e);
    lv_event_code_t code = lv_event_get_code(e);

    if (!g) return;

    if (code == LV_EVENT_FOCUSED) {
        lv_obj_move_foreground(card);

        // "Зум" через увеличение размеров (без transform_zoom)
        const lv_coord_t dw = 14;
        const lv_coord_t dh = 14;

        lv_obj_set_size(card, g->w + dw, g->h + dh);
        lv_obj_set_pos(card, g->x - dw/2, g->y - dh/2);

        lv_obj_invalidate(card);
    }
    else if (code == LV_EVENT_DEFOCUSED) {
        // Вернуть исходную геометрию
        lv_obj_set_size(card, g->w, g->h);
        lv_obj_set_pos(card, g->x, g->y);

        lv_obj_invalidate(card);
    }
}


void create_beautiful_menu(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    if (menu_group == NULL) {
        menu_group = lv_group_create();
        lv_group_set_default(menu_group);
    }

    lv_obj_t *cont = lv_obj_create(scr);
    lv_obj_set_size(cont, 320, 170);
    lv_obj_center(cont);

    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_opa(cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_pad_all(cont, 0, 0);

    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLL_ON_FOCUS);

    const char *icons[] = { LV_SYMBOL_GPS, LV_SYMBOL_WIFI, LV_SYMBOL_SETTINGS };
    const char *names[] = { "RF", "WiFi", "Settings" };
    lv_color_t colors[] = {
        lv_palette_main(LV_PALETTE_RED),
        lv_palette_main(LV_PALETTE_BLUE),
        lv_palette_main(LV_PALETTE_AMBER),
    };

    const lv_coord_t card_w = 80;
    const lv_coord_t card_h = 100;
    const lv_coord_t gap    = 10;

    const lv_coord_t total_w = (card_w * 3) + (gap * 2);
    const lv_coord_t start_x = (320 - total_w) / 2;
    const lv_coord_t y       = (170 - card_h) / 2;

    lv_obj_t *first_card = NULL;

    for (int i = 0; i < 3; i++) {
        lv_obj_t *card = lv_obj_create(cont);
        if (!first_card) first_card = card;

        lv_coord_t x = start_x + i * (card_w + gap);

        lv_obj_set_size(card, card_w, card_h);
        lv_obj_set_pos(card, x, y);

        lv_obj_set_style_radius(card, 12, 0);
        lv_obj_set_style_bg_color(card, lv_color_hex(0x1A1A1A), 0);
        lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(card, 1, 0);
        lv_obj_set_style_border_color(card, lv_color_hex(0x333333), 0);

        // Фокус-стиль (без transform_zoom)
        lv_obj_set_style_border_width(card, 4, LV_STATE_FOCUSED);
        lv_obj_set_style_border_color(card, colors[i], LV_STATE_FOCUSED);
        lv_obj_set_style_shadow_width(card, 18, LV_STATE_FOCUSED);
        lv_obj_set_style_shadow_color(card, colors[i], LV_STATE_FOCUSED);
        lv_obj_set_style_shadow_opa(card, LV_OPA_70, LV_STATE_FOCUSED);

        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLL_ON_FOCUS);

        // Icon label (задаём стиль явно для ANY, чтобы тема не лезла)
        lv_obj_t *lbl_icon = lv_label_create(card);
        lv_label_set_text(lbl_icon, icons[i]);
        lv_obj_set_style_text_font(lbl_icon, &lv_font_montserrat_24, 0);
        lv_obj_set_style_text_color(lbl_icon, colors[i], 0);
        
        lv_obj_align(lbl_icon, LV_ALIGN_TOP_MID, 0, 14);

        // Name label
        lv_obj_t *lbl_name = lv_label_create(card);
        lv_label_set_text(lbl_name, names[i]);
        lv_obj_set_style_text_color(lbl_name, lv_color_white(), 0);
        
        lv_obj_align(lbl_name, LV_ALIGN_BOTTOM_MID, 0, -10);

        // Геометрия для “зум без transform”
        card_geom_t *geom = (card_geom_t *)lv_mem_alloc(sizeof(card_geom_t));
        geom->x = x; geom->y = y; geom->w = card_w; geom->h = card_h;

        lv_obj_add_event_cb(card, card_focus_cb, LV_EVENT_FOCUSED, geom);
        lv_obj_add_event_cb(card, card_focus_cb, LV_EVENT_DEFOCUSED, geom);

        lv_group_add_obj(menu_group, card);
    }

    if (first_card) lv_group_focus_obj(first_card);
}

void init_display(){

    // 1. Backlight Initialization
    gpio_config_t bk_gpio_config = {};
    bk_gpio_config.pin_bit_mask = 1ULL << PIN_NUM_BK_LIGHT;
    bk_gpio_config.mode = GPIO_MODE_OUTPUT;
    gpio_config(&bk_gpio_config);
    gpio_set_level((gpio_num_t)PIN_NUM_BK_LIGHT, 1);

    // 2. SPI Bus Initialization
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = PIN_NUM_MOSI;
    buscfg.miso_io_num = PIN_NUM_MISO;
    buscfg.sclk_io_num = PIN_NUM_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t);
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

}

void init_panel(){

       // 3. LCD Panel IO
    // --- FIX: Declare the handle here! ---
    esp_lcd_panel_io_handle_t io_handle = NULL;

    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.dc_gpio_num = PIN_NUM_DC;
    io_config.cs_gpio_num = PIN_NUM_CS;
    io_config.pclk_hz = 40 * 1000 * 1000;
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;
    io_config.spi_mode = 0;
    io_config.trans_queue_depth = 10;

    // Now io_handle is declared, so this call will work
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    // 4. ST7789 Panel Driver
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.reset_gpio_num = PIN_NUM_RST;
    panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    panel_config.bits_per_pixel = 16;

    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_invert_color(panel_handle, true);
    esp_lcd_panel_set_gap(panel_handle, 0, 35);
    //esp_lcd_panel_swap_xy(panel_handle, true);
    //esp_lcd_panel_mirror(panel_handle, false, true);

    esp_lcd_panel_disp_on_off(panel_handle, true);

    // 5. Initialize LVGL Porting Layer
    lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
     lvgl_port_init(&lvgl_cfg);

 
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = 320 * 40, // Размер буфера
        .double_buffer = false,
        .hres = 320, // Теперь ширина 320
        .vres = 170, // А высота 170
        .monochrome = false,
        .rotation = {
            .swap_xy = true, // ВАЖНО: ставьте false, если уже сделали swap в драйвере выше
            .mirror_x = false,
            .mirror_y = true,
        }};
    // We store the pointer but cast to void to silence the "unused variable" warning
    lv_disp_t *disp = lvgl_port_add_disp(&disp_cfg);
    (void)disp;

}
static lv_indev_t *s_encoder_indev = NULL;

static lv_indev_t *init_encoder_indev(void)
{
    if (s_encoder_indev) {
        return s_encoder_indev; // уже зарегистрирован
    }

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type    = LV_INDEV_TYPE_ENCODER;
    indev_drv.read_cb = encoder_read_cb;

    s_encoder_indev = lv_indev_drv_register(&indev_drv);
    return s_encoder_indev;
}


void app_main(void)
{
    //init hardware
    init_display();
    init_panel();
    init_hardware_encoder();
    
    if (lvgl_port_lock(0)) {
        create_beautiful_menu();

        lv_indev_t *enc = init_encoder_indev();     // один раз
        lv_indev_set_group(enc, menu_group);        // привязка к группе

        lvgl_port_unlock();
    }

    cc1101_power_on(true);

    //--------CC1101--------------
      cc1101_t cc;
    cc1101_cfg_t cccfg = {
        .host = LCD_HOST,
        .pin_cs = PIN_CC_CS,
        .clock_hz = 2 * 1000 * 1000,
    };
    ESP_ERROR_CHECK(cc1101_init_dev(&cc, &cccfg));

    // мягкий reset стробом (минимум)
    ESP_ERROR_CHECK(cc1101_strobe(&cc, CC1101_SRES));
    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t part = 0, ver = 0, marc = 0;
    ESP_ERROR_CHECK(cc1101_read_status(&cc, CC1101_PARTNUM, &part));
    ESP_ERROR_CHECK(cc1101_read_status(&cc, CC1101_VERSION, &ver));
    ESP_ERROR_CHECK(cc1101_read_status(&cc, CC1101_MARCSTATE, &marc));
    ESP_LOGI(TAG, "CC1101 PART=0x%02X VER=0x%02X MARC=0x%02X", part, ver, marc);




       esp_err_t err;
    err = cc1101_apply_preset_pairs_then_patable(&cc, subghz_device_cc1101_preset_2fsk_dev12khz_async_regs);
    vTaskDelay(pdMS_TO_TICKS(40));
    ESP_ERROR_CHECK(cc1101_set_freq_hz(&cc, 314350000UL)); // 314.35 MHz

    vTaskDelay(pdMS_TO_TICKS(40));
    ESP_ERROR_CHECK(cc1101_enter_rx(&cc));
    vTaskDelay(pdMS_TO_TICKS(40));
    //-------------CC1101 X-----------------

    // ----- Decoder Task -----
      xTaskCreatePinnedToCore(
        rmt_rx_loop_task, // Функция из decoder.c
        "rmt_decoder",    // Имя
        4096,             // Стек
        NULL,             // Передаем хендл канала
        5,                // Приоритет
        NULL,
        1 // Ядро 1
    );

    ESP_LOGI("MAIN", "Декодер запущен!");
    // ------- Decoder X -------

    ESP_LOGI(TAG, "LVGL Setup Complete");

}