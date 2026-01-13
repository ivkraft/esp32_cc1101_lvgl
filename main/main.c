#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "esp_lvgl_port.h"

// esp-iot-solution input components
#include "iot_button.h"
#include "button_gpio.h"

// knob header name differs between versions; include what you have
#if __has_include("iot_knob.h")
#include "iot_knob.h"
#else
#include "knob.h"
#endif

#include "cc1101.h"
#include "cc1101_presets.h"
#include "cc1101_regs.h"
#include "decoder.h"

static const char *TAG = "main";

// ===== Display pins / config =====
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

// ===== Encoder pins =====
#define ENCODER_A GPIO_NUM_4
#define ENCODER_B GPIO_NUM_5
#define ENCODER_KEY GPIO_NUM_0
#define KEY_ESC GPIO_NUM_6

// ===== CC1101 pins =====
#define PIN_CC_CS 12 // manual CS
#define PIN_CC_GDO0 3
#define PIN_CC_GDO2 38




// ===== LVGL handles =====
static lv_disp_t *s_disp = NULL;
static lv_indev_t *s_encoder = NULL;
static lv_group_t *menu_group = NULL;
static button_handle_t s_esc_btn = NULL;

    const char *icons[] = { LV_SYMBOL_GPS, LV_SYMBOL_WIFI, LV_SYMBOL_BLUETOOTH, LV_SYMBOL_DRIVE, LV_SYMBOL_SETTINGS };
    const char *names[] = { "RF", "WiFi", "Bluetooth", "Drive", "Settings" };


// Direction invert if needed
static const int invert_dir = 0;

// ------------------------- Power -------------------------
static void cc1101_power_on(bool on)
{
    gpio_config_t pwr_cfg = {
        .pin_bit_mask = 1ULL << PIN_POWER_EN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&pwr_cfg);
    gpio_set_level(PIN_POWER_EN, on);
    vTaskDelay(pdMS_TO_TICKS(50));
}

// ------------------------- UI -------------------------
typedef struct
{
    lv_coord_t x, y, w, h;
} card_geom_t;

static bool s_in_submenu = false; // true когда вы “внутри” карточки
static lv_group_t *s_sub_group = NULL;

static lv_indev_t *s_nav_btns = NULL; // опционально, если захотите remove

static void create_beautiful_menu(void);
static void ui_back_to_menu_group(void);
static void init_esc_button(void);
static void esc_btn_cb(void *button_handle, void *usr_data);

static void esc_btn_cb(void *button_handle, void *usr_data)
{
    (void)button_handle;
    (void)usr_data;

    if (!s_in_submenu)
        return;

    if (lvgl_port_lock(0))
    {
        ui_back_to_menu_group();
        lvgl_port_unlock();
    }
}

static void init_esc_button(void)
{
    static const button_gpio_config_t esc_cfg = {
        .gpio_num = KEY_ESC,
        .active_level = 0, // active-low
    };
    static const button_config_t btn_cfg = {0};

    ESP_ERROR_CHECK(iot_button_new_gpio_device(&btn_cfg, &esc_cfg, &s_esc_btn));

    // сигнатура вашей функции: (handle, event, event_args, cb, usr_data)
    ESP_ERROR_CHECK(iot_button_register_cb(s_esc_btn, BUTTON_PRESS_UP, NULL, esc_btn_cb, NULL));
}

static void ui_back_to_menu_group(void)
{
    s_in_submenu = false;

    // убрать подменю
    lv_obj_clean(lv_scr_act());

    // восстановить меню
    create_beautiful_menu();

    // вернуть управление энкодера меню
    if (s_encoder && menu_group)
    {
        lv_indev_set_group(s_encoder, menu_group);
    }
}

static void card_key_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_KEY) return;

    uint32_t key = lv_event_get_key(e);
    if (key != LV_KEY_ENTER) return;

    uintptr_t idx = (uintptr_t)lv_event_get_user_data(e);
    ESP_LOGI("UI", "Card  new %u clicked", (unsigned)idx);

    // вход в подменю
    s_in_submenu = true;
    // ui_open_submenu(idx); // если у вас есть
}


static lv_style_t s_style_focus;
static bool s_style_focus_inited = false;

static void focus_style_init_once(void)
{
    if (s_style_focus_inited) return;
    s_style_focus_inited = true;

    lv_style_init(&s_style_focus);
    lv_style_set_outline_width(&s_style_focus, 6);      // толщина рамки
    lv_style_set_outline_pad(&s_style_focus, 2);        // “наружу”
    lv_style_set_outline_color(&s_style_focus, lv_color_make(255, 0, 0)); // Set the color (Red in this case)
    lv_style_set_outline_opa(&s_style_focus, LV_OPA_COVER);
    // цвет зададим отдельно на каждой карточке (см. ниже), либо оставим общий
}

static void card_clicked_cb(lv_event_t *e)
{
    lv_obj_t *card = lv_event_get_target(e);
    uintptr_t idx = (uintptr_t)lv_event_get_user_data(e);

    ESP_LOGI("UI", "Card %s clicked", names[idx]);
    s_in_submenu = true;

    // TODO: тут переключение экранов / открытие меню / вызов функции
    // open_rf_screen(); / open_wifi_screen(); / open_settings_screen();
}





static void create_beautiful_menu(void)
{
    lv_obj_t *scr = lv_disp_get_scr_act(s_disp);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    if (menu_group == NULL) {
        menu_group = lv_group_create();
        lv_group_set_default(menu_group);
    }

    const lv_coord_t view_w = 320;
    const lv_coord_t view_h = 170;

    const lv_coord_t card_w = 100;
    const lv_coord_t card_h = 140;
    const lv_coord_t gap    = 10;

    lv_obj_t *cont = lv_obj_create(scr);
    lv_obj_set_size(cont, view_w, view_h);
    lv_obj_center(cont);

    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_opa(cont, LV_OPA_TRANSP, 0);

    lv_obj_set_style_pad_top(cont, 0, 0);
    lv_obj_set_style_pad_bottom(cont, 0, 0);

    lv_coord_t side_pad = (view_w - card_w) / 2;
    lv_obj_set_style_pad_left(cont, side_pad, 0);
    lv_obj_set_style_pad_right(cont, side_pad, 0);

    lv_obj_add_flag(cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(cont, LV_DIR_HOR);
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_OFF);

    lv_obj_add_flag(cont, LV_OBJ_FLAG_SCROLL_ON_FOCUS);
    lv_obj_set_scroll_snap_x(cont, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_scroll_snap_y(cont, LV_SCROLL_SNAP_NONE);

    // ВАЖНО: чтобы outline не клипался — ставим на РОДИТЕЛЯ
    lv_obj_add_flag(cont, LV_OBJ_FLAG_OVERFLOW_VISIBLE);



    lv_color_t colors[] = {
        lv_palette_main(LV_PALETTE_RED),
        lv_palette_main(LV_PALETTE_BLUE),
        lv_palette_main(LV_PALETTE_AMBER),
        lv_palette_main(LV_PALETTE_GREEN),
        lv_palette_main(LV_PALETTE_PURPLE),
    };

    // Инициализируем стиль фокуса 1 раз
    focus_style_init_once();

    lv_obj_t *first_card = NULL;

    for (int i = 0; i < 5; i++) {
        lv_obj_t *card = lv_btn_create(cont);
        if (!first_card) first_card = card;

        lv_obj_set_size(card, card_w, card_h);

        lv_coord_t x = i * (card_w + gap);
        lv_coord_t y = (view_h - card_h) / 2;
        lv_obj_set_pos(card, x, y);

        // базовый стиль
        lv_obj_set_style_radius(card, 12, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(card, lv_color_hex(0x1A1A1A), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_opa(card, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);

        lv_obj_set_style_border_width(card, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_color(card, lv_color_hex(0x333333), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_opa(card, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);

        // Контент
        lv_obj_t *lbl_icon = lv_label_create(card);
        lv_label_set_text(lbl_icon, icons[i]);
        lv_obj_set_style_text_font(lbl_icon, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_color(lbl_icon, colors[i], LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align(lbl_icon, LV_ALIGN_TOP_MID, 0, 14);

        lv_obj_t *lbl_name = lv_label_create(card);
        lv_label_set_text(lbl_name, names[i]);
        lv_obj_set_style_text_color(lbl_name, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align(lbl_name, LV_ALIGN_BOTTOM_MID, 0, -10);

        // ВАЖНО: если хочешь РАЗНЫЙ цвет рамки — один общий стиль не подойдет.
        // Тогда либо делай 5 style (по одному на цвет), либо оставь цвет общий.
        // Если устраивает общий цвет рамки (например красный) — просто add_style:
        lv_obj_add_style(card, &s_style_focus, LV_STATE_FOCUS_KEY);

        lv_group_add_obj(menu_group, card);
        lv_obj_add_event_cb(card, card_clicked_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
    }

    if (first_card) {
        lv_group_focus_obj(first_card);
    }
}




// ------------------------- Display init (ваш код) -------------------------
static void init_display(void)
{
    gpio_config_t bk_gpio_config = {};
    bk_gpio_config.pin_bit_mask = 1ULL << PIN_NUM_BK_LIGHT;
    bk_gpio_config.mode = GPIO_MODE_OUTPUT;
    gpio_config(&bk_gpio_config);
    gpio_set_level((gpio_num_t)PIN_NUM_BK_LIGHT, 1);

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = PIN_NUM_MOSI;
    buscfg.miso_io_num = PIN_NUM_MISO;
    buscfg.sclk_io_num = PIN_NUM_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t);
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
}

static void init_panel(void)
{
    esp_lcd_panel_io_handle_t io_handle = NULL;

      lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));
    
    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.dc_gpio_num = PIN_NUM_DC;
    io_config.cs_gpio_num = PIN_NUM_CS;
    io_config.pclk_hz = 40 * 1000 * 1000;
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;
    io_config.spi_mode = 0;
    io_config.trans_queue_depth = 10;

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

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
    esp_lcd_panel_disp_on_off(panel_handle, true);

  

    // Оставляем вашу структуру (без полей, которых нет в вашем заголовке)
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = 320 * 170,
        .double_buffer = true,
        .hres = 320,
        .vres = 170,
        .monochrome = false,
        .rotation = {
            .swap_xy = true,
            .mirror_x = false,
            .mirror_y = true,
        },
        .flags = {
            .buff_spiram = false,
        }
    };

    s_disp = lvgl_port_add_disp(&disp_cfg);
    if (!s_disp)
    {
        ESP_LOGE(TAG, "lvgl_port_add_disp failed");
        abort();
    }
}

// ------------------------- Encoder via esp_lvgl_port 2.7.0 -------------------------
static lv_indev_t *init_encoder_via_lvgl_port(void)
{
    if (s_encoder)
        return s_encoder;

    // GPIO button (enter)
    static const button_gpio_config_t encoder_btn_gpio_cfg = {
        .gpio_num = ENCODER_KEY,
        .active_level = 0, // active-low
        .disable_pull = true
    };
    const button_config_t btn_cfg = {0};

    button_handle_t encoder_btn_handle = NULL;
    ESP_ERROR_CHECK(iot_button_new_gpio_device(&btn_cfg, &encoder_btn_gpio_cfg, &encoder_btn_handle));

    // Quadrature knob (A/B)
    const knob_config_t encoder_ab_cfg = {
        .default_direction = invert_dir ? 1 : 0,
        .gpio_encoder_a = ENCODER_A,
        .gpio_encoder_b = ENCODER_B,
    };

    const lvgl_port_encoder_cfg_t enc_cfg = {
        .disp = s_disp,
        .encoder_a_b = &encoder_ab_cfg,
        .encoder_enter = encoder_btn_handle,
    };

    s_encoder = lvgl_port_add_encoder(&enc_cfg);
    if (!s_encoder)
    {
        ESP_LOGE(TAG, "lvgl_port_add_encoder failed");
        abort();
    }

    return s_encoder;
}

// ------------------------- app_main -------------------------
void app_main(void)
{
    init_display();
    init_panel();

    init_esc_button();

    if (lvgl_port_lock(0))
    {
        // 1) Create LVGL objects
        create_beautiful_menu();

        // 2) Add encoder indev and bind to group (ONE time)
     lv_indev_t *enc = init_encoder_via_lvgl_port();

        lv_indev_set_group(enc, menu_group);

        lvgl_port_unlock();
    }

    ESP_LOGI(TAG, "LVGL Setup Complete");

    // -------- CC1101 ----------
    cc1101_power_on(true);

    cc1101_t cc;
    cc1101_cfg_t cccfg = {
        .host = LCD_HOST,
        .pin_cs = PIN_CC_CS,
        .clock_hz = 2 * 1000 * 1000,
    };
    ESP_ERROR_CHECK(cc1101_init_dev(&cc, &cccfg));

    ESP_ERROR_CHECK(cc1101_strobe(&cc, CC1101_SRES));
    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t part = 0, ver = 0, marc = 0;
    ESP_ERROR_CHECK(cc1101_read_status(&cc, CC1101_PARTNUM, &part));
    ESP_ERROR_CHECK(cc1101_read_status(&cc, CC1101_VERSION, &ver));
    ESP_ERROR_CHECK(cc1101_read_status(&cc, CC1101_MARCSTATE, &marc));
    ESP_LOGI(TAG, "CC1101 PART=0x%02X VER=0x%02X MARC=0x%02X", part, ver, marc);

    ESP_ERROR_CHECK(cc1101_apply_preset_pairs_then_patable(&cc, subghz_device_cc1101_preset_2fsk_dev12khz_async_regs));
    vTaskDelay(pdMS_TO_TICKS(40));
    ESP_ERROR_CHECK(cc1101_set_freq_hz(&cc, 314350000UL)); // 314.35 MHz
    vTaskDelay(pdMS_TO_TICKS(40));
    ESP_ERROR_CHECK(cc1101_enter_rx(&cc));
    vTaskDelay(pdMS_TO_TICKS(40));

    xTaskCreatePinnedToCore(
        rmt_rx_loop_task,
        "rmt_decoder",
        4096,
        NULL,
        5,
        NULL,
        1);

    ESP_LOGI(TAG, "Decoder task started");
}
