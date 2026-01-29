#include "esp_check.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"

#include <stdio.h>
#include <sys/time.h>
#include <time.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "display/lv_display.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"

#include "esp_lvgl_port.h"

// esp-iot-solution input components
#include "button_gpio.h"
#include "iot_button.h"

// knob header name differs between versions; include what you have
#if __has_include("iot_knob.h")
#include "iot_knob.h"
#else
#include "knob.h"
#endif

#include "bq25896.h"
#include "bq27220.h"
#include "cc1101.h"
#include "cc1101_presets.h"
#include "cc1101_regs.h"
#include "decoder.h"
#include "rtc.h"

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
static lv_display_t *s_disp = NULL;
static lv_indev_t *s_encoder = NULL;

TaskHandle_t decoder_task_handle = NULL;

static button_handle_t s_esc_btn = NULL;
// Menu data
const char *icons[] = {LV_SYMBOL_GPS, LV_SYMBOL_WIFI, LV_SYMBOL_BLUETOOTH,
                       LV_SYMBOL_DRIVE, LV_SYMBOL_SETTINGS};
const char *names[] = {"RF", "WiFi", "Bluetooth", "Drive", "Settings"};

// Direction invert if needed
static const int invert_dir = 0;

//--battary

static int batt_proc = 50; // 0..100 (you will update this from your code)

static void ui_set_battery_percent(int pct, bool isCharge);

static void fuel_gauge_task(void *arg) {
  (void)arg;
  static bq27220_t bq_cfg;
  static bq25896_t s_charger;
  // пробуем стартовать I2C + девайс
  if (i2c_bq27220_init(&bq_cfg) != ESP_OK) {
    ESP_LOGE("BQ27220", "I2C init failed");
    vTaskDelete(NULL);
    return;
  } else {
    ESP_LOGI("BQ27220", "Sending Soft Reset to recalibrate SOC...");
    bq_write_subcmd(0x0042, &bq_cfg);
    vTaskDelay(pdMS_TO_TICKS(500)); // Даем время чипу очнуться
  }

  if (bq25896_init(&s_charger, bq_cfg.s_i2c_bus) != ESP_OK) {
    ESP_LOGE("BQ25896", "I2C init failed");
    vTaskDelete(NULL);
    return;
  } else {
    ESP_LOGI(TAG, "Configuring BQ25896...");

    // Используем простую проверку, чтобы не "падать"
    if (bq25896_write_reg(&s_charger, 0x03, 0x1A) != ESP_OK)
      ESP_LOGE(TAG, "WD fix failed");
    if (bq25896_write_reg(&s_charger, 0x00, 0x28) != ESP_OK)
      ESP_LOGE(TAG, "IINLIM fix failed");
    if (bq25896_write_reg(&s_charger, 0x04, 0x10) != ESP_OK)
      ESP_LOGE(TAG, "ICHG fix failed");
    if (bq25896_write_reg(&s_charger, 0x06, 0x5E) != ESP_OK)
      ESP_LOGE(TAG, "VREG fix failed");
  }
  // ESP_ERROR_CHECK(bq25896_init(&s_charger, bq_cfg.s_i2c_bus,
  // BQ25896_SLAVE_ADDRESS, 400000));

  bool orange;
  bq25896_status_t st;

  while (1) {
    uint16_t soc = 0, mv = 0;

    esp_err_t e1 = bq_read_u16(BQ27220_REG_SOC, &soc, &bq_cfg);
    esp_err_t e2 = bq_read_u16(BQ27220_REG_VOLTAGE, &mv, &bq_cfg);

    if (e1 == ESP_OK) {
      int pct = (int)soc;
      if (pct < 0)
        pct = 0;
      if (pct > 100)
        pct = 100;
      batt_proc = pct;
    } else {
      ESP_LOGW("BQ27220", "SOC read err: %s", esp_err_to_name(e1));
    }

    if (e2 == ESP_OK) {
      ESP_LOGI("BQ27220", "SOC=%u%%  V=%umV", (unsigned)soc, (unsigned)mv);
    } else {
      ESP_LOGW("BQ27220", "V read err: %s", esp_err_to_name(e2));
    }

    uint8_t fault_reg = 0;
    bq25896_read_reg(&s_charger, 0x0C, &fault_reg);
    ESP_LOGI("BQ25896", "Fault Register: 0x%02X", fault_reg);

    // --- СЕКЦИЯ ПОДДЕРЖАНИЯ ЗАРЯДКИ ---
    // Каждые 2 секунды принудительно гасим Watchdog и ставим лимиты
    // Это "костыль", который лечит любые сбросы чипа
    bq25896_write_reg(&s_charger, 0x03, 0x1A); // WD off + Charge Enable
    bq25896_write_reg(&s_charger, 0x00, 0x28); // IINLIM 2A

    // Запускаем конверсию АЦП, чтобы видеть актуальный ток
    // REG02: бит 7 (CONV_START)
    uint8_t reg02 = 0;
    bq25896_read_reg(&s_charger, 0x02, &reg02);
    bq25896_write_reg(&s_charger, 0x02, reg02 | 0x80);

    // --- СЕКЦИЯ ЧТЕНИЯ ДАННЫХ ---
    uint16_t soc1 = 0;
    uint16_t mv1 = 0;
    bq25896_status_t st;
    uint8_t ichg_raw = 0;

    bq_read_u16(BQ27220_REG_SOC, &soc1, &bq_cfg);
    bq_read_u16(BQ27220_REG_VOLTAGE, &mv1, &bq_cfg);
    bq25896_get_status(&s_charger, &st);
    bq25896_read_reg(&s_charger, 0x12, &ichg_raw);

    uint8_t fault = 0;
    bq25896_read_reg(&s_charger, 0x0C, &fault);

    int current_ma = (ichg_raw & 0x7F) * 50;

    ESP_LOGI(TAG, "SOC=%d%% V=%dmV Fault=0x%02X Current=%dmA", soc1, mv1, fault,
             current_ma);

    orange = bq25896_is_charging_active(&st);

    // обновляем UI безопасно через lock
    if (lvgl_port_lock(0)) {
      if (bq25896_get_status(&s_charger, &st) == ESP_OK) {
        ui_set_battery_percent(batt_proc, orange);
      }

      lvgl_port_unlock();
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// ---battary X ------

// ------------------------- Power -------------------------

// ------------------------- UI -------------------------
typedef struct {
  lv_coord_t x, y, w, h;
} card_geom_t;

static bool s_in_submenu = false; // true когда вы “внутри” карточки
static lv_group_t *s_sub_group = NULL;

static lv_indev_t *s_nav_btns = NULL; // опционально, если захотите remove

lv_indev_t *enc;

static void create_beautiful_menu(void);
static void ui_back_to_menu_group(void);
static void init_esc_button(void);
static void esc_btn_cb(void *button_handle, void *usr_data);

static void esc_btn_cb(void *button_handle, void *usr_data) {
  (void)button_handle;
  (void)usr_data;

  if (!s_in_submenu)
    return;

  if (lvgl_port_lock(0)) {
    ui_back_to_menu_group();
    lvgl_port_unlock();
  }
}

// DISPLAY LED

static bool s_backlight_on = true;
static bool s_ignore_next_up = false;

static inline void backlight_set(bool on) {
  s_backlight_on = on;
  gpio_set_level(PIN_NUM_BK_LIGHT, on ? 1 : 0);
}

static void esc_short_up_cb(void *btn_handle, void *usr_data) {
  (void)btn_handle;
  (void)usr_data;

  if (s_ignore_next_up) { // это отпускание после long-press
    s_ignore_next_up = false;
    return;
  }

  if (lvgl_port_lock(0)) {
    ui_back_to_menu_group();
    lvgl_port_unlock();
  }
}

static void esc_long_cb(void *btn_handle, void *usr_data) {
  (void)btn_handle;
  (void)usr_data;

  s_ignore_next_up = true; // чтобы отпускание не сделало "назад"
  backlight_set(!s_backlight_on); // toggle подсветки
  // esp_restart();
}
//---- DISPLAY LED X ----

static void init_esc_button(void) {
  static const button_gpio_config_t esc_cfg = {
      .gpio_num = KEY_ESC,
      .active_level = 0, // active-low
  };
  static const button_config_t btn_cfg = {0};

  ESP_ERROR_CHECK(iot_button_new_gpio_device(&btn_cfg, &esc_cfg, &s_esc_btn));
  uint32_t lp_ms = 3000;
  // сигнатура вашей функции: (handle, event, event_args, cb, usr_data)
  ESP_ERROR_CHECK(iot_button_register_cb(s_esc_btn, BUTTON_LONG_PRESS_TIME_MS,
                                         NULL, esc_btn_cb, &lp_ms));

  ESP_ERROR_CHECK(iot_button_register_cb(s_esc_btn, BUTTON_LONG_PRESS_START,
                                         NULL, esc_long_cb, NULL));
  ESP_ERROR_CHECK(iot_button_register_cb(s_esc_btn, BUTTON_PRESS_UP, NULL,
                                         esc_short_up_cb, NULL));
}

static void ui_back_to_menu_group(void) {
  s_in_submenu = false;

  // убрать подменю
  lv_obj_clean(lv_scr_act());

  // восстановить меню
  create_beautiful_menu();

  // вернуть управление энкодера меню
}

static lv_style_t s_style_focus;
static bool s_style_focus_inited = false;

static lv_obj_t *s_rf_label = NULL;
static lv_timer_t *s_rf_timer = NULL;

static void rf_screen_close(void) {
  if (s_rf_timer) {
    lv_timer_del(s_rf_timer);
    s_rf_timer = NULL;
  }
  s_rf_label = NULL;
}

static void rf_timer_cb(lv_timer_t *t) {
  (void)t;
  if (!s_rf_label)
    return;

  if (!last_pkt.updated)
    return;

  // ---------------------------------------------------------------------------

  char hex[3 * 128 + 1]; // "AA " * 128 + '\0'
  size_t p = 0;

  int n = last_pkt.len;
  if (n < 0)
    n = 0;
  if (n > 128)
    n = 128;

  for (int i = 0; i < n && (p + 3) < sizeof(hex); i++) {
    p += (size_t)snprintf(hex + p, sizeof(hex) - p, "%02X ", last_pkt.data[i]);
  }
  hex[p] = 0;

  char text[700];
  snprintf(text, sizeof(text), "Last packet: %d bytes\n%s", n, hex);

  lv_label_set_text(s_rf_label, text);
}

static void back_to_menu_cb(lv_event_t *e) {
  rf_screen_close();
  lv_obj_t *scr = lv_display_get_screen_active(s_disp);
  lv_obj_clean(scr);
  s_in_submenu = false;
  if (lvgl_port_lock(0)) {
    if (decoder_task_handle != NULL) {
        printf("DEcoder is %d\n",decoder_rmt_running);

      if (decoder_rmt_running) {
        decoder_rmt_running = false;
        printf("Decoder suspend\n");
      }
    }

    create_beautiful_menu();
    lvgl_port_unlock();
  }
}

void open_rf_screen(void) {

  if (lvgl_port_lock(0)) {

    if (decoder_task_handle != NULL) {


      if (!decoder_rmt_running ) {
        decoder_rmt_running = true;
                printf("Decoder resume\n");

      }
      } else {
        xTaskCreatePinnedToCore(rmt_rx_loop_task, "rmt_decoder", 4096, NULL, 5,
                                &decoder_task_handle, 1);
                                decoder_rmt_running = true;
                                printf("Decoder run\n");
      }
    

    lv_obj_t *scr = lv_display_get_screen_active(s_disp);
    lv_obj_clean(scr);

    static lv_group_t *rf_group = NULL;

    if (rf_group == NULL) {
      rf_group = lv_group_create();
      lv_group_set_default(rf_group);
    }

    // Заголовок
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "RF");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);
    // Основной лейбл
    s_rf_label = lv_label_create(scr);
    lv_label_set_text(s_rf_label, "Waiting for packet...");
    lv_obj_set_width(s_rf_label, 300);
    lv_obj_set_style_text_font(s_rf_label, &lv_font_unscii_8, LV_PART_MAIN);

    lv_label_set_long_mode(s_rf_label, LV_LABEL_LONG_WRAP);
    lv_obj_align(s_rf_label, LV_ALIGN_TOP_LEFT, 20, 40);

    lv_obj_t *btn = lv_btn_create(scr);
    lv_obj_set_size(btn, 80, 20);
    lv_obj_align(btn, LV_ALIGN_TOP_LEFT, 3, 3);
    lv_obj_add_event_cb(btn, back_to_menu_cb, LV_EVENT_CLICKED, NULL);
    lv_group_add_obj(rf_group, btn);

    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, LV_SYMBOL_LEFT " Back");
    lv_obj_center(lbl);

    lv_indev_set_group(enc, rf_group);
    lv_group_set_default(rf_group);

    if (s_rf_timer) {
      lv_timer_del(s_rf_timer);
      s_rf_timer = NULL;
    }
    s_rf_timer = lv_timer_create(rf_timer_cb, 200, NULL); // 20Hz
    s_in_submenu = true;
    lvgl_port_unlock();
  }
}

static void card_clicked_cb(lv_event_t *e) {
  lv_obj_t *card = lv_event_get_target(e);
  uintptr_t idx = (uintptr_t)lv_event_get_user_data(e);

  ESP_LOGI("UI", "Card %s clicked", names[idx]);
  switch (idx) {
  case 0:
    open_rf_screen();
    break;
  case 1:
    //   open_wifi_screen();
    break;
  case 2:
    //   open_bluetooth_screen();
    break;
  case 3:
    //   open_drive_screen();
    break;
  case 4:
    //   open_settings_screen();
    break;
  default:
    ESP_LOGW("UI", "Unknown card index %u", (unsigned)idx);
    break;
  }
  // TODO: тут переключение экранов / открытие меню / вызов функции
  // open_rf_screen(); / open_wifi_screen(); / open_settings_screen();
}

// Add these globals somewhere near your other UI globals
static lv_obj_t *s_batt_label = NULL;

//-- time

static lv_obj_t *s_time_label = NULL;
static lv_timer_t *s_time_timer = NULL;

// Simple battery glyph: 0..10 bars inside [..........]
// You can replace this with LV_SYMBOL_BATTERY_FULL etc. later if you want.
static const char *battery_symbol_from_percent(int pct) {
  if (pct < 0)
    pct = 0;
  if (pct > 100)
    pct = 100;

  // 5 levels: empty, 1,2,3, full
  if (pct <= 5)
    return LV_SYMBOL_BATTERY_EMPTY;
  if (pct <= 25)
    return LV_SYMBOL_BATTERY_1;
  if (pct <= 50)
    return LV_SYMBOL_BATTERY_2;
  if (pct <= 75)
    return LV_SYMBOL_BATTERY_3;
  return LV_SYMBOL_BATTERY_FULL;
}

static void ui_set_battery_percent(int pct, bool isCharge) {
  if (!s_batt_label)
    return;
  lv_label_set_text(s_batt_label, battery_symbol_from_percent(pct));
  lv_obj_set_style_text_color(
      s_batt_label, isCharge ? lv_color_hex(0xA5FF00) : lv_color_white(), 0);
}

lv_obj_t *cont;
// ------------------------------------------------MENU
// CREATION---------------------------------------------------------------------------------
static void create_beautiful_menu(void) {
  lv_obj_t *scr = lv_display_get_screen_active(s_disp);
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
  static lv_group_t *menu_group = NULL;

  if (menu_group == NULL) {
    menu_group = lv_group_create();
    lv_group_set_default(menu_group);
  }

  const lv_coord_t view_w = 320;
  const lv_coord_t view_h = 170;

  const lv_coord_t card_w = 240;
  const lv_coord_t card_h = 120;
  const lv_coord_t gap = 10;

  cont = lv_obj_create(scr);
  lv_obj_set_size(cont, view_w, view_h);
  lv_obj_center(cont);

  lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_opa(cont, LV_OPA_TRANSP, 0);

  lv_obj_set_style_pad_top(cont, 10, 0);
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

  // to avoid clipping outline
  //  lv_obj_add_flag(cont, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

  lv_color_t colors[] = {
      lv_palette_main(LV_PALETTE_RED),    lv_palette_main(LV_PALETTE_BLUE),
      lv_palette_main(LV_PALETTE_AMBER),  lv_palette_main(LV_PALETTE_GREEN),
      lv_palette_main(LV_PALETTE_PURPLE),
  };

  // focus_style_init_once();

  // ---------------- Battery (top-right) ----------------
  // Put it on SCR so it doesn't move when cont scrolls
  s_batt_label = lv_label_create(scr);
  lv_obj_set_style_text_color(s_batt_label, lv_color_white(), 0);
  lv_obj_align(s_batt_label, LV_ALIGN_TOP_RIGHT, 0, 4);
  ui_set_battery_percent(batt_proc, false); // initial draw
  lv_obj_set_style_text_font(s_batt_label, &lv_font_montserrat_18, 0);

  // ---------------- Time (top-center) ----------------
  // ui_time_init(scr);
  // -----------------------------------------------------

  lv_obj_t *first_card = NULL;

  static lv_style_t style_card_common;
  static bool style_init = false;
  // create styles for card
  if (!style_init) {
    lv_style_init(&style_card_common);
    // lv_style_set_radius(&style_card_common, 12);
    lv_style_set_bg_color(&style_card_common, lv_color_hex(0x1A1A1A));
    // lv_style_set_bg_opa(&style_card_common, LV_OPA_COVER);
    //   lv_style_set_border_width(&style_card_common, 1);
    //   lv_style_set_border_color(&style_card_common, lv_color_hex(0x333333));
    style_init = true;
  }

  for (int i = 0; i < 5; i++) {
    lv_obj_t *card = lv_btn_create(cont);
    if (!first_card)
      first_card = card;

    lv_obj_set_size(card, card_w, card_h);

    lv_coord_t x = i * (card_w + gap);
    lv_coord_t y = (view_h - card_h) / 2;
    lv_obj_set_pos(card, x, y);

    // base style
    lv_obj_add_style(card, &style_card_common, 0);

    // content
    lv_obj_t *lbl_icon = lv_label_create(card);
    lv_label_set_text(lbl_icon, icons[i]);
    lv_obj_set_style_text_font(lbl_icon, &lv_font_montserrat_48,
                               LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(lbl_icon, colors[i],
                                LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(lbl_icon, LV_ALIGN_TOP_MID, 0, 14);

    lv_obj_t *lbl_name = lv_label_create(card);
    lv_label_set_text(lbl_name, names[i]);
    lv_obj_set_style_text_color(lbl_name, lv_color_white(),
                                LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(lbl_name, LV_ALIGN_BOTTOM_MID, 0, -10);

    // focus outline (encoder/keypad): LV_STATE_FOCUS_KEY
    lv_obj_add_style(card, &s_style_focus, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_anim_time(card, 0, 0);

    lv_group_add_obj(menu_group, card);
    lv_obj_add_event_cb(card, card_clicked_cb, LV_EVENT_CLICKED,
                        (void *)(uintptr_t)i);
  }

  if (first_card) {
    lv_group_focus_obj(first_card);
  }
  lv_indev_set_group(enc, menu_group);
  lv_group_set_default(menu_group);
}

// ------------------------- MENU CREATION
// X---------------------------------------------------------------------------------

// static void create_beautiful_menu(void)
// {
//     lv_obj_t *scr = lv_display_get_screen_active(s_disp);
//     lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);

//     lv_color_t colors[] = {
//         lv_palette_main(LV_PALETTE_RED),
//         lv_palette_main(LV_PALETTE_BLUE),
//         lv_palette_main(LV_PALETTE_AMBER),
//         lv_palette_main(LV_PALETTE_GREEN),
//         lv_palette_main(LV_PALETTE_PURPLE),
//     };

//     if (menu_group == NULL)
//     {
//         menu_group = lv_group_create();
//         lv_group_set_default(menu_group);
//     }

//     // Удаляем старый статус-бар если был, и создаем заново
//     s_batt_label = lv_label_create(scr);
//     lv_obj_set_style_text_color(s_batt_label, lv_color_white(), 0);
//     lv_obj_align(s_batt_label, LV_ALIGN_TOP_RIGHT, -10, 5);
//     ui_set_battery_percent(batt_proc, false);

//     // Геометрия плиток
//     const int tile_w = 90;
//     const int tile_h = 65;
//     const int start_x = 15;
//     const int start_y = 25;
//     const int gap_x = 10;
//     const int gap_y = 10;

//     static lv_style_t style_tile;
//     static bool style_inited = false;
//     if (!style_inited)
//     {
//         lv_style_init(&style_tile);
//         lv_style_set_bg_color(&style_tile, lv_color_hex(0x1A1A1A));
//         lv_style_set_bg_opa(&style_tile, LV_OPA_COVER);
//         lv_style_set_border_width(&style_tile, 1);
//         lv_style_set_border_color(&style_tile, lv_color_hex(0x333333));
//         lv_style_set_radius(&style_tile, 8); // Можно оставить небольшим

//         // Стиль фокуса
//         lv_style_init(&s_style_focus);
//         lv_style_set_border_color(&s_style_focus,
//         lv_palette_main(LV_PALETTE_BLUE));
//         lv_style_set_border_width(&s_style_focus, 3);
//         style_inited = true;
//     }

//     for (int i = 0; i < 5; i++)
//     {
//         lv_obj_t *tile = lv_btn_create(scr); // Прямо на экран!

//         // Рассчитываем позицию (3 в ряд)
//         int row = i / 3;
//         int col = i % 3;
//         lv_obj_set_size(tile, tile_w, tile_h);
//         lv_obj_set_pos(tile, start_x + col * (tile_w + gap_x), start_y + row
//         * (tile_h + gap_y));

//         lv_obj_add_style(tile, &style_tile, 0);
//         lv_obj_add_style(tile, &s_style_focus, LV_STATE_FOCUSED);

//         // Контент плитки
//         lv_obj_t *lbl_icon = lv_label_create(tile);
//         lv_label_set_text(lbl_icon, icons[i]);
//         lv_obj_set_style_text_font(lbl_icon, &lv_font_montserrat_18, 0);
//         lv_obj_set_style_text_color(lbl_icon, colors[i], 0);
//         lv_obj_align(lbl_icon, LV_ALIGN_TOP_MID, 0, 5);

//         lv_obj_t *lbl_name = lv_label_create(tile);
//         lv_label_set_text(lbl_name, names[i]);
//         lv_obj_set_style_text_font(lbl_name, &lv_font_montserrat_12, 0);
//         lv_obj_align(lbl_name, LV_ALIGN_BOTTOM_MID, 0, -5);

//         lv_group_add_obj(menu_group, tile);
//         lv_obj_add_event_cb(tile, card_clicked_cb, LV_EVENT_CLICKED, (void
//         *)(uintptr_t)i);
//     }
// }

// ------------------------- Display init (ваш код) -------------------------
static void init_display(void) {
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
  buscfg.max_transfer_sz = 64 * 1024;
  ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
}

static void init_panel(void) {
  esp_lcd_panel_io_handle_t io_handle = NULL;

  vTaskDelay(pdMS_TO_TICKS(5));
  esp_lcd_panel_io_spi_config_t io_config = {};
  io_config.dc_gpio_num = PIN_NUM_DC;
  io_config.cs_gpio_num = PIN_NUM_CS;
  io_config.pclk_hz = 40 * 1000 * 1000;
  io_config.lcd_cmd_bits = 8;
  io_config.lcd_param_bits = 8;
  io_config.spi_mode = 0;
  io_config.flags.lsb_first = 0; // Обычный порядок
  io_config.trans_queue_depth = 10;

  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
                                           &io_config, &io_handle));

  esp_lcd_panel_handle_t panel_handle = NULL;
  esp_lcd_panel_dev_config_t panel_config = {};
  panel_config.reset_gpio_num = PIN_NUM_RST;
  panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
  panel_config.bits_per_pixel = 16;
  ESP_ERROR_CHECK(
      esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
  esp_lcd_panel_reset(panel_handle);
  esp_lcd_panel_init(panel_handle);
  esp_lcd_panel_invert_color(panel_handle, true);
  esp_lcd_panel_set_gap(panel_handle, 0, 35);

  // bug fix https://github.com/espressif/esp-idf/issues/11416
  esp_lcd_panel_io_tx_param(io_handle, 0xb0, (uint8_t[]){0x00, 0xE8}, 2);

  esp_lcd_panel_disp_on_off(panel_handle, true);

  lvgl_port_cfg_t lvgl_cfg = {
      .task_priority = 4,
      .task_stack = 12288, /* LVGL task stack size */
      .task_affinity = -1, /* LVGL task pinned to core (-1 is no affinity) */
      .task_max_sleep_ms = 500, /* Maximum sleep in LVGL task */
      .timer_period_ms = 5      /* LVGL timer tick period in ms */
  };
  ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));
  // Оставляем вашу структуру (без полей, которых нет в вашем заголовке)

  const lvgl_port_display_cfg_t disp_cfg = {.io_handle = io_handle,
                                            .panel_handle = panel_handle,
                                            .buffer_size = 320 * 50,
                                            .double_buffer = true,
                                            .hres = 320,
                                            .vres = 170,

                                            .monochrome = false,
                                            .rotation =
                                                {
                                                    .swap_xy = true,
                                                    .mirror_x = false,
                                                    .mirror_y = true,
                                                },
                                            .flags =
                                                {
                                                    .buff_dma = true,
                                                }

  };

  s_disp = lvgl_port_add_disp(&disp_cfg);
  if (!s_disp) {
    ESP_LOGE(TAG, "lvgl_port_add_disp failed");
    abort();
  }
}

// ------------------------- Encoder via esp_lvgl_port 2.7.0
// -------------------------
static lv_indev_t *init_encoder_via_lvgl_port(void) {
  if (s_encoder)
    return s_encoder;

  // GPIO button (enter)
  static const button_gpio_config_t encoder_btn_gpio_cfg = {
      .gpio_num = ENCODER_KEY,
      .active_level = 0, // active-low
      .disable_pull = true};
  const button_config_t btn_cfg = {0};

  button_handle_t encoder_btn_handle = NULL;
  ESP_ERROR_CHECK(iot_button_new_gpio_device(&btn_cfg, &encoder_btn_gpio_cfg,
                                             &encoder_btn_handle));

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
  if (!s_encoder) {
    ESP_LOGE(TAG, "lvgl_port_add_encoder failed");
    abort();
  }

  return s_encoder;
}

// ------------------------- app_main -------------------------
void app_main(void) {

  set_time_to_2026_01_13_17_00_local();
  init_display();
  init_panel();

  init_esc_button();
  // lv_timer_set_period(s_disp->refr_timer, 10);

  if (lvgl_port_lock(0)) {
    // 1) Create LVGL objects
    enc = init_encoder_via_lvgl_port();
    create_beautiful_menu();

    // 2) Add encoder indev and bind to group (ONE time)

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

  ESP_ERROR_CHECK(cc1101_apply_preset_pairs_then_patable(
      &cc, subghz_device_cc1101_preset_2fsk_dev12khz_async_regs));
  vTaskDelay(pdMS_TO_TICKS(40));
  ESP_ERROR_CHECK(cc1101_set_freq_hz(&cc, 314350000UL)); // 314.35 MHz
  vTaskDelay(pdMS_TO_TICKS(40));
  ESP_ERROR_CHECK(cc1101_enter_rx(&cc));
  vTaskDelay(pdMS_TO_TICKS(40));

  // xTaskCreatePinnedToCore(
  //     fuel_gauge_task,
  //     "fuel_gauge",
  //     4096,
  //     NULL,
  //     5,
  //     NULL,
  //     0);

  // ESP_LOGI(TAG, "Decoder task started");
}
