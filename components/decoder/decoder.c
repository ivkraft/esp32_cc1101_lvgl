
#include "freertos/FreeRTOS.h"

#include "freertos/queue.h"
#include "driver/rmt_rx.h"
#include "esp_log.h"
#include <stdlib.h>
#include "decoder.h"

#define PIN_CC_GDO0 3


static const char *TAG = "DECODER";
packet_t last_pkt = { .data = {0}, .len = 0, .updated = false };

static QueueHandle_t rmt_rx_evt_queue = NULL;

// Этот callback вызывается драйвером, когда пакет принят (по таймауту паузы)
static bool IRAM_ATTR rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    // Отправляем количество принятых символов в очередь
    xQueueSendFromISR(rmt_rx_evt_queue, &edata->num_symbols, &high_task_wakeup);
    // printf("go\n");

    return high_task_wakeup == pdTRUE;
}

void rmt_rx_loop_task(void *arg)
{
    // 1. Создаем канал прямо здесь (раз ты не делаешь этого в main)
    rmt_rx_channel_config_t rx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = PIN_CC_GDO0,
        .mem_block_symbols = 64,
        .resolution_hz = 100000, // 100 кГц (1 тик = 10 мкс)
    };

    rmt_channel_handle_t rx_chan = NULL;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_chan));

    // 2. Очередь событий
    rmt_rx_evt_queue = xQueueCreate(10, sizeof(size_t));

    // 3. Коллбэки
    rmt_rx_event_callbacks_t cbs = {.on_recv_done = rmt_rx_done_callback};
    rmt_rx_register_event_callbacks(rx_chan, &cbs, NULL);

    // 4. Включаем канал
    ESP_ERROR_CHECK(rmt_enable(rx_chan));

    // 5. ВАЖНО: static выносит массив из стека в общую память, предотвращая Crash
    static rmt_symbol_word_t raw_symbols[1000];

    // 6. Конфиг приема (настраиваем тайм-аут тишины)
    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 10000,   // 10 мкс минимум
        .signal_range_max_ns = 2000000, // 2 мс максимум
    };

    printf("RMT Декодер инициализирован внутри задачи и запущен!\n");
    while (1)
    {
        ESP_ERROR_CHECK(rmt_receive(rx_chan, raw_symbols, sizeof(raw_symbols), &receive_config));

        size_t num_symbols = 0;
        // Ждем сообщения из коллбэка о том, что прием окончен
        if (xQueueReceive(rmt_rx_evt_queue, &num_symbols, portMAX_DELAY))
        {
            if (num_symbols < 32)
                continue;
            printf("\n--- Пакет PWM ---\n");

            uint32_t last_dur = 0;
            int last_lvl = -1;
            uint8_t current_byte = 0;
            int bit_count = 0;
            int bytes_printed = 0; // Теперь переменная на месте

            for (size_t i = 0; i < num_symbols; i++)
            {
                uint32_t durs[2] = {raw_symbols[i].duration0, raw_symbols[i].duration1};
                int lvls[2] = {raw_symbols[i].level0, raw_symbols[i].level1};

                for (int j = 0; j < 2; j++)
                {
                    if (durs[j] == 0)
                        continue;

                    if (last_lvl == -1)
                    {
                        last_lvl = lvls[j];
                        last_dur = durs[j];
                    }
                    else if (lvls[j] == last_lvl)
                    {
                        last_dur += durs[j];
                    }
                    else
                    {
                        // Реальный переход уровня
                        if (durs[j] > 5)
                        { // Игнорируем совсем мелкий шум

                            // Нас интересует только длительность ПОЛОЖИТЕЛЬНОГО импульса
                            if (last_lvl == 1)
                            {
                                int bit = -1;
                                if (last_dur >= 25 && last_dur <= 45)
                                    bit = 0; // ~350 мкс
                                else if (last_dur >= 60 && last_dur <= 85)
                                    bit = 1; // ~700 мкс

                                if (bit != -1)
                                {
                                    current_byte = (current_byte << 1) | bit;
                                    bit_count++;

                                    if (bit_count == 8)
                                    {
                                        if (bytes_printed < sizeof(last_pkt.data))
                                        {
                                            last_pkt.data[bytes_printed] = current_byte;
                                        }
                                        bytes_printed++; // Считаем байты

                                        // Твой старый printf остается
                                        printf("%02X ", current_byte);

                                        bit_count = 0;
                                        current_byte = 0;
                                    }
                                }
                            }

                            last_lvl = lvls[j];
                            last_dur = durs[j];
                        }
                    }
                }
            }
            last_pkt.len = bytes_printed;
            last_pkt.updated = true;
            printf("\n--- Конец пакета ---\n");
        }
    }
}