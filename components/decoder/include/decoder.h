#ifndef DECODER_H
#define DECODER_H
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_rx.h"
#include "freertos/queue.h"

#include "esp_log.h"


typedef struct {
    uint8_t data[128]; // Буфер для HEX данных
    int len;           // Кол-во принятых байт
    bool updated;      // Флаг для main.c
} packet_t;

// extern говорит компилятору: "сама переменная в другом файле, просто знай о ней"
extern packet_t last_pkt;

extern bool decoder_rmt_running;

void rmt_rx_loop_task(void *arg);

#endif