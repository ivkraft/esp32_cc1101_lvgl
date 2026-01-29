#include "rtc.h"




void set_time_to_2026_01_13_17_00_local(void)
{
    // ВАЖНО: это "локальное" время. Если TZ не задан — будет интерпретация как UTC.
    // Для Нью-Йорка, например:
    // setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0/2", 1);
    // tzset();

    struct tm t = {0};
    t.tm_year = 2026 - 1900; // годы с 1900
    t.tm_mon = 0;            // Jan = 0
    t.tm_mday = 13;
    t.tm_hour = 17; // 5:00 PM = 17:00
    t.tm_min = 0;
    t.tm_sec = 0;
    t.tm_isdst = -1; // пусть libc сама определит DST по TZ

    time_t epoch = mktime(&t);
    if (epoch == (time_t)-1)
    {
        ESP_LOGE("TIME", "mktime failed");
        return;
    }

    struct timeval tv = {.tv_sec = epoch, .tv_usec = 0};
    settimeofday(&tv, NULL);

    ESP_LOGI("TIME", "Time set to 2026-01-13 17:00 (local)");
}