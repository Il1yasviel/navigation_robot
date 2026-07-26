#include "esp32_drivers/system_time.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

uint32_t esp32_time_millis(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

void esp32_delay_ms(uint32_t milliseconds)
{
    vTaskDelay(pdMS_TO_TICKS(milliseconds));
}
