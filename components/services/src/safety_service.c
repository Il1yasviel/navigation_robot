#include "services/safety_service.h"

#include "config/robot_config.h"
#include "esp32_drivers/system_time.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "protocols/m0601c111_motor.h"
#include "services/motor_service.h"

static void safety_task(void *argument)
{
    (void)argument;
    for (;;) {
        motor_chassis_snapshot_t snapshot;
        motor_service_get_chassis_snapshot(&snapshot);
        const uint32_t age = esp32_time_millis() - snapshot.last_control_ms;
        const bool watchdog_expired = age >= CONFIG_ROBOT_CONTROL_WATCHDOG_MS;
        const bool wheel_fault =
            snapshot.wheel[MOTOR_LEFT_INDEX].fault != 0u ||
            snapshot.wheel[MOTOR_RIGHT_INDEX].fault != 0u;
        if (snapshot.control_active && (watchdog_expired || wheel_fault)) {
            motor_request_t stop = {.brake = M0601_BRAKE_ON};
            if (snapshot.dual_control_active) {
                stop.action = MOTOR_ACTION_STOP_DUAL;
                stop.id = snapshot.wheel[MOTOR_LEFT_INDEX].motor_id;
                stop.right_id = snapshot.wheel[MOTOR_RIGHT_INDEX].motor_id;
            } else {
                stop.action = MOTOR_ACTION_STOP;
                stop.id = snapshot.selected.motor_id;
            }
            (void)motor_service_execute(&stop);
            if (watchdog_expired) {
                motor_service_note_watchdog_stop();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

esp_err_t safety_service_start(void)
{
    return xTaskCreate(safety_task, "motor_safety", 3072, NULL, 13, NULL) == pdPASS
               ? ESP_OK
               : ESP_ERR_NO_MEM;
}
