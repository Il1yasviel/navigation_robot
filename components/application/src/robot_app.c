#include "application/robot_app.h"

#include "esp_err.h"
#include "services/host_link_service.h"
#include "services/motor_service.h"
#include "services/safety_service.h"

void robot_app_start(void)
{
    ESP_ERROR_CHECK(motor_service_start());
    ESP_ERROR_CHECK(host_link_service_start());
    ESP_ERROR_CHECK(safety_service_start());
}
