#include "esp32_drivers/wifi_tcp.h"

#include <errno.h>
#include <string.h>

#include "config/robot_config.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "mdns.h"
#include "nvs_flash.h"

#define WIFI_GOT_IP_BIT BIT0

static EventGroupHandle_t s_events;
static SemaphoreHandle_t s_socket_mutex;
static int s_listen_socket = -1;
static int s_client_socket = -1;
static esp32_wifi_tcp_receive_fn s_receive;
static esp32_wifi_tcp_disconnect_fn s_disconnect;
static void *s_callback_context;
static bool s_mdns_started;

static void notify_disconnect(void)
{
    if (s_disconnect != NULL) s_disconnect(s_callback_context);
}

static bool close_client_locked(void)
{
    if (s_client_socket < 0) return false;
    shutdown(s_client_socket, SHUT_RDWR);
    close(s_client_socket);
    s_client_socket = -1;
    return true;
}

static void close_network_sockets(void)
{
    bool client_closed;
    xSemaphoreTake(s_socket_mutex, portMAX_DELAY);
    client_closed = close_client_locked();
    if (s_listen_socket >= 0) {
        shutdown(s_listen_socket, SHUT_RDWR);
        close(s_listen_socket);
        s_listen_socket = -1;
    }
    xSemaphoreGive(s_socket_mutex);
    if (client_closed) notify_disconnect();
}

static void wifi_event_handler(void *argument, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    (void)argument;
    (void)event_data;
    if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_events, WIFI_GOT_IP_BIT);
    } else if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_events, WIFI_GOT_IP_BIT);
        close_network_sockets();
    }
}

static void start_mdns_once(void)
{
    if (s_mdns_started || mdns_init() != ESP_OK) return;
    (void)mdns_hostname_set(CONFIG_ROBOT_WIFI_HOSTNAME);
    (void)mdns_instance_name_set("Navigation Robot Controller");
    (void)mdns_service_add(NULL, "_navigation-robot", "_tcp",
                           CONFIG_ROBOT_WIFI_TCP_PORT, NULL, 0);
    s_mdns_started = true;
}

static int create_listen_socket(void)
{
    const int socket_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (socket_fd < 0) return -1;
    const int reuse = 1;
    (void)setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    const struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_ROBOT_WIFI_TCP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(socket_fd, (const struct sockaddr *)&address, sizeof(address)) != 0 ||
        listen(socket_fd, 1) != 0) {
        close(socket_fd);
        return -1;
    }
    return socket_fd;
}

static void serve_client(int client)
{
    const struct timeval timeout = {.tv_sec = 0, .tv_usec = 200000};
    (void)setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    uint8_t received[256];
    while ((xEventGroupGetBits(s_events) & WIFI_GOT_IP_BIT) != 0u) {
        const int length = recv(client, received, sizeof(received), 0);
        if (length > 0) {
            if (s_receive != NULL) {
                s_receive(s_callback_context, received, (size_t)length);
            }
        } else if (length == 0) {
            break;
        } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
            break;
        }
    }
}

static void wifi_tcp_task(void *argument)
{
    (void)argument;
    static const uint16_t retry_delays_ms[] = {1000, 2000, 5000, 10000};
    size_t retry_index = 0;
    for (;;) {
        if ((xEventGroupGetBits(s_events) & WIFI_GOT_IP_BIT) == 0u) {
            (void)esp_wifi_connect();
            vTaskDelay(pdMS_TO_TICKS(retry_delays_ms[retry_index]));
            if ((xEventGroupGetBits(s_events) & WIFI_GOT_IP_BIT) == 0u) {
                if (retry_index + 1u < sizeof(retry_delays_ms) /
                                              sizeof(retry_delays_ms[0])) {
                    ++retry_index;
                }
                continue;
            }
        }
        retry_index = 0;
        start_mdns_once();
        const int listen_socket = create_listen_socket();
        if (listen_socket < 0) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        xSemaphoreTake(s_socket_mutex, portMAX_DELAY);
        s_listen_socket = listen_socket;
        xSemaphoreGive(s_socket_mutex);
        const int client = accept(listen_socket, NULL, NULL);
        if (client >= 0) {
            xSemaphoreTake(s_socket_mutex, portMAX_DELAY);
            s_client_socket = client;
            xSemaphoreGive(s_socket_mutex);
            serve_client(client);
            xSemaphoreTake(s_socket_mutex, portMAX_DELAY);
            const bool closed = close_client_locked();
            xSemaphoreGive(s_socket_mutex);
            if (closed) notify_disconnect();
        }
        xSemaphoreTake(s_socket_mutex, portMAX_DELAY);
        if (s_listen_socket == listen_socket) {
            close(s_listen_socket);
            s_listen_socket = -1;
        }
        xSemaphoreGive(s_socket_mutex);
    }
}

esp_err_t esp32_wifi_tcp_start(esp32_wifi_tcp_receive_fn receive,
                               esp32_wifi_tcp_disconnect_fn disconnect,
                               void *context)
{
#if !CONFIG_ROBOT_WIFI_ENABLED
    (void)receive;
    (void)disconnect;
    (void)context;
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (CONFIG_ROBOT_WIFI_SSID[0] == '\0') return ESP_ERR_INVALID_STATE;
    s_receive = receive;
    s_disconnect = disconnect;
    s_callback_context = context;
    s_events = xEventGroupCreate();
    s_socket_mutex = xSemaphoreCreateMutex();
    if (s_events == NULL || s_socket_mutex == NULL) return ESP_ERR_NO_MEM;
    esp_err_t status = nvs_flash_init();
    if (status == ESP_ERR_NVS_NO_FREE_PAGES ||
        status == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        status = nvs_flash_init();
    }
    if (status != ESP_OK) return status;
    status = esp_netif_init();
    if (status != ESP_OK && status != ESP_ERR_INVALID_STATE) return status;
    status = esp_event_loop_create_default();
    if (status != ESP_OK && status != ESP_ERR_INVALID_STATE) return status;
    if (esp_netif_create_default_wifi_sta() == NULL) return ESP_FAIL;
    /* 静态 IP 配置：非空时关闭 DHCP 并手动设置 */
    if (CONFIG_ROBOT_WIFI_STATIC_IP[0] != '\0') {
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif == NULL) return ESP_FAIL;
        (void)esp_netif_dhcpc_stop(netif);
        esp_netif_ip_info_t ip_info = {0};
        ip_info.ip.addr = esp_ip4addr_aton(CONFIG_ROBOT_WIFI_STATIC_IP);
        ip_info.gw.addr = esp_ip4addr_aton(CONFIG_ROBOT_WIFI_STATIC_GW);
        ip_info.netmask.addr = esp_ip4addr_aton(CONFIG_ROBOT_WIFI_STATIC_NETMASK);
        status = esp_netif_set_ip_info(netif, &ip_info);
        if (status != ESP_OK) return status;
    }
    const wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    if ((status = esp_wifi_init(&init)) != ESP_OK) return status;
    (void)esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                     wifi_event_handler, NULL);
    (void)esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                     wifi_event_handler, NULL);
    wifi_config_t configuration = {0};
    strncpy((char *)configuration.sta.ssid, CONFIG_ROBOT_WIFI_SSID,
            sizeof(configuration.sta.ssid) - 1u);
    strncpy((char *)configuration.sta.password, CONFIG_ROBOT_WIFI_PASSWORD,
            sizeof(configuration.sta.password) - 1u);
    configuration.sta.threshold.authmode = CONFIG_ROBOT_WIFI_PASSWORD[0] == '\0'
                                               ? WIFI_AUTH_OPEN
                                               : WIFI_AUTH_WPA2_PSK;
    configuration.sta.pmf_cfg.capable = true;
    configuration.sta.pmf_cfg.required = false;
    if ((status = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK ||
        (status = esp_wifi_set_config(WIFI_IF_STA, &configuration)) != ESP_OK ||
        (status = esp_wifi_start()) != ESP_OK) return status;
    return xTaskCreate(wifi_tcp_task, "wifi_tcp", 5120, NULL, 8, NULL) == pdPASS
               ? ESP_OK : ESP_ERR_NO_MEM;
#endif
}

esp_err_t esp32_wifi_tcp_write(const uint8_t *data, size_t length,
                               uint32_t timeout_ms)
{
    if (data == NULL || length == 0u || s_socket_mutex == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    const struct timeval timeout = {
        .tv_sec = (long)(timeout_ms / 1000u),
        .tv_usec = (long)((timeout_ms % 1000u) * 1000u),
    };
    esp_err_t status = ESP_OK;
    xSemaphoreTake(s_socket_mutex, portMAX_DELAY);
    if (s_client_socket < 0) {
        status = ESP_ERR_INVALID_STATE;
    } else {
        (void)setsockopt(s_client_socket, SOL_SOCKET, SO_SNDTIMEO,
                         &timeout, sizeof(timeout));
        size_t sent = 0;
        while (sent < length) {
            const int result = send(s_client_socket, data + sent, length - sent, 0);
            if (result <= 0) {
                status = errno == EAGAIN || errno == EWOULDBLOCK
                             ? ESP_ERR_TIMEOUT : ESP_FAIL;
                break;
            }
            sent += (size_t)result;
        }
    }
    xSemaphoreGive(s_socket_mutex);
    return status;
}

bool esp32_wifi_tcp_connected(void)
{
    if (s_socket_mutex == NULL) return false;
    xSemaphoreTake(s_socket_mutex, portMAX_DELAY);
    const bool connected = s_client_socket >= 0;
    xSemaphoreGive(s_socket_mutex);
    return connected;
}

bool esp32_wifi_tcp_has_ip(void)
{
    return s_events != NULL &&
           (xEventGroupGetBits(s_events) & WIFI_GOT_IP_BIT) != 0u;
}
