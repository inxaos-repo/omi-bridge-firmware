#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "bridge.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

static const char *TAG = "net";
static EventGroupHandle_t s_ev;
#define WIFI_UP BIT0

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_ev, WIFI_UP);
        ESP_LOGW(TAG, "wifi disconnected, reconnecting");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = data;
        ESP_LOGI(TAG, "got ip " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_ev, WIFI_UP);
    }
}

void net_start(void)
{
    s_ev = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, NULL));

    wifi_config_t wc = {0};
    strlcpy((char *)wc.sta.ssid, g_cfg.wifi_ssid, sizeof(wc.sta.ssid));
    strlcpy((char *)wc.sta.password, g_cfg.wifi_pass, sizeof(wc.sta.password));
    wc.sta.threshold.authmode = g_cfg.wifi_pass[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    /* BLE coexistence requires modem sleep (default WIFI_PS_MIN_MODEM) */
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_sntp_config_t sc = ESP_NETIF_SNTP_DEFAULT_CONFIG(g_cfg.ntp);
    esp_netif_sntp_init(&sc);
    if (!g_cfg.wifi_ssid[0]) ESP_LOGE(TAG, "no WiFi SSID configured (menuconfig or NVS omi/wifi_ssid)");
}

bool net_wifi_up(void) { return s_ev && (xEventGroupGetBits(s_ev) & WIFI_UP); }

bool net_time_valid(void) { return time(NULL) > 1700000000; }

uint32_t net_epoch(void) { return (uint32_t)time(NULL); }
