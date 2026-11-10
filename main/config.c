#include <string.h>

#include "bridge.h"
#include "esp_log.h"
#include "nvs.h"
#include "sdkconfig.h"

static const char *TAG = "config";
bridge_config_t g_cfg;

static void load_str(nvs_handle_t h, const char *key, char *dst, size_t cap, const char *dflt)
{
    strlcpy(dst, dflt, cap);
    if (!h) return;
    size_t len = cap;
    if (nvs_get_str(h, key, dst, &len) == ESP_OK) {
        ESP_LOGI(TAG, "%s from NVS", key);
    } else {
        strlcpy(dst, dflt, cap);
    }
}

void config_load(void)
{
    nvs_handle_t h = 0;
    if (nvs_open("omi", NVS_READONLY, &h) != ESP_OK) h = 0;
    load_str(h, "wifi_ssid", g_cfg.wifi_ssid, sizeof(g_cfg.wifi_ssid), CONFIG_OMI_WIFI_SSID);
    load_str(h, "wifi_pass", g_cfg.wifi_pass, sizeof(g_cfg.wifi_pass), CONFIG_OMI_WIFI_PASSWORD);
    load_str(h, "ingest_url", g_cfg.ingest_url, sizeof(g_cfg.ingest_url), CONFIG_OMI_INGEST_URL);
    load_str(h, "token", g_cfg.token, sizeof(g_cfg.token), CONFIG_OMI_INGEST_TOKEN);
    load_str(h, "device_id", g_cfg.device_id, sizeof(g_cfg.device_id), CONFIG_OMI_DEVICE_ID);
    load_str(h, "pendant_mac", g_cfg.pendant_mac, sizeof(g_cfg.pendant_mac), CONFIG_OMI_PENDANT_MAC);
    load_str(h, "ntp", g_cfg.ntp, sizeof(g_cfg.ntp), CONFIG_OMI_NTP_SERVER);
#ifdef CONFIG_OMI_STORAGE_SYNC
    g_cfg.storage_sync = true;
#else
    g_cfg.storage_sync = false;
#endif
    uint8_t ss;
    if (h && nvs_get_u8(h, "storage_sync", &ss) == ESP_OK) g_cfg.storage_sync = ss != 0;
    if (h) nvs_close(h);

    size_t n = strlen(g_cfg.ingest_url);
    while (n && g_cfg.ingest_url[n - 1] == '/') g_cfg.ingest_url[--n] = 0;

    ESP_LOGI(TAG, "ssid='%s' ingest=%s device=%s mac_filter='%s' token=%s storage_sync=%s",
             g_cfg.wifi_ssid, g_cfg.ingest_url, g_cfg.device_id, g_cfg.pendant_mac,
             g_cfg.token[0] ? "set" : "NONE", g_cfg.storage_sync ? "ON (consumes pendant data)" : "off");
}
