/*
 * omi-bridge: ESP32-S3 NimBLE central that forwards Omi pendant Opus audio to
 * omi-monologue-pipeline's ingest (POST /v1/frames). See README.md.
 */
#include "bridge.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "nvs_flash.h"

static const char *TAG = "main";

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    const esp_app_desc_t *app = esp_app_get_description();
    ESP_LOGI(TAG, "omi-bridge %s (IDF %s), PSRAM %u KB", app->version, app->idf_ver,
             esp_psram_is_initialized() ? (unsigned)(esp_psram_get_size() / 1024) : 0);

    config_load();
    led_init();
    spool_init();
    uplink_init();
    net_start();
    uplink_start();
    ring_sync_init();
    ble_central_start();
}
