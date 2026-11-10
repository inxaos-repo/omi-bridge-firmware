idf_component_register(
    SRCS "app_main.c" "config.c" "led.c" "net.c" "spool.c" "uplink.c" "ble_central.c" "ring_sync.c"
    INCLUDE_DIRS "."
    REQUIRES omi_proto omi_sizing
    PRIV_REQUIRES bt nvs_flash esp_wifi esp_netif esp_event esp_http_client
                  esp_timer esp_app_format driver vfs esp_psram esp_ringbuf joltwallet__littlefs espressif__led_strip)
