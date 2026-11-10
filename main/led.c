/*
 * Health LED. Two drivers, chosen per board at compile time:
 *  - CONFIG_OMI_LED_WS2812: RGB LED (SuperMini GPIO21). blue = scanning, green = streaming OK,
 *    amber = buffering in PSRAM, red = spooling to flash / no WiFi. Low brightness.
 *  - plain GPIO LED (blink patterns), or nothing when CONFIG_OMI_LED_GPIO < 0 (XIAO default).
 */
#include "bridge.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#if CONFIG_OMI_LED_GPIO >= 0 && CONFIG_OMI_LED_WS2812
#include "led_strip.h"
#endif

static volatile led_state_t s_state = LED_NO_WIFI;

#if CONFIG_OMI_LED_GPIO >= 0 && CONFIG_OMI_LED_WS2812

static led_strip_handle_t s_strip;

static void rgb(uint32_t r, uint32_t g, uint32_t b)
{
    const uint32_t k = CONFIG_OMI_LED_BRIGHTNESS;
    r = r * k / 255;
    g = g * k / 255;
    b = b * k / 255;
#if CONFIG_OMI_LED_WS2812_RGB_ORDER
    uint32_t t = r; r = g; g = t;   /* driver sends GRB; swap for RGB-order parts */
#endif
    led_strip_set_pixel(s_strip, 0, r, g, b);
    led_strip_refresh(s_strip);
}

static void led_task(void *arg)
{
    led_state_t last = (led_state_t)-1;
    for (;;) {
        led_state_t s = s_state;
        if (s != last) {
            switch (s) {
            case LED_IDLE: rgb(0, 0, 255); break;       /* blue: scanning/connecting */
            case LED_STREAMING: rgb(0, 255, 0); break;  /* green */
            case LED_BACKLOG: rgb(255, 110, 0); break;  /* amber: PSRAM buffering */
            case LED_SPOOLING:                          /* red: flash spool */
            case LED_NO_WIFI:
            default: rgb(255, 0, 0); break;             /* red: error */
            }
            last = s;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void led_init(void)
{
    led_strip_config_t cfg = {
        .strip_gpio_num = CONFIG_OMI_LED_GPIO,
        .max_leds = 1,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
    };
    led_strip_rmt_config_t rmt = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
    };
    if (led_strip_new_rmt_device(&cfg, &rmt, &s_strip) != ESP_OK) {
        ESP_LOGE("led", "WS2812 init failed on GPIO%d", CONFIG_OMI_LED_GPIO);
        return;
    }
    led_strip_clear(s_strip);
    xTaskCreate(led_task, "led", 3072, NULL, 2, NULL);
}

#elif CONFIG_OMI_LED_GPIO >= 0

static void led_out(int on)
{
#if CONFIG_OMI_LED_ACTIVE_LOW
    on = !on;
#endif
    gpio_set_level(CONFIG_OMI_LED_GPIO, on);
}

static void led_task(void *arg)
{
    int t = 0;
    for (;;) {
        int on;
        switch (s_state) {
        case LED_STREAMING: on = 1; break;
        case LED_IDLE: on = (t / 10) % 2; break;        /* 1 s on / 1 s off */
        case LED_BACKLOG:
        case LED_SPOOLING: on = (t / 2) % 2; break;     /* 5 Hz */
        default: on = (t % 20) == 0; break;             /* blip every 2 s */
        }
        led_out(on);
        t++;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void led_init(void)
{
    gpio_reset_pin(CONFIG_OMI_LED_GPIO);
    gpio_set_direction(CONFIG_OMI_LED_GPIO, GPIO_MODE_OUTPUT);
    xTaskCreate(led_task, "led", 2048, NULL, 2, NULL);
}

#else

void led_init(void) {}

#endif

void led_set(led_state_t s) { s_state = s; }
