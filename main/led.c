#include "bridge.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static volatile led_state_t s_state = LED_NO_WIFI;

static void led_out(int on)
{
#if CONFIG_OMI_LED_GPIO >= 0
#if CONFIG_OMI_LED_ACTIVE_LOW
    on = !on;
#endif
    gpio_set_level(CONFIG_OMI_LED_GPIO, on);
#else
    (void)on;
#endif
}

#if CONFIG_OMI_LED_GPIO >= 0
static void led_task(void *arg)
{
    int t = 0;
    for (;;) {
        int on;
        switch (s_state) {
        case LED_STREAMING: on = 1; break;
        case LED_IDLE: on = (t / 10) % 2; break;        /* 1 s on / 1 s off */
        case LED_BACKLOG: on = (t / 2) % 2; break;      /* 5 Hz */
        default: on = (t % 20) == 0; break;             /* blip every 2 s */
        }
        led_out(on);
        t++;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
#endif

void led_init(void)
{
#if CONFIG_OMI_LED_GPIO >= 0
    gpio_reset_pin(CONFIG_OMI_LED_GPIO);
    gpio_set_direction(CONFIG_OMI_LED_GPIO, GPIO_MODE_OUTPUT);
    xTaskCreate(led_task, "led", 2048, NULL, 2, NULL);
#endif
}

void led_set(led_state_t s) { s_state = s; }
