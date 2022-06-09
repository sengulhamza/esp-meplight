#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "core_includes.h"
#include "app_config.h"
#include "app_types.h"
#include "led_strip.h"

#define LED_STRIP_RMT_TX_CHANNEL        RMT_CHANNEL_0
#define LED_STRIP_CHASE_SPEED_MS        (50)
#define LED_STRIP_DATA_IN_IO_NUM        18
#define LED_STRIP_LED_NUMBER            61
#define LED_STRIP_BLUE_LED_BEGIN        0
#define LED_STRIP_BLUE_LED_END          22
#define LED_STRIP_WHITE_RED_LED_BEGIN   23
#define LED_STRIP_WHITE_BLUE_LED_BEGIN  30
#define LED_STRIP_RED_LED_BEGIN         37
#define LED_STRIP_RED_LED_END           60

#define LED_DEFAULT_EFFECT_IS_FLASH
static const char *TAG = "led_strip";
static led_strip_t *s_strip = NULL;

/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}

static void strip_clear(led_strip_t *strip)
{
    ESP_ERROR_CHECK(strip->refresh(strip, 100));
    vTaskDelay(pdMS_TO_TICKS(LED_STRIP_CHASE_SPEED_MS));
    strip->clear(strip, 50);
    vTaskDelay(pdMS_TO_TICKS(LED_STRIP_CHASE_SPEED_MS));
}

static void strip_blink_blue(led_strip_t *strip, uint8_t blink_times)
{
    for (uint8_t i =  0; i <= blink_times; i++) {
        for (int j = 0; j < LED_STRIP_LED_NUMBER; j ++) {
            if (j >= LED_STRIP_BLUE_LED_BEGIN && j < LED_STRIP_BLUE_LED_END + 1) {
                ESP_ERROR_CHECK(strip->set_pixel(strip, j, 0, 0, 255));
            }
            if (j >= LED_STRIP_WHITE_BLUE_LED_BEGIN && j < LED_STRIP_RED_LED_BEGIN + 1) {
                ESP_ERROR_CHECK(strip->set_pixel(strip, j, 255, 255, 255));
            }
        }
        strip_clear(strip);
    }
}

static void strip_blink_red(led_strip_t *strip, uint8_t blink_times)
{
    for (uint8_t i =  0; i <= blink_times; i++) {
        for (int j = 0; j < LED_STRIP_LED_NUMBER; j ++) {
            if (j > LED_STRIP_RED_LED_BEGIN && j < LED_STRIP_RED_LED_END + 1) {
                ESP_ERROR_CHECK(strip->set_pixel(strip, j, 255, 0, 0));
            }
            if (j >= LED_STRIP_WHITE_RED_LED_BEGIN && j < LED_STRIP_WHITE_BLUE_LED_BEGIN + 1) {
                ESP_ERROR_CHECK(strip->set_pixel(strip, j, 255, 255, 255));
            }
        }
        strip_clear(strip);
    }
}

static void strip_init_task(void *params)
{
    (void)params;
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(s_strip->clear(s_strip, 100));
    ESP_LOGI(TAG, "led task (%s) started.", __func__);

#ifdef LED_DEFAULT_EFFECT_IS_FLASH
    while (pdTRUE)
    {
        strip_blink_blue(s_strip, 3);
        strip_blink_red(s_strip, 3);
        vTaskDelay(pdMS_TO_TICKS(LED_STRIP_CHASE_SPEED_MS));

    }
#else
    uint32_t red = 0, green = 0, blue = 0;
    uint16_t hue = 0, start_rgb = 0;
    while (pdTRUE) {
        for (int i = 0; i < 3; i++) {
            for (int j = i; j < LED_STRIP_LED_NUMBER; j += 3) {
                // Build RGB values
                hue = j * 360 / LED_STRIP_LED_NUMBER + start_rgb;
                led_strip_hsv2rgb(hue, 100, 100, &red, &green, &blue);
                // Write RGB values to strip driver
                ESP_ERROR_CHECK(s_strip->set_pixel(s_strip, j, red, green, blue));
            }
            // Flush RGB values to LEDs
            ESP_ERROR_CHECK(s_strip->refresh(s_strip, 100));
            vTaskDelay(pdMS_TO_TICKS(LED_STRIP_CHASE_SPEED_MS));
            s_strip->clear(s_strip, 50);
            vTaskDelay(pdMS_TO_TICKS(LED_STRIP_CHASE_SPEED_MS));
        }
        start_rgb += 60;
    }
#endif
}

esp_err_t strip_process_start(void)
{
    if (s_strip)
    {
        free((void*) s_strip);
    }

    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(LED_STRIP_DATA_IN_IO_NUM, LED_STRIP_RMT_TX_CHANNEL);
    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(LED_STRIP_LED_NUMBER, (led_strip_dev_t)config.channel);
    s_strip = led_strip_new_rmt_ws2812(&strip_config);

    if (!s_strip) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
        return ESP_FAIL;
    }

    xTaskCreate(strip_init_task,
                CORE_STRIP_INIT_TASK_NAME,
                CORE_STRIP_INIT_TASK_STACK,
                NULL,
                CORE_STRIP_INIT_TASK_PRIO,
                NULL);

    return ESP_OK;
}
