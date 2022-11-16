#include <stdio.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

static const char *LEDSTRIP = "ledstrip";

#define ledstrip_GPIO GPIO_NUM_14
#define CONFIG_BLINK_LED_RMT_CHANNEL 1

static led_strip_t *ledstrip;


static void set_ledstrip(int ledNumber)
{
    ledstrip->clear(ledstrip, 50);
    /* If the addressable LED is enabled */
    ESP_LOGI(LEDSTRIP, "Turning ON LED %d!", ledNumber);
    ledstrip->set_pixel(ledstrip, ledNumber, 255, 255, 255);
    /* Refresh the strip to send data */
    ledstrip->refresh(ledstrip, 100);
}

static void configure_ledstrip(void)
{
    ESP_LOGI(LEDSTRIP, "configuring the ledstrip");
    /* initialise the led strip with rmt channel, gpio for data line and amount om leds on the strip*/
    ledstrip = led_strip_init(CONFIG_BLINK_LED_RMT_CHANNEL, ledstrip_GPIO, 26);
    /* Set all LED off to clear all pixels */
    ledstrip->clear(ledstrip, 50);
}

extern "C"
{
	void app_main(void)
	{
		configure_ledstrip();
		set_ledstrip(0);
        vTaskDelay(5000/portTICK_PERIOD_MS);
        set_ledstrip(1);
	}
}
