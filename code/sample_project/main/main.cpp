#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "driver/spi_master.h"

static const char *error = "error";

//ledstrip variables
static const char *LEDSTRIP = "ledstrip";
#define ledstrip_GPIO GPIO_NUM_14
#define CONFIG_BLINK_LED_RMT_CHANNEL 1
static led_strip_t *ledstrip;

//for every io expander
#define IODIRA 0x00
#define IODIRB 0x01
#define GPIOA 0x12
#define GPIOB 0x13
//only for keyboard expander
#define KEYBOARD_CS 21
#define KEYBOARDWRITEADDR 0x40
#define KEYBOARDREADADDR 0x41
unsigned char  MCP_state = 0; 
unsigned char  MCP_stateOriginal = 0;

char keyboardPoints[3][3] = {{'a','b','c'}, {'d','e','f'},{'g','h','i'}};


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
		// configure_ledstrip();
		// set_ledstrip(0);
        // vTaskDelay(5000/portTICK_PERIOD_MS);
        // set_ledstrip(1);

        esp_err_t ret;
        spi_device_handle_t spi;

        spi_bus_config_t buscfg = {
            .mosi_io_num = GPIO_NUM_18,
            .miso_io_num = GPIO_NUM_19,
            .sclk_io_num = GPIO_NUM_5,
            .quadwp_io_num=-1,
            .quadhd_io_num=-1,
            .max_transfer_sz = 32
        };

        spi_device_interface_config_t devcfg = {
            .mode=0,                                //SPI mode 0
            .clock_speed_hz=400*1000, //400kHz
            .spics_io_num=KEYBOARD_CS,               //CS pin
            .queue_size=7                         //We want to be able to queue 7 transactions at a time
        };
        
        //this initializes the spi bus
        ret = spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
        ESP_ERROR_CHECK(ret);//if this gives an error it wil show in the terminal
        ESP_LOGI(error, "error code %d!", ret);//this way I get extra confirmation if the error code is 0

        //this adds 1 io expander to the bus (with cs pin 21)
        ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
        ESP_ERROR_CHECK(ret);
        ESP_LOGI(error, "error code %d!", ret);

        ESP_LOGI(LEDSTRIP, "start delay");
        vTaskDelay(500000/portTICK_PERIOD_MS);
        ESP_LOGI(LEDSTRIP, "stop delay");


	}
}
