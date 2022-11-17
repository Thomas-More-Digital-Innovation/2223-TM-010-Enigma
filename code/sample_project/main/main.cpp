#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "driver/spi_master.h"
#include <string.h>

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
#define KEYBOARD_CS GPIO_NUM_21
#define KEYBOARDWRITEADDR 0x40
#define KEYBOARDREADADDR 0x41
unsigned char  MCP_state = 0; 
unsigned char  MCP_stateOriginal = 0;

char keyboardPoints[3][3] = {{'a','b','c'}, {'d','e','f'},{'g','h','i'}};

void gpio_setup(gpio_num_t pinnumber,gpio_mode_t mode, bool on_or_off, bool pullup_en, bool pulldown_en){
    gpio_pad_select_gpio(pinnumber);//GPIO_NUM_13 is gpio pin 13
    ESP_ERROR_CHECK(gpio_set_direction(pinnumber,mode));
    ESP_ERROR_CHECK(gpio_set_level(pinnumber,on_or_off));
    if (pullup_en){
        gpio_pullup_en(pinnumber);
    }else{
        gpio_pullup_dis(pinnumber);
    }

    if(pulldown_en){
        gpio_pulldown_en(pinnumber);
    } else{
        gpio_pulldown_dis(pinnumber);
    }
}

void spi_cmd(spi_device_handle_t spi,const uint8_t addresExpander,const uint8_t addresRegister,const uint8_t cmd)
{
    gpio_set_level(KEYBOARD_CS,0);//cs needs to be low in order to start sending data
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.flags=SPI_TRANS_USE_TXDATA;   //now I can use tx_data
    t.length=24;                     //Command is 8 bits
    t.tx_data[0] = addresExpander;   //the addres of the expander vb 0x40
    t.tx_data[1] = addresRegister;   //the addres of the register
    t.tx_data[2] = cmd;              //what to put in the register
    ret=spi_device_polling_transmit(spi, &t);  //transmit
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(error, "error code %d!", ret);
    gpio_set_level(KEYBOARD_CS,1);
}


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
        //de cs pin van de keyboard io expander instellen als output
        gpio_setup(KEYBOARD_CS,GPIO_MODE_OUTPUT,1,0,0);
		configure_ledstrip();
		set_ledstrip(0);//makes the first led burn

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
            .clock_speed_hz=10*1000*1000, //400kHz
            .spics_io_num=KEYBOARD_CS,               //CS pin
            .queue_size=7                         //We want to be able to queue 7 transactions at a time
        };

        
        ret = spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);//this initializes the spi bus
        ESP_ERROR_CHECK(ret);//if this gives an error it wil show in the terminal
        ESP_LOGI(error, "error code %d!", ret);//this way I get extra confirmation if the error code is 0
        
        ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);//this adds 1 io expander to the bus (with cs pin 21)
        ESP_ERROR_CHECK(ret);
        ESP_LOGI(error, "error code %d!", ret);

        //de B kant op output zetten
        spi_cmd(spi,KEYBOARDWRITEADDR,IODIRB,0x00);

        vTaskDelay(500/portTICK_PERIOD_MS);

        //de B outputs zetten
        spi_cmd(spi,KEYBOARDWRITEADDR,GPIOB,0b00001111);

        
        ESP_LOGI(LEDSTRIP, "start delay");
        vTaskDelay(500000/portTICK_PERIOD_MS);
        ESP_LOGI(LEDSTRIP, "stop delay");

	}
}
