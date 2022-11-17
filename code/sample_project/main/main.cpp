#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "driver/spi_master.h"
#include <string.h>

esp_err_t ret;
spi_device_handle_t spi;
spi_device_handle_t spi2;

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
static const char *keyboardLog = "keyboard";
#define KEYBOARD_CS GPIO_NUM_21
#define KEYBOARDWRITEADDR 0x40
#define KEYBOARDREADADDR 0x41
unsigned char  gpioa_state = 0; 
unsigned char  gpioa_stateOriginal = 0;

char keyboardPoints[3][3] = {{'a','b','c'}, {'d','e','f'},{'g','h','i'}};

//stepper motor
static const char *stepper = "stepper motor";
#define STEPPER_CS GPIO_NUM_32
#define STEPPERWRITEADDR 0x42
#define STEPPERREADADDR 0x43

//////////////////////////
///////FUNCTIONS/////////
/////////////////////////

/////gpio///////////////////////////
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
/////spi///////////////////////////
void spi_write(spi_device_handle_t spi,const uint8_t addresExpander,const uint8_t addresRegister,const uint8_t cmd)
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
    // ESP_LOGI(error, "error code %d!", ret);
    gpio_set_level(KEYBOARD_CS,1);
}


uint8_t spi_read(spi_device_handle_t spi,const uint8_t addresExpander,const uint8_t addresRegister){
  gpio_set_level(KEYBOARD_CS,0);
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));

  uint8_t tx_data[4];
  uint8_t rx_data[4];

  tx_data[0] = addresExpander;
  tx_data[1] = addresRegister;
  tx_data[2] = 0x00;
  tx_data[3] = 0x00;

  t.tx_buffer = tx_data;	
  t.rx_buffer = rx_data;
  t.length=32;

  esp_err_t ret = spi_device_polling_transmit(spi, &t);
  assert( ret == ESP_OK );
  gpio_set_level(KEYBOARD_CS,1);

  return rx_data[2];	
}

/////keyboard///////////////////////////

int readButtonInputs(){
  gpioa_stateOriginal = spi_read(spi, KEYBOARDREADADDR, GPIOA);//read the gpio a register
  gpioa_state = gpioa_stateOriginal;
  vTaskDelay(10/portTICK_PERIOD_MS);

  while (gpioa_state !=0) //as long as the button is pushed keep reading the register
  {
    gpioa_state = spi_read(spi, KEYBOARDREADADDR, GPIOA);
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
  return gpioa_stateOriginal;//when the button is released return what was in the register
}

void setButtonOutput(int binairyNumber){
  spi_write(spi,KEYBOARDWRITEADDR,GPIOB,binairyNumber);  
  vTaskDelay(10/portTICK_PERIOD_MS);
}


void getLetterFromInputs(int column, int row){
  char letter;
  if(column==1){//collumn is the number in GPIOA
    letter = keyboardPoints[0][row];
    ESP_LOGI(keyboardLog, "letter %c", letter);
    
  }else if (column == 2)
  {
    letter = keyboardPoints[1][row];
    ESP_LOGI(keyboardLog, "letter %c", letter);
  } else if (column == 4)
  {
    letter = keyboardPoints[2][row];
    ESP_LOGI(keyboardLog, "letter %c", letter);
  }
}

void keyboard(){
  
  //Set a row high en read the collumn. the combination of row and collumn tells if button is pressend and which one
  setButtonOutput(0b00000001);
  getLetterFromInputs(readButtonInputs(),0);
  setButtonOutput(0b00000010);
  getLetterFromInputs(readButtonInputs(),1);
  setButtonOutput(0b00000100);
  getLetterFromInputs(readButtonInputs(),2);
  
}

/////ledstrip///////////////////////////
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
/////stepperMotor///////////////////////////
void turnMotor(int steps, int motor, int GPIOAB){
  //motor 1 and 2 are on gpioB, 3 on gpioA
  //motor 2 is attatched to the last 4 bits of the GPIOB => <<4
  int step1 = (motor==1 || motor==3) ? 0b00000011 : 0b00000011<<4;
  int step2 = (motor==1 || motor==3) ? 0b00000110 : 0b00000110<<4;
  int step3 = (motor==1 || motor==3) ? 0b00001100 : 0b00001100<<4;
  int step4 = (motor==1 || motor==3) ? 0b00001001 : 0b00001001<<4;

  ESP_LOGI(stepper, "step1 %d!", step1);
  ESP_LOGI(stepper, "step2 %d!", step2);
  ESP_LOGI(stepper, "step3 %d!", step3);
  ESP_LOGI(stepper, "step4 %d!", step4);

  if (steps >0){
    for (size_t i = 0; i < steps; i++)
    {
      spi_write(spi2,STEPPERWRITEADDR,GPIOAB,step1);
      vTaskDelay(10/portTICK_PERIOD_MS);
      spi_write(spi2,STEPPERWRITEADDR,GPIOAB,step2);
      vTaskDelay(10/portTICK_PERIOD_MS);
      spi_write(spi2,STEPPERWRITEADDR,GPIOAB,step3);
      vTaskDelay(10/portTICK_PERIOD_MS);
      spi_write(spi2,STEPPERWRITEADDR,GPIOAB,step4);
      vTaskDelay(10/portTICK_PERIOD_MS);
    }
  } else if (steps <0)
  {
    for (int j = 0; j > steps; j--)
    {
      spi_write(spi2,STEPPERWRITEADDR,GPIOAB,step4);
      vTaskDelay(10/portTICK_PERIOD_MS);
      spi_write(spi2,STEPPERWRITEADDR,GPIOAB,step3);
      vTaskDelay(10/portTICK_PERIOD_MS);
      spi_write(spi2,STEPPERWRITEADDR,GPIOAB,step2);
      vTaskDelay(10/portTICK_PERIOD_MS);
      spi_write(spi2,STEPPERWRITEADDR,GPIOAB,step1);
      vTaskDelay(10/portTICK_PERIOD_MS);
    }
  }
  
}

//////////////////////////
///////MAIN/////////
/////////////////////////


extern "C"
{
	void app_main(void)
	{
    uint8_t dataFromPoll;
    //de cs pin van de keyboard io expander instellen als output
    gpio_setup(KEYBOARD_CS,GPIO_MODE_OUTPUT,1,0,0);
    configure_ledstrip();
    set_ledstrip(0);//makes the first led burn

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

    spi_device_interface_config_t devcfg2 = {
        .mode=0,                                //SPI mode 0
        .clock_speed_hz=10*1000*1000, //400kHz
        .spics_io_num=STEPPER_CS,               //CS pin
        .queue_size=7                         //We want to be able to queue 7 transactions at a time
    };

    ret = spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);//this initializes the spi bus
    ESP_ERROR_CHECK(ret);//if this gives an error it wil show in the terminal
    ESP_LOGI(error, "error code %d!", ret);//this way I get extra confirmation if the error code is 0
    
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);//this adds 1 io expander to the bus (with cs pin 21)
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(error, "error code %d!", ret);

    ret=spi_bus_add_device(HSPI_HOST, &devcfg2, &spi2);//this adds 1 io expander to the bus (with cs pin 21)
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(error, "error code %d!", ret);

    //set the io expanders as in or output
    spi_write(spi,KEYBOARDWRITEADDR,IODIRB,0x00);//all output
    spi_write(spi,KEYBOARDWRITEADDR,IODIRA,0xFF);//all input
    spi_write(spi2,STEPPERWRITEADDR,IODIRB,0x00);
    spi_write(spi2,STEPPERWRITEADDR,IODIRA,0x00);

    vTaskDelay(500/portTICK_PERIOD_MS);

    //set B outputs 
    spi_write(spi,KEYBOARDWRITEADDR,GPIOB,0b00000000);

    turnMotor(80,1,GPIOB);
    turnMotor(-80,1,GPIOB);
    turnMotor(80,2,GPIOB);
    

    // for (;;)//endless loop
    // {
    //   keyboard();
    // }
    
    ESP_LOGI(LEDSTRIP, "start delay");
    vTaskDelay(500000/portTICK_PERIOD_MS);
    ESP_LOGI(LEDSTRIP, "stop delay");

	}
}
