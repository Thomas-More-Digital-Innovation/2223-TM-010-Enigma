#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "driver/spi_master.h"
#include <string.h>
#include <vector>

unsigned short int choice_rotor_left = 1;
unsigned short int choice_rotor_mid = 2;
unsigned short int choice_rotor_right = 3;

//regpresents what letters are attatched to te switchboard io expander registers
int switchboard3RegA[8] = {1,2,3,4,5,6,7,8};
int switchboard3RegB[8] = {9,10,11,12,13,14,15,16};
int switchboard4RegA[8] = {17,18,19,20,21,22,23,24};
int switchboard4RegB[2] = {25,26};

std::vector<unsigned short int> switchboardA = {};
std::vector<unsigned short int> switchboardB = {};

esp_err_t ret;
spi_device_handle_t spi;
spi_device_handle_t spi2;
spi_device_handle_t spi3;
spi_device_handle_t spi4;
spi_device_handle_t spi5;

static const char *error = "error";
static const char *TAG = "test";
static const char *SPI = "spi";

//ledstrip variables
static const char *LEDSTRIP = "ledstrip";
#define ledstrip_GPIO GPIO_NUM_14
#define CONFIG_BLINK_LED_RMT_CHANNEL 1
// static led_strip_t *ledstrip;

//for every io expander
#define IODIRA 0x00
#define IODIRB 0x01
#define GPIOA 0x12
#define GPIOB 0x13
//only for keyboard expander
static const char *keyboardLog = "keyboard";
#define KEYBOARD_CS GPIO_NUM_14
#define KEYBOARDWRITEADDR 0x40
#define KEYBOARDREADADDR 0x41
unsigned char  gpioa_state = 0; 
unsigned char  gpioa_stateOriginal = 0;
int global_state = 0;

char keyboardPoints[4][7] = {{'q','w','e','r','t','z','u'}, {'a','s','d','f','g','h','j'},{'p','y','x','c','v','b','n'},{'i','o','k','m','l','1','2'}};//'1' en '2' is enkel om [4][7] te doen kloppen.

//stepper motor
static const char *stepper = "stepper motor";
#define STEPPER_CS GPIO_NUM_32
#define STEPPERWRITEADDR 0x42
#define STEPPERREADADDR 0x43

//switchboard
static const char *switchboard = "switchboard";
#define SWITCHBOARD1_CS GPIO_NUM_12
#define SWITCHBOARD2_CS GPIO_NUM_27

#define SWITCHBOARD1WRITEADDR 0x44
#define SWITCHBOARD1READADDR 0X45
#define SWITCHBOARD2WRITEADDR 0X46
#define SWITCHBOARD2READADDR 0X47

unsigned char  read_a = 0; 
unsigned char  read_b = 0;

//rotary encoders
int GpioRegister5A;
int GpioRegister5B;
int A1LastState;  
int A2LastState;  
int A3LastState;  

static const char *rotary = "rotary";
#define ROTARY_CS GPIO_NUM_15

#define ROTARYWRITEADDR 0x48
#define ROTARYREADADDR 0X49

//////////////////////////
///////FUNCTIONS/////////
/////////////////////////

/////gpio///////////////////////////
void GpioSetup(gpio_num_t pinnumber,gpio_mode_t mode, bool on_or_off, bool pullup_en, bool pulldown_en){
    esp_rom_gpio_pad_select_gpio(pinnumber);//GPIO_NUM_13 is gpio pin 13
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
void WriteSpi(spi_device_handle_t spi,const uint8_t addresExpander,const uint8_t addresRegister,const uint8_t cmd,gpio_num_t cs_pin)
{
    gpio_set_level(cs_pin,0);//cs needs to be low in order to start sending data
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
    gpio_set_level(cs_pin,1);
}


uint8_t ReadSpi(spi_device_handle_t spi,const uint8_t addresExpander,const uint8_t addresRegister,gpio_num_t cs_pin){
  gpio_set_level(cs_pin,0);
  

  uint8_t tx_data[4] ={ addresExpander, addresRegister, 0, 0 };
  uint8_t rx_data[4];

  spi_transaction_t t;
  memset(&t, 0, sizeof(t));

  t.tx_buffer = tx_data;	
  t.rx_buffer = rx_data;
  t.length=32;

  esp_err_t ret = spi_device_polling_transmit(spi, &t);
  assert( ret == ESP_OK );
  gpio_set_level(cs_pin,1);

  return rx_data[2];	
}

//this function rotates a byte to the left by 1
unsigned char rotl(unsigned char c,int iteration)//https://stackoverflow.com/questions/19204750/how-do-i-perform-a-circular-rotation-of-a-byte
{
  if (iteration)//if this is 0 the number stays the same, else it's rotated
  {
    return (c << 1) | (c >> 7);
  }
  
  return c;
}

//https://www.geeksforgeeks.org/extract-k-bits-given-position-number/
int GetBit(int number, int amountBits, int startingPos)
{
    return (((1 << amountBits) - 1) & (number >> (startingPos - 1)));
}

int GetBitHigh(int number){
  for (size_t i = 1; i < 9; i++)
  {
    if (GetBit(number,1,i))
    {
      return i;
    }
  }
  return 0;
  
}

int GetBitHigh(int number,int ignoreNumber){
  for (size_t i = 1; i < 9; i++)
  {
    if (i!=ignoreNumber)//the ignore number is the nr of the pin that is set as a high output
    {
      if (GetBit(number,1,i))
      {
        return i;
      }
    }
  }
  return 0;
  
}

void printRegister(int status){
  // aState = ReadSpi(spi4,SWITCHBOARD2READADDR,GPIOA,SWITCHBOARD2_CS);
  ESP_LOGI(rotary, "VOLLEDIG: %d!", status);
  ESP_LOGI(rotary, "1: %d!", GetBit(status,1,1));
  ESP_LOGI(rotary, "2: %d!", GetBit(status,1,2));
  ESP_LOGI(rotary, "3: %d!", GetBit(status,1,3));
  ESP_LOGI(rotary, "4: %d!", GetBit(status,1,4));
  ESP_LOGI(rotary, "5: %d!", GetBit(status,1,5));
  ESP_LOGI(rotary, "6: %d!", GetBit(status,1,6));
  ESP_LOGI(rotary, "7: %d!", GetBit(status,1,7));
  ESP_LOGI(rotary, "8: %d!", GetBit(status,1,8));
}
/////keyboard///////////////////////////

int readButtonInputs(){
  gpioa_stateOriginal = ReadSpi(spi, KEYBOARDREADADDR, GPIOA,KEYBOARD_CS);//read the gpio a register
  gpioa_state = gpioa_stateOriginal;
  vTaskDelay(10/portTICK_PERIOD_MS);

  while (gpioa_state !=0) //as long as the button is pushed keep reading the register
  {
    ESP_LOGI(keyboardLog, "gpio %d", gpioa_stateOriginal);
    gpioa_state = ReadSpi(spi, KEYBOARDREADADDR, GPIOA,KEYBOARD_CS);
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
  global_state = gpioa_state;
  return gpioa_stateOriginal;//when the button is released return what was in the register
}

void setButtonOutput(int binairyNumber){
  WriteSpi(spi,KEYBOARDWRITEADDR,GPIOB,binairyNumber,KEYBOARD_CS);  
  vTaskDelay(10/portTICK_PERIOD_MS);
}


void getLetterFromInputs(int column, int row){
  char letter;
  //collumn is the number in GPIOA
  if(column==192){
    letter = keyboardPoints[row][0];
    ESP_LOGI(keyboardLog, "letter %c", letter);
    
  }else if (column == 1)
  {
    letter = keyboardPoints[row][1];
    ESP_LOGI(keyboardLog, "letter %c", letter);
  } else if (column == 2)
  {
    letter = keyboardPoints[row][2];
    ESP_LOGI(keyboardLog, "letter %c", letter);
  } else if (column == 4)
  {
    letter = keyboardPoints[row][3];
    ESP_LOGI(keyboardLog, "letter %c", letter);
  } else if (column == 8)
  {
    letter = keyboardPoints[row][4];
    ESP_LOGI(keyboardLog, "letter %c", letter);
  }else if (column == 16)
  {
    letter = keyboardPoints[row][5];
    ESP_LOGI(keyboardLog, "letter %c", letter);
  }else if (column == 32)
  {
    letter = keyboardPoints[row][6];
    ESP_LOGI(keyboardLog, "letter %c", letter);
  }
  
}



/////ledstrip///////////////////////////
// static void set_ledstrip(int ledNumber)
// {
//   ledstrip->clear(ledstrip, 50);
//   /* If the addressable LED is enabled */
//   ESP_LOGI(LEDSTRIP, "Turning ON LED %d!", ledNumber);
//   ledstrip->set_pixel(ledstrip, ledNumber, 0, 255, 0);
//   /* Refresh the strip to send data */
//   ledstrip->refresh(ledstrip, 100);
// }

// static void set_multiple_leds(int ledNumber1, int ledNumber2, int ledNumber3)
// {
//   ledstrip->clear(ledstrip, 50);
//   /* If the addressable LED is enabled */
//   ESP_LOGI(LEDSTRIP, "Turning ON LED %d, %d,%d!", ledNumber1,ledNumber2,ledNumber3);
//   ledstrip->set_pixel(ledstrip, ledNumber1, 16, 16, 16);
//   ledstrip->set_pixel(ledstrip, ledNumber2, 16, 16, 16);
//   ledstrip->set_pixel(ledstrip, ledNumber3, 16, 16, 16);
//   /* Refresh the strip to send data */
//   ledstrip->refresh(ledstrip, 100);
// }

// static void configure_ledstrip(void)
// {
//   ESP_LOGI(LEDSTRIP, "configuring the ledstrip");
//   /* initialise the led strip with rmt channel, gpio for data line and amount om leds on the strip*/
//   ledstrip = led_strip_init(CONFIG_BLINK_LED_RMT_CHANNEL, ledstrip_GPIO, 26);
//   /* Set all LED off to clear all pixels */
//   ledstrip->clear(ledstrip, 50);
// }
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
      WriteSpi(spi2,STEPPERWRITEADDR,GPIOAB,step1,STEPPER_CS);
      vTaskDelay(10/portTICK_PERIOD_MS);
      WriteSpi(spi2,STEPPERWRITEADDR,GPIOAB,step2,STEPPER_CS);
      vTaskDelay(10/portTICK_PERIOD_MS);
      WriteSpi(spi2,STEPPERWRITEADDR,GPIOAB,step3,STEPPER_CS);
      vTaskDelay(10/portTICK_PERIOD_MS);
      WriteSpi(spi2,STEPPERWRITEADDR,GPIOAB,step4,STEPPER_CS);
      vTaskDelay(10/portTICK_PERIOD_MS);
    }
  } else if (steps <0)
  {
    for (int j = 0; j > steps; j--)
    {
      WriteSpi(spi2,STEPPERWRITEADDR,GPIOAB,step4,STEPPER_CS);
      vTaskDelay(10/portTICK_PERIOD_MS);
      WriteSpi(spi2,STEPPERWRITEADDR,GPIOAB,step3,STEPPER_CS);
      vTaskDelay(10/portTICK_PERIOD_MS);
      WriteSpi(spi2,STEPPERWRITEADDR,GPIOAB,step2,STEPPER_CS);
      vTaskDelay(10/portTICK_PERIOD_MS);
      WriteSpi(spi2,STEPPERWRITEADDR,GPIOAB,step1,STEPPER_CS);
      vTaskDelay(10/portTICK_PERIOD_MS);
    }
  }
  
}


void readSwitchboard(){
  int allInput = 0b11111111;
  int changingOutput = 0b11111110;
  int outputLevel = 0b00000001;
  int changedOutput;
  int firstLetter,secondLetter;

  int reg3A,reg3B,reg4A,reg4B;
  int highBitOutput,highBitMatch;

  //make sure the switchboard vectors are empty before they are read
  switchboardA.clear();
  switchboardB.clear();
  //spi3 with register A changes output
  for (size_t i = 0; i < 8 ; i++)
  {
    //this function shifts the position of the 0(output) in the byte 1 to the left
    changedOutput = rotl(changingOutput,i);
    //set only 1 as output, all the rest are inputs
    WriteSpi(spi3,SWITCHBOARD1WRITEADDR,IODIRA,changedOutput,SWITCHBOARD1_CS);// one after the other the pins become an output
    WriteSpi(spi3,SWITCHBOARD1WRITEADDR,IODIRB,allInput,SWITCHBOARD1_CS);
    WriteSpi(spi4,SWITCHBOARD2WRITEADDR,IODIRA,allInput,SWITCHBOARD2_CS);
    WriteSpi(spi4,SWITCHBOARD2WRITEADDR,IODIRB,allInput,SWITCHBOARD2_CS);

    //read all the switchboard registers
    reg3A = ReadSpi(spi3,SWITCHBOARD1READADDR,GPIOA,SWITCHBOARD1_CS);
    reg3B = ReadSpi(spi3,SWITCHBOARD1READADDR,GPIOB,SWITCHBOARD1_CS);

    reg4A = ReadSpi(spi4,SWITCHBOARD2READADDR,GPIOA,SWITCHBOARD2_CS);
    reg4B = ReadSpi(spi4,SWITCHBOARD2READADDR,GPIOB,SWITCHBOARD2_CS);

    highBitOutput = GetBitHigh(changedOutput);//need to know which bit was set to high so it can be ignored when searching for the high input

    if(GetBitHigh(reg3A,highBitOutput)){//if there wasn't a high bit somewhere this function would return 0 and the if wouln't be true
      highBitMatch = GetBitHigh(reg3A,highBitOutput);//look for high input but ignore the high output
      firstLetter = switchboard3RegA[highBitOutput-1];
      secondLetter = switchboard3RegA[highBitMatch-1];
    }else if (GetBitHigh(reg3B))
    {
      highBitMatch = GetBitHigh(reg3B);
      firstLetter = switchboard3RegA[highBitOutput-1];
      secondLetter = switchboard3RegB[highBitMatch-1];
    }else if (GetBitHigh(reg4A))
    {
      highBitMatch = GetBitHigh(reg4A);
      firstLetter = switchboard3RegA[highBitOutput-1];
      secondLetter = switchboard4RegA[highBitMatch-1];
    }else if (GetBitHigh(reg4B))
    {
      highBitMatch = GetBitHigh(reg4B);
      firstLetter = switchboard3RegA[highBitOutput-1];
      secondLetter = switchboard4RegB[highBitMatch-1];
    }else{
      highBitMatch = 0;
    }

    if (highBitMatch)
    {
      switchboardA.push_back(firstLetter);
      switchboardB.push_back(secondLetter);
    }
  }
  //spi3 with register B changes output
  for (size_t i = 0; i < 8 ; i++)
  {
    changedOutput = rotl(changingOutput,i);
    //set only 1 as output
    WriteSpi(spi3,SWITCHBOARD1WRITEADDR,IODIRA,allInput,SWITCHBOARD1_CS);
    WriteSpi(spi3,SWITCHBOARD1WRITEADDR,IODIRB,changedOutput,SWITCHBOARD1_CS);// one after the other the pins become an output
    WriteSpi(spi4,SWITCHBOARD2WRITEADDR,IODIRA,allInput,SWITCHBOARD2_CS);
    WriteSpi(spi4,SWITCHBOARD2WRITEADDR,IODIRB,allInput,SWITCHBOARD2_CS);

    //read all the switchboard registers
    reg3A = ReadSpi(spi3,SWITCHBOARD1READADDR,GPIOA,SWITCHBOARD1_CS);
    reg3B = ReadSpi(spi3,SWITCHBOARD1READADDR,GPIOB,SWITCHBOARD1_CS);

    reg4A = ReadSpi(spi4,SWITCHBOARD2READADDR,GPIOA,SWITCHBOARD2_CS);
    reg4B = ReadSpi(spi4,SWITCHBOARD2READADDR,GPIOB,SWITCHBOARD2_CS);

    highBitOutput = GetBitHigh(changedOutput);//need to know which bit was set to high so it can be ignored when searching for the high input

    if(GetBitHigh(reg3A)){
      highBitMatch = GetBitHigh(reg3A);//look for high input but ignore the high output
      firstLetter = switchboard3RegB[highBitOutput-1];
      secondLetter = switchboard3RegA[highBitMatch-1];
    }else if (GetBitHigh(reg3B,highBitOutput))
    {
      highBitMatch = GetBitHigh(reg3B,highBitOutput);
      firstLetter = switchboard3RegB[highBitOutput-1];
      secondLetter = switchboard3RegB[highBitMatch-1];
    }else if (GetBitHigh(reg4A))
    {
      highBitMatch = GetBitHigh(reg4A);
      firstLetter = switchboard3RegB[highBitOutput-1];
      secondLetter = switchboard4RegA[highBitMatch-1];
    }else if (GetBitHigh(reg4B))
    {
      highBitMatch = GetBitHigh(reg4B);
      firstLetter = switchboard3RegB[highBitOutput-1];
      secondLetter = switchboard4RegB[highBitMatch-1];
    }else{
      highBitMatch = 0;
    }

    if (highBitMatch)
    {
      switchboardA.push_back(firstLetter);
      switchboardB.push_back(secondLetter);
    }
  }

  //spi4 with register A changes output
  for (size_t i = 0; i < 8 ; i++)
  {
    changedOutput = rotl(changingOutput,i);
    //set only 1 as output
    WriteSpi(spi3,SWITCHBOARD1WRITEADDR,IODIRA,allInput,SWITCHBOARD1_CS);
    WriteSpi(spi3,SWITCHBOARD1WRITEADDR,IODIRB,allInput,SWITCHBOARD1_CS);// one after the other the pins become an output
    WriteSpi(spi4,SWITCHBOARD2WRITEADDR,IODIRA,changedOutput,SWITCHBOARD2_CS);
    WriteSpi(spi4,SWITCHBOARD2WRITEADDR,IODIRB,allInput,SWITCHBOARD2_CS);

    //read all the switchboard registers
    reg3A = ReadSpi(spi3,SWITCHBOARD1READADDR,GPIOA,SWITCHBOARD1_CS);
    reg3B = ReadSpi(spi3,SWITCHBOARD1READADDR,GPIOB,SWITCHBOARD1_CS);

    reg4A = ReadSpi(spi4,SWITCHBOARD2READADDR,GPIOA,SWITCHBOARD2_CS);
    reg4B = ReadSpi(spi4,SWITCHBOARD2READADDR,GPIOB,SWITCHBOARD2_CS);

    highBitOutput = GetBitHigh(changedOutput);//need to know which bit was set to high so it can be ignored when searching for the high input

    if(GetBitHigh(reg3A)){
      highBitMatch = GetBitHigh(reg3A);//look for high input but ignore the high output
      firstLetter = switchboard4RegA[highBitOutput-1];
      secondLetter = switchboard3RegA[highBitMatch-1];
    }else if (GetBitHigh(reg3B))
    {
      highBitMatch = GetBitHigh(reg3B);
      firstLetter = switchboard4RegA[highBitOutput-1];
      secondLetter = switchboard3RegB[highBitMatch-1];
    }else if (GetBitHigh(reg4A,highBitOutput))
    {
      highBitMatch = GetBitHigh(reg4A,highBitOutput);
      firstLetter = switchboard4RegA[highBitOutput-1];
      secondLetter = switchboard4RegA[highBitMatch-1];
    }else if (GetBitHigh(reg4B))
    {
      highBitMatch = GetBitHigh(reg4B);
      firstLetter = switchboard4RegA[highBitOutput-1];
      secondLetter = switchboard4RegB[highBitMatch-1];
    }else{
      highBitMatch = 0;
    }

    if (highBitMatch)
    {
      switchboardA.push_back(firstLetter);
      switchboardB.push_back(secondLetter);
    }
  }

  //spi4 with register B changes output
  for (size_t i = 0; i < 8 ; i++)
  {
    changedOutput = rotl(changingOutput,i);
    //set only 1 as output
    WriteSpi(spi3,SWITCHBOARD1WRITEADDR,IODIRA,allInput,SWITCHBOARD1_CS);
    WriteSpi(spi3,SWITCHBOARD1WRITEADDR,IODIRB,allInput,SWITCHBOARD1_CS);// one after the other the pins become an output
    WriteSpi(spi4,SWITCHBOARD2WRITEADDR,IODIRA,allInput,SWITCHBOARD2_CS);
    WriteSpi(spi4,SWITCHBOARD2WRITEADDR,IODIRB,changedOutput,SWITCHBOARD2_CS);

    //read all the switchboard registers
    reg3A = ReadSpi(spi3,SWITCHBOARD1READADDR,GPIOA,SWITCHBOARD1_CS);
    reg3B = ReadSpi(spi3,SWITCHBOARD1READADDR,GPIOB,SWITCHBOARD1_CS);

    reg4A = ReadSpi(spi4,SWITCHBOARD2READADDR,GPIOA,SWITCHBOARD2_CS);
    reg4B = ReadSpi(spi4,SWITCHBOARD2READADDR,GPIOB,SWITCHBOARD2_CS);

    highBitOutput = GetBitHigh(changedOutput);//need to know which bit was set to high so it can be ignored when searching for the high input
    
   
    if(GetBitHigh(reg3A)){
      highBitMatch = GetBitHigh(reg3A);//look for high input but ignore the high output
      firstLetter = switchboard4RegB[highBitOutput-1];
      secondLetter = switchboard3RegA[highBitMatch-1];
    }else if (GetBitHigh(reg3B))
    {
      highBitMatch = GetBitHigh(reg3B);
      firstLetter = switchboard4RegB[highBitOutput-1];
      secondLetter = switchboard3RegB[highBitMatch-1];
    }else if (GetBitHigh(reg4A))
    {
      highBitMatch = GetBitHigh(reg4A);
      firstLetter = switchboard4RegB[highBitOutput-1];
      secondLetter = switchboard4RegA[highBitMatch-1];
    }else if (GetBitHigh(reg4B,highBitOutput))
    {
      highBitMatch = GetBitHigh(reg4B,highBitOutput);
      firstLetter = switchboard4RegB[highBitOutput-1];
      secondLetter = switchboard4RegB[highBitMatch-1];
    }else{
      highBitMatch = 0;
    }

    if (highBitMatch)
    {
      switchboardA.push_back(firstLetter);
      switchboardB.push_back(secondLetter);
    }
  }

}
  


void ReadRotary(int RegisterA,int NrA, int NrB, int motor,int GPIOAB){
  int AState = GetBit(RegisterA,1,NrA);
  int BState = GetBit(RegisterA,1,NrB);
  int ALastState = -1;
  int counter = 0;

  if (NrA == 0)
  {
    ALastState = A1LastState;
  }else if(NrA == 2){
    ALastState = A2LastState;
  }else if(NrA ==4){
    ALastState = A3LastState;
  }

  if (AState<ALastState){
      if(AState == 0 && BState == 1){
          counter ++;
          turnMotor(10,motor,GPIOAB);
      } else{
          counter --;
          turnMotor(-10,motor,GPIOAB);
      }
      ESP_LOGI(rotary, "counter %d ",counter);
      ESP_LOGI(rotary, "a: %d, b: %d",AState,BState);
      ESP_LOGI(rotary," ");
  }

  if (NrA == 0)
  {
    A1LastState = AState;
  }else if(NrA == 2){
    A2LastState = AState;
  }else if(NrA ==4){
    A3LastState = AState;
  }
  
}

//////////////////////////
///////MAIN/////////
/////////////////////////

extern "C"
{
	void app_main(void)
	{

    // uint8_t dataFromPoll;
    //de cs pin van de keyboard io expander instellen als output
    GpioSetup(KEYBOARD_CS,GPIO_MODE_OUTPUT,1,0,0);
    GpioSetup(STEPPER_CS,GPIO_MODE_OUTPUT,1,0,0);
    GpioSetup(SWITCHBOARD1_CS,GPIO_MODE_OUTPUT,1,0,0);
    GpioSetup(SWITCHBOARD2_CS,GPIO_MODE_OUTPUT,1,0,0);
    GpioSetup(ROTARY_CS,GPIO_MODE_OUTPUT,1,0,0);
    // configure_ledstrip();

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
        .clock_speed_hz=5*1000*1000, //10MHz
        .spics_io_num=KEYBOARD_CS,               //CS pin
        .queue_size=20                         //We want to be able to queue 7 transactions at a time
    };

    spi_device_interface_config_t devcfg2 = {
        .mode=0,                                //SPI mode 0
        .clock_speed_hz=5*1000*1000, //10MHz
        .spics_io_num=STEPPER_CS,               //CS pin
        .queue_size=7                         //We want to be able to queue 7 transactions at a time
    };

    spi_device_interface_config_t devcfg3 = {
        .mode=0,                                //SPI mode 0
        .clock_speed_hz=5*1000*1000, //10MHz
        .spics_io_num=SWITCHBOARD1_CS,               //CS pin
        .queue_size=7                         //We want to be able to queue 7 transactions at a time
    };
    spi_device_interface_config_t devcfg4 = {
        .mode=0,                                //SPI mode 0
        .clock_speed_hz=5*1000*1000, //10MHz
        .spics_io_num=SWITCHBOARD2_CS,               //CS pin
        .queue_size=7                         //We want to be able to queue 7 transactions at a time
    };

        spi_device_interface_config_t devcfg5 = {
        .mode=0,                                //SPI mode 0
        .clock_speed_hz=5*1000*1000, //10MHz
        .spics_io_num=ROTARY_CS,               //CS pin
        .queue_size=7                         //We want to be able to queue 7 transactions at a time
    };

    ret = spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);//this initializes the spi bus
    ESP_ERROR_CHECK(ret);//if this gives an error it wil show in the terminal
    ESP_LOGI(error, "error bus code %d!", ret);//this way I get extra confirmation if the error code is 0

    //start big loop
    while (true)
    {

      // ////////////////////////////////////////////////////////////////////////////////
      // //////////////////////////////////SETUP ENIGMA//////////////////////////////////
      // ////////////////////////////////////////////////////////////////////////////////
      // //spi 2,5 needed
      // ret=spi_bus_add_device(HSPI_HOST, &devcfg2, &spi2);//this adds io expander 2 to the bus 
      // ESP_ERROR_CHECK(ret);
      // ESP_LOGI(SPI, "added spi2");

      // ret=spi_bus_add_device(HSPI_HOST, &devcfg5, &spi5);//this adds io expander 5 to the bus
      // ESP_ERROR_CHECK(ret);
      // ESP_LOGI(SPI, "added spi5");

      // //stepper motor io expander as outputs
      // WriteSpi(spi2,STEPPERWRITEADDR,IODIRA,0b00000000,STEPPER_CS);
      // WriteSpi(spi2,STEPPERWRITEADDR,IODIRB,0b00000000,STEPPER_CS);
      // //rotary encoder io expander as inputs
      // WriteSpi(spi5,ROTARYWRITEADDR,IODIRA,0b11111111,ROTARY_CS);
      // WriteSpi(spi5,ROTARYWRITEADDR,IODIRB,0b11111111,ROTARY_CS);

      // GpioRegister5A = ReadSpi(spi5,ROTARYREADADDR,GPIOA,ROTARY_CS);

      // A1LastState = GetBit(GpioRegister5A,1,0);
      // A2LastState = GetBit(GpioRegister5A,1,2);
      // A3LastState = GetBit(GpioRegister5A,1,4);

      
      // while (1)//condition will be while start message is not pushed
      // {
      //   //checks the buttons pushed
      //   GpioRegister5B = ReadSpi(spi5,ROTARYREADADDR,GPIOB,ROTARY_CS);
      //   if (GetBit(GpioRegister5B,1,0))
      //   {
      //     //rotary 1 pushed
      //     if (choice_rotor_left != 5)
      //     {
      //       choice_rotor_left +=1;
      //     }else{
      //       choice_rotor_left = 1;
      //     }
      //     set_multiple_leds(choice_rotor_left,choice_rotor_mid,choice_rotor_right);
      //   } else if(GetBit(GpioRegister5B,1,1)){
      //     //rotary 2 pushed
      //     if (choice_rotor_mid != 5)
      //     {
      //       choice_rotor_mid +=1;
      //     }else{
      //       choice_rotor_mid = 1;
      //     }
      //     set_multiple_leds(choice_rotor_left,choice_rotor_mid,choice_rotor_right);
      //   }else if (GetBit(GpioRegister5B,1,2)){
      //     //rotary 3 pushed
      //     if (choice_rotor_right != 5)
      //     {
      //       choice_rotor_right +=1;
      //     }else{
      //       choice_rotor_right = 1;
      //     }
      //     set_multiple_leds(choice_rotor_left,choice_rotor_mid,choice_rotor_right);
      //   }

      //   //checks the turning of the rotary encoder and turns the stepper motor accordingly
      //   GpioRegister5A = ReadSpi(spi5,ROTARYREADADDR,GPIOA,ROTARY_CS);

      //   ReadRotary(GpioRegister5A,0,1,1,GPIOB);//this reads the first rotary encoder and turns the stepper motor if neccesary
      //   ReadRotary(GpioRegister5A,2,3,2,GPIOB);//this reads the second rotary encoder and turns the stepper motor if neccesary
      //   ReadRotary(GpioRegister5A,4,5,3,GPIOA);//this reads the third rotary encoder and turns the stepper motor if neccesary

      // }

      

      // ////////////////////////////////////////////////////////////////////////////////
      // ////////////////////////////////SWITCHBOARD READ////////////////////////////////
      // ////////////////////////////////////////////////////////////////////////////////
      // //spi 3,4 needed
      // ret=spi_bus_remove_device(spi2);//this frees space on the bus else ESP_ERR_NO_MEM
      // ESP_ERROR_CHECK(ret);
      // ESP_LOGI(SPI, "removed spi2");

      // ret=spi_bus_remove_device(spi5);//this frees space on the bus else ESP_ERR_NO_MEM
      // ESP_ERROR_CHECK(ret);
      // ESP_LOGI(SPI, "removed spi5");

      // ret=spi_bus_add_device(HSPI_HOST, &devcfg3, &spi3);//this adds io expander 3 to the bus 
      // ESP_ERROR_CHECK(ret);
      // ESP_LOGI(SPI, "added spi3");

      // WriteSpi(spi3,SWITCHBOARD1WRITEADDR,IODIRA,0b11111111,SWITCHBOARD1_CS);
      // while (1)
      // {
      //   printRegister(ReadSpi(spi3,SWITCHBOARD1READADDR,GPIOA,SWITCHBOARD1_CS));
      //   vTaskDelay(2000/portTICK_PERIOD_MS);
      // }
      

      // ret=spi_bus_add_device(HSPI_HOST, &devcfg4, &spi4);//this adds io expander 4 to the bus
      // ESP_ERROR_CHECK(ret);
      // ESP_LOGI(SPI, "added spi4");

      // //the switchboard is only read when the start message button is pressed
      // //IODIR happens in fuction because it changes often
      // readSwitchboard();

      ////////////////////////////////////////////////////////////////////////////////
      ///////////////////////////////////ENIGMA USE///////////////////////////////////
      ////////////////////////////////////////////////////////////////////////////////
      
      // ret=spi_bus_remove_device(spi3);//this frees space on the bus else ESP_ERR_NO_MEM
      // ESP_ERROR_CHECK(ret);
      // ESP_LOGI(SPI, "removed spi2");

      // ret=spi_bus_remove_device(spi4);//this frees space on the bus else ESP_ERR_NO_MEM
      // ESP_ERROR_CHECK(ret);
      // ESP_LOGI(SPI, "removed spi4");

      ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);//this adds io expander 1 to the bus 
      ESP_ERROR_CHECK(ret);
      ESP_LOGI(SPI, "added spi");

      ret=spi_bus_add_device(HSPI_HOST, &devcfg2, &spi2);//this adds io expander 2 to the bus
      ESP_ERROR_CHECK(ret);
      ESP_LOGI(SPI, "added spi2");

      //keyboard io expander has its inputs at A and outputs at B
      WriteSpi(spi,KEYBOARDWRITEADDR,IODIRA,0b11111111,KEYBOARD_CS);
      WriteSpi(spi,KEYBOARDWRITEADDR,IODIRB,0b00000000,KEYBOARD_CS);
      //stepper motor io expander is all output
      WriteSpi(spi2,STEPPERWRITEADDR,IODIRA,0b00000000,STEPPER_CS);
      WriteSpi(spi2,STEPPERWRITEADDR,IODIRB,0b00000000,STEPPER_CS);


      ESP_LOGI(keyboardLog, "started reading");

      while (1)// condition is as long as the message end button isn't pressed
      {
        //Set a row high en read the collumn. the combination of row and collumn tells if button is pressend and which one
        setButtonOutput(0b00000001);
        getLetterFromInputs(readButtonInputs(),0);
        setButtonOutput(0b00000010);
        getLetterFromInputs(readButtonInputs(),1);
        setButtonOutput(0b00000100);
        getLetterFromInputs(readButtonInputs(),2);
        setButtonOutput(0b00001000);
        getLetterFromInputs(readButtonInputs(),3);
        //is nu voor 3x3 button matrix, moet nog vergroot worden, de letter moet nog vertaald worden en de led moet nog aan gaan
        int read_keyboard = ReadSpi(spi,KEYBOARDREADADDR,GPIOA,KEYBOARD_CS);
        // printRegister(read_keyboard);
        // vTaskDelay(20/portTICK_PERIOD_MS);
  

      }
      ESP_LOGI(keyboardLog, "ended reading");
      

      ////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////SEND MESSAGE//////////////////////////////////
      ////////////////////////////////////////////////////////////////////////////////
      ret=spi_bus_remove_device(spi);//this frees space on the bus else ESP_ERR_NO_MEM
      ESP_ERROR_CHECK(ret);
      ESP_LOGI(SPI, "removed spi");

      ret=spi_bus_remove_device(spi2);//this frees space on the bus else ESP_ERR_NO_MEM
      ESP_ERROR_CHECK(ret);
      ESP_LOGI(SPI, "removed spi2");

    }
    //end of the endless loop of the program

    //this delay shouldn't show
    ESP_LOGI(LEDSTRIP, "start delay");
    vTaskDelay(500000/portTICK_PERIOD_MS);
    ESP_LOGI(LEDSTRIP, "stop delay");

	}
}