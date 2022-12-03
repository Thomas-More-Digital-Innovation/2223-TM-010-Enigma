#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "driver/spi_master.h"
#include <string.h>

unsigned short int choice_rotor_left = 1;
unsigned short int choice_rotor_mid = 2;
unsigned short int choice_rotor_right = 3;

esp_err_t ret;
spi_device_handle_t spi;
spi_device_handle_t spi2;
spi_device_handle_t spi3;
spi_device_handle_t spi4;
spi_device_handle_t spi5;

static const char *error = "error";
static const char *TAG = "test";
static const char *SPI = "test";

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

//switchboard
static const char *switchboard = "switchboard";
#define SWITCHBOARD1_CS GPIO_NUM_15
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

static const char *ROTARY = "rotary";
#define ROTARY_CS GPIO_NUM_12

#define ROTARYWRITEADDR 0x48
#define ROTARYREADADDR 0X49

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
  
  return gpioa_stateOriginal;//when the button is released return what was in the register
}

void setButtonOutput(int binairyNumber){
  WriteSpi(spi,KEYBOARDWRITEADDR,GPIOB,binairyNumber,KEYBOARD_CS);  
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
  ledstrip->set_pixel(ledstrip, ledNumber, 0, 255, 0);
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
//this function rotates a byte to the left by 1
unsigned char rotl(unsigned char c)//https://stackoverflow.com/questions/19204750/how-do-i-perform-a-circular-rotation-of-a-byte
{
    return (c << 1) | (c >> 7);
}

void readSwitchboard(){
  int directionIO = 0b00000000;
  int outputIO = 0b00000001;
  WriteSpi(spi3,SWITCHBOARD1WRITEADDR,IODIRB,directionIO,SWITCHBOARD1_CS);//dit is een probleem maar er zij nog 6 pinnen over dus kan als de onderste 2 niet gebruikt worden
  WriteSpi(spi3,SWITCHBOARD2WRITEADDR,IODIRA,0b11111111,SWITCHBOARD2_CS);

  WriteSpi(spi3,SWITCHBOARD1WRITEADDR,GPIOB,outputIO,SWITCHBOARD1_CS);

  for (;;)
  {
    read_a = ReadSpi(spi4,SWITCHBOARD2READADDR,GPIOA,SWITCHBOARD2_CS);//hiermee gekoppeld => 1bit ernaast (verwacht 64 krijg 32, verwacht 4 krijg 2,...)
    ESP_LOGI(switchboard,"%d",read_a);
    // read_b = ReadSpi(spi3,SWITCHBOARD1READADDR,GPIOB,SWITCHBOARD1_CS);//hier hetzelfde //in arduino code is het wel juist
    // ESP_LOGI(switchboard,"%d",read_b);
    vTaskDelay(2000/portTICK_PERIOD_MS);
  }

}
//https://www.geeksforgeeks.org/extract-k-bits-given-position-number/
int GetBit(int number, int amountBits, int startingPos)
{
    return (((1 << amountBits) - 1) & (number >> (startingPos - 1)));
}
void printRegister(int status){
  // aState = ReadSpi(spi4,SWITCHBOARD2READADDR,GPIOA,SWITCHBOARD2_CS);
  ESP_LOGI(rotary, "VOLLEDIG: %d!", status);
  ESP_LOGI(rotary, "1: %d!", GetBit(status,1,0));
  ESP_LOGI(rotary, "2: %d!", GetBit(status,1,1));
  ESP_LOGI(rotary, "3: %d!", GetBit(status,1,2));
  ESP_LOGI(rotary, "4: %d!", GetBit(status,1,3));
  ESP_LOGI(rotary, "5: %d!", GetBit(status,1,4));
  ESP_LOGI(rotary, "6: %d!", GetBit(status,1,5));
  ESP_LOGI(rotary, "7: %d!", GetBit(status,1,6));
  ESP_LOGI(rotary, "8: %d!", GetBit(status,1,7));
}

void ReadRotary(int RegisterA,int NrA, int NrB){
  int AState = GetBit(RegisterA,1,NrA);
  int BState = GetBit(RegisterA,1,NrB);
  int ALastState;

  if (NrA == 0)
  {
    ALastState = A1LastState
  }else if(NrA == 2){
    ALastState = A2LastState
  }else if(NrA ==4){
    ALastState = A3LastState
  }

  if (AState<ALastState){
      if(AState == 0 && BState == 1){
          counter ++;//this should be the stepper motor turning
      } else{
          counter --;
      }
      ESP_LOGI(ROTARY, "counter %d ",counter);
      ESP_LOGI(ROTARY, "a: %d, b: %d",AState,BState);
      ESP_LOGI(ROTARY," ");
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


// void readRotary(){

//   GpioRegisterA = ReadSpi(spi5,ROTARYREADADDR,GPIOA,ROTARY_CS);
//   a1State = GetBit(GpioRegisterA,1,0);
//   b1State = GetBit(GpioRegisterA,1,1);

//   if (aState<aLastState){
//       if(aState == 0 && bState == 1){
//           counter ++;
//       } else{
//           counter --;
//       }
//       ESP_LOGI(TAG, "counter %d ",counter);
//       ESP_LOGI(TAG, "a: %d, b: %d",aState,bState);
//       ESP_LOGI(TAG," ");
//   }
//   aLastState = aState;
//   vTaskDelay(10/portTICK_PERIOD_MS);
// }


//////////////////////////
///////MAIN/////////
/////////////////////////


extern "C"
{
	void app_main(void)
	{

    // uint8_t dataFromPoll;
    //de cs pin van de keyboard io expander instellen als output
    gpio_setup(KEYBOARD_CS,GPIO_MODE_OUTPUT,1,0,0);
    gpio_setup(STEPPER_CS,GPIO_MODE_OUTPUT,1,0,0);
    gpio_setup(SWITCHBOARD1_CS,GPIO_MODE_OUTPUT,1,0,0);
    gpio_setup(SWITCHBOARD2_CS,GPIO_MODE_OUTPUT,1,0,0);
    configure_ledstrip();
    set_ledstrip(8);//makes the first led burn

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
        .clock_speed_hz=10*1000*1000, //10MHz
        .spics_io_num=KEYBOARD_CS,               //CS pin
        .queue_size=20                         //We want to be able to queue 7 transactions at a time
    };

    spi_device_interface_config_t devcfg2 = {
        .mode=0,                                //SPI mode 0
        .clock_speed_hz=10*1000*1000, //10MHz
        .spics_io_num=STEPPER_CS,               //CS pin
        .queue_size=7                         //We want to be able to queue 7 transactions at a time
    };

    spi_device_interface_config_t devcfg3 = {
        .mode=0,                                //SPI mode 0
        .clock_speed_hz=10*1000*1000, //10MHz
        .spics_io_num=SWITCHBOARD1_CS,               //CS pin
        .queue_size=7                         //We want to be able to queue 7 transactions at a time
    };
    spi_device_interface_config_t devcfg4 = {
        .mode=0,                                //SPI mode 0
        .clock_speed_hz=10*1000*1000, //10MHz
        .spics_io_num=SWITCHBOARD2_CS,               //CS pin
        .queue_size=7                         //We want to be able to queue 7 transactions at a time
    };

        spi_device_interface_config_t devcfg5 = {
        .mode=0,                                //SPI mode 0
        .clock_speed_hz=10*1000*1000, //10MHz
        .spics_io_num=ROTARY_CS,               //CS pin
        .queue_size=7                         //We want to be able to queue 7 transactions at a time
    };

    ret = spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);//this initializes the spi bus
    ESP_ERROR_CHECK(ret);//if this gives an error it wil show in the terminal
    ESP_LOGI(error, "error bus code %d!", ret);//this way I get extra confirmation if the error code is 0

    //start big loop
    while (True)
    {
      ////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////SETUP ENIGMA//////////////////////////////////
      ////////////////////////////////////////////////////////////////////////////////
      //spi 2,5 needed
      ret=spi_bus_add_device(HSPI_HOST, &devcfg2, &spi2);//this adds io expander 2 to the bus 
      ESP_ERROR_CHECK(ret);
      ESP_LOGI(SPI, "added spi2");

      ret=spi_bus_add_device(HSPI_HOST, &devcfg5, &spi5);//this adds io expander 5 to the bus
      ESP_ERROR_CHECK(ret);
      ESP_LOGI(SPI, "added spi5");

      //stepper motor io expander as outputs
      WriteSpi(spi2,STEPPERWRITEADDR,IODIRA,0b00000000,STEPPER_CS);
      WriteSpi(spi2,STEPPERWRITEADDR,IODIRB,0b00000000,STEPPER_CS);
      //rotary encoder io expander as inputs
      WriteSpi(spi5,ROTARYWRITEADDR,IODIRA,0b11111111,ROTARY_CS);
      WriteSpi(spi5,ROTARYWRITEADDR,IODIRB,0b11111111,ROTARY_CS);

      //this happens until the start message button is pressed
      GpioRegister5A = ReadSpi(spi5,ROTARYREADADDR,GPIOA,ROTARY_CS);

      A1LastState = GetBit(aLastState,1,0);
      A2LastState = GetBit(aLastState,1,2);
      A3LastState = GetBit(aLastState,1,4);
      ReadRotary(GpioRegister5A,0,1);//this reads the first rotary encoder and turns the stepper motor if neccesary
      ReadRotary(GpioRegister5A,2,3);//this reads the second rotary encoder and turns the stepper motor if neccesary
      ReadRotary(GpioRegister5A,4,5);//this reads the third rotary encoder and turns the stepper motor if neccesary

      GpioRegister5B = ReadSpi(spi5,ROTARYREADADDR,GPIOB,ROTARY_CS);
      if (GetBit(GpioRegister5B,1,0))
      {
        //rotary 1 pushed
        if (choice_rotor_left != 5)
        {
          choice_rotor_left +=1;
        }else{
          choice_rotor_left = 1;
        }
      } else if(GetBit(GpioRegister5B,1,1)){
        //rotary 2 pushed
        if (choice_rotor_mid != 5)
        {
          choice_rotor_mid +=1;
        }else{
          choice_rotor_mid = 1;
        }
      }else if (GetBit(GpioRegister5B,1,2)){
        //rotary 3 pushed
        if (choice_rotor_right != 5)
        {
          choice_rotor_right +=1;
        }else{
          choice_rotor_right = 1;
        }
      }
      
      



      ////////////////////////////////////////////////////////////////////////////////
      ////////////////////////////////SWITCHBOARD READ////////////////////////////////
      ////////////////////////////////////////////////////////////////////////////////
      //spi 3,4 needed
      ret=spi_bus_remove_device(spi2);//this frees space on the bus else ESP_ERR_NO_MEM
      ESP_ERROR_CHECK(ret);
      ESP_LOGI(SPI, "removed spi2");

      ret=spi_bus_remove_device(spi5);//this frees space on the bus else ESP_ERR_NO_MEM
      ESP_ERROR_CHECK(ret);
      ESP_LOGI(SPI, "removed spi5");

      ret=spi_bus_add_device(HSPI_HOST, &devcfg3, &spi3);//this adds io expander 3 to the bus 
      ESP_ERROR_CHECK(ret);
      ESP_LOGI(SPI, "added spi3");

      ret=spi_bus_add_device(HSPI_HOST, &devcfg4, &spi4);//this adds io expander 4 to the bus
      ESP_ERROR_CHECK(ret);
      ESP_LOGI(SPI, "added spi4");

      //IODIR happens in fuction because it changes often


      //the switchboard is only read when the start message button is pressed


      ////////////////////////////////////////////////////////////////////////////////
      ///////////////////////////////////ENIGMA USE///////////////////////////////////
      ////////////////////////////////////////////////////////////////////////////////
      
      ret=spi_bus_remove_device(spi3);//this frees space on the bus else ESP_ERR_NO_MEM
      ESP_ERROR_CHECK(ret);
      ESP_LOGI(SPI, "removed spi2");

      ret=spi_bus_remove_device(spi4);//this frees space on the bus else ESP_ERR_NO_MEM
      ESP_ERROR_CHECK(ret);
      ESP_LOGI(SPI, "removed spi4");

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


      // this happens as long as the message end button isn't pressed

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
