#include <Arduino.h>
#include <SPI.h>
#include <vector>
#include <FastLED.h>
#include "WiFi.h"
#include <PubSubClient.h>
#include <HTTPClient.h>//for the post
#include <ArduinoJson.h>//for the post
#include <UrlEncode.h>

using namespace std;

class Rotor{
  public:
    int currentPosition;
    int positionTurnRotorToLeft;
    int rotorChoice;
    int previousPositionSetup;
    vector<unsigned short int> vectorRotorA;
    vector<unsigned short int> vectorRotorB;
    vector<unsigned short int> vectorRotorC;

    //constructor
    Rotor(int currentPosition,int positionTurnRotorToLeft, int rotorChoice,int previousPositionSetup,vector<unsigned short int> vectorRotorA,vector<unsigned short int> vectorRotorB,vector<unsigned short int> vectorRotorC){
      this->currentPosition = currentPosition;
      this->positionTurnRotorToLeft = positionTurnRotorToLeft;
      this->rotorChoice = rotorChoice;
      this->vectorRotorA = vectorRotorA;
      this->vectorRotorB = vectorRotorB;
      this->vectorRotorC = vectorRotorC;
      this->previousPositionSetup = previousPositionSetup;
    }
    
    //methods
    int buttonPushed(){
      
      rotorChoice +=1;
      if (rotorChoice>5){
        rotorChoice = 1;
      }
      Serial.println(rotorChoice);

      return rotorChoice;
    }
};

//wifi
const char* ssid = "embedded";
const char* password =  "IoTembedded";
//mqtt
const char* mqtt_server = "broker.hivemq.com";
WiFiClient espWifiClient;
PubSubClient client(espWifiClient);
//api
DynamicJsonDocument doc(1024);//for the post response

String originalUrl = "https://2223-tm-010-enigma.pages.dev/api/";
String xApiKey = "EnigmaAPITokenTest";
String textToPost = "";

String phoneNumber = "+32479731154";
String apiKeyWhatsapp = "616691";

//ledstrip
#define NUM_LEDS 26
#define LED_PIN 21
CRGB leds[NUM_LEDS];

vector<char> letterNumber({'a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z'});
vector<unsigned short int> rotor1a({ 1,2, 3, 4,5,6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26});
vector<unsigned short int> rotor1b({16,5,12,26,7,1,11,15,24,22, 2,17,14, 8,19,20,18,21, 3,23,25,10, 4, 6,13,9});
vector<unsigned short int> rotor1c({6,11,19,23,2,24,5,14,26,22,7,3,25,13,8,1,12,17,15,16,18,10,20,9,21,4});

vector<unsigned short int> rotor2a({1, 2, 3, 4, 5, 6, 7, 8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26});
vector<unsigned short int> rotor2b({10,25,22,13,12,16,23,2,1,8, 11,15,26,17,18,21, 4,20,3, 24, 5, 6, 7,9, 14,19});
vector<unsigned short int> rotor2c({9,8,19,17,21,22,23,10,24,1,11,5,4,25,12,6,14,15,26,18,16,3,7,20,2,13});

vector<unsigned short int> rotor3a({1,2, 3, 4,5, 6, 7, 8, 9, 10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26});
vector<unsigned short int> rotor3b({1,13,16,6,15,14,12,18,23,25,3, 17,7, 21,10,11,19,24,8, 2, 20,4, 5, 22,26,9});
vector<unsigned short int> rotor3c({1,20,11,22,23,4,13,19,26,15,16,7,2,6,5,3,12,8,17,21,14,24,9,18,10,25});

vector<unsigned short int> rotor4a({1,2,3,4, 5, 6, 7,8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26});
vector<unsigned short int> rotor4b({9,6,2,8,13,20,19,3,11,22,16,15,14,23,25,17,26, 1,12,21,10, 4,24, 5,18, 7});
vector<unsigned short int> rotor4c({18,3,8,22,24,2,26,4,1,21,9,19,5,13,12,11,16,25,7,6,20,10,14,23,15,17});

vector<unsigned short int> rotor5a({1, 2, 3, 4, 5, 6, 7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26});
vector<unsigned short int> rotor5b({1,11,18,12,22,19,21,2,8,10, 5,15,14,16,20,25,23, 3, 4,24, 9,17, 7,26,13, 6});
vector<unsigned short int> rotor5c({1,8,18,19,11,26,23,9,21,10,2,4,25,13,12,14,22,3,6,15,7,5,17,20,16,24});

vector<unsigned short int> reflectora({8, 5, 6, 16,7, 9, 2,26,10,14,25,12,15});
vector<unsigned short int> reflectorb({20,21,11,1, 13,24,3,4, 22,18,23,17,19});



vector<vector<unsigned short int>> possibleRotors{rotor1a,rotor1b,rotor1c,rotor2a,rotor2b,rotor2c,rotor3a,rotor3b,rotor3c,rotor4a,rotor4b,rotor4c,rotor5a,rotor5b,rotor5c};

vector<unsigned short int> ledSequence({16,25,24,3,22,2,14,13,12,11,10,8,7,6,4,19,1,17,23,5,18,20,26,21,9,15});

vector<unsigned short int> switchboardNumberChange({17,23,5,1,19,4,16,25,24,18,20,26,6,7,8,3,22,2,21,9,15,10,11,13,14,12});

vector<unsigned short int> keyboardSequence({16,25,24,3,22,2,14,13,12,1,19,4,6,7,8,10,11,17,23,5,18,20,26,21,9,15});
vector<unsigned short int> keyboardTopLayer({17,23,5,18,20,26,21,9,15});
vector<unsigned short int> keyboardMidLayer({1,19,4,6,7,8,10,11});
vector<unsigned short int> keyboardBottomLayer({16,25,24,3,22,2,14,13,12});

vector<unsigned short int> acceptibleKeyboardPresses({1,2,4,8,16,32,64});


//for every io expander
#define IODIRA 0x00
#define IODIRB 0x01
#define GPIOA 0x12
#define GPIOB 0x13
//only for keyboard expander
#define KEYBOARD_CS 14
#define KEYBOARDWRITEADDR 0x40
#define KEYBOARDREADADDR 0x41
unsigned char  gpioa_state = 0; 
unsigned char  gpioa_stateOriginal = 0;
int global_state = 0;

// char keyboardPoints[4][7] = {{'q','w','e','r','t','z','u'}, {'a','s','d','f','g','h','j'},{'p','y','x','c','v','b','n'},{'i','o','k','m','l','1','2'}};//'1' en '2' is enkel om [4][7] te doen kloppen.
char keyboardPoints[4][7] = {{'o','i','u','z','t','r','e'}, {'k','j','h','g','f','d','s'},{'l','m','n','b','v','c','x'},{'w','q','a','y','p','1','2'}};
int keyboardPointsInt[4][7] = {{15,9,21,26,20,18,5}, {11,10,8,7,6,4,19},{12,13,14,2,22,3,24},{23,17,1,25,16,30,30}};
//stepper motor
#define STEPPER_CS 32
#define STEPPERWRITEADDR 0x42
#define STEPPERREADADDR 0x43

//switchboard
#define SWITCHBOARD1_CS 12
#define SWITCHBOARD2_CS 27

#define SWITCHBOARD1WRITEADDR 0x44
#define SWITCHBOARD1READADDR 0X45
#define SWITCHBOARD2WRITEADDR 0X46
#define SWITCHBOARD2READADDR 0X47

int stepSizeLetter = 5;

//regpresents what letters are attatched to te switchboard io expander registers
int switchboard3RegA[8] = {1,2,3,4,5,6,7,8};
int switchboard3RegB[8] = {9,10,11,12,13,14,15,16};
int switchboard4RegA[8] = {17,18,19,20,21,22,23,24};
int switchboard4RegB[2] = {25,26};

std::vector<unsigned short int> switchboardA = {};
std::vector<unsigned short int> switchboardB = {};

unsigned char  read_a = 0; 
unsigned char  read_b = 0;

//rotary encoders
int GpioRegister5A;
int GpioRegister5B;
int A1LastState;  
int A2LastState;  
int A3LastState;  
int rotary1Counter;
int rotary2Counter;
int rotary3Counter;

#define ROTARY_CS 15

#define ROTARYWRITEADDR 0x48
#define ROTARYREADADDR 0X49

// int counter = 0; 
int aState;
int ALastState; 

//booleans for stop and start button
bool pressedStopButton = false;
bool pressedStartButton = false;

Rotor rotorLeft(1,7,1,1,possibleRotors[0],possibleRotors[1],rotor1c);
Rotor rotorMid(1,8,2,1,possibleRotors[2],possibleRotors[3],rotor2c);
Rotor rotorRight(1,8,3,1,possibleRotors[4],possibleRotors[5],rotor3c);

int actualRotor;
int rotaryPushed = false;
bool alreadyTurnedLeft = false;
//////////////////////////
///////FUNCTIONS/////////
/////////////////////////

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

int invertBits(int num)
{
    for (int i = 0; i < 8; i++)
       num = (num ^ (1 << i));
  
    return num;
}

unsigned short int get_index(vector<unsigned short int> vector, unsigned short int letter)
{
    auto position = find(vector.begin(), vector.end(), letter);//met find wordt er niet van 0 geteld

    if (position != vector.end()) 
    {
        int index = position - vector.begin();//dit maakt het de index, vector eerste element is index 0
        return index;
    }
    else {
        return 30;//30 is te veel en betekend dat de letter niet gevonden werd in de vector
    }
}




//led functions
void showPositionLed(int position){
  FastLED.clear();
  int index = get_index(ledSequence,position);
  if (index>=0 && index<=26)
  {
    leds[index] = CRGB::White;
  }else{
    Serial.println("index of led not in between 1 and 26 index:"+ String(index)+" position:"+String(position));
  }
  FastLED.show();
}

void showLedsRotorChoice(int choiceLeft, int choiceMid, int choiceRight){
  
  FastLED.clear();

  leds[choiceLeft+16] = CRGB::White;
  leds[-choiceMid+17] = CRGB::White;
  leds[choiceRight-1] = CRGB::White;

  FastLED.show();

}

void ledsMoving(){
  for (size_t i = 0; i < 26; i++)
  {
    FastLED.clear();
    leds[i] = CRGB::White;
    FastLED.show();
    vTaskDelay(50/portTICK_PERIOD_MS);
  }
  FastLED.clear(true); 
}
void rotaryLeds(){
  FastLED.clear();
  leds[20] = CRGB::White;
  FastLED.show();
  vTaskDelay(500/portTICK_PERIOD_MS);
  FastLED.clear();
  leds[25] = CRGB::White;
  FastLED.show();
  vTaskDelay(500/portTICK_PERIOD_MS);
  FastLED.clear();
  leds[21] = CRGB::White;
  FastLED.show();
  vTaskDelay(500/portTICK_PERIOD_MS);
  FastLED.clear();
  FastLED.show();
}

void wifiLeds(){
  FastLED.clear();
  leds[18] = CRGB::White;
  FastLED.show();
  vTaskDelay(500/portTICK_PERIOD_MS);
  FastLED.clear();
  leds[24] = CRGB::White;
  FastLED.show();
  vTaskDelay(500/portTICK_PERIOD_MS);
  FastLED.clear();
  leds[13] = CRGB::White;
  FastLED.show();
  vTaskDelay(500/portTICK_PERIOD_MS);
  FastLED.clear();
  leds[24] = CRGB::White;
  FastLED.show();
  vTaskDelay(800/portTICK_PERIOD_MS);
}

void keyboardProblemLed(){
  FastLED.clear();
  leds[9] = CRGB::Red;
  FastLED.show();
  vTaskDelay(500/portTICK_PERIOD_MS);
  FastLED.clear();
  leds[19] = CRGB::Red;
  FastLED.show();
  vTaskDelay(500/portTICK_PERIOD_MS);
  FastLED.clear();
  leds[1] = CRGB::Red;
  FastLED.show();
  vTaskDelay(800/portTICK_PERIOD_MS);
}

void printRegister(int status){
  Serial.println("1: "+ String(GetBit(status,1,1)));
  Serial.println("2: "+ String(GetBit(status,1,2)));
  Serial.println("3: "+ String(GetBit(status,1,3)));
  Serial.println("4: "+ String(GetBit(status,1,4)));
  Serial.println("5: "+ String(GetBit(status,1,5)));
  Serial.println("6: "+ String(GetBit(status,1,6)));
  Serial.println("7: "+ String(GetBit(status,1,7)));
  Serial.println("8: "+ String(GetBit(status,1,8)));
}

//functions to write and read via SPI
void WriteSpi(int addresExpander,int addresRegister,int cmd,int cs_pin){
  digitalWrite(cs_pin, LOW); //zet CS low to start sending data
  SPI.transfer(addresExpander);//address of the ioexpander
  SPI.transfer(addresRegister);//number of the register
  SPI.transfer(cmd);//this command will be put in the register
  digitalWrite(cs_pin, HIGH);
}
int ReadSpi(int addresExpander,int addresRegister,int cs_pin){
  digitalWrite(cs_pin, LOW);
  SPI.transfer(addresExpander);
  SPI.transfer(addresRegister);
  int returnFromRegister = SPI.transfer(addresRegister); 
  digitalWrite(cs_pin, HIGH);
  return returnFromRegister;
}


void wifiConnect(){
  //wifi switch is on this register
  WriteSpi(ROTARYWRITEADDR,IODIRB,0b11111111,ROTARY_CS);
  //check if wifi switch is on
  if(GetBit(ReadSpi(ROTARYREADADDR,GPIOB,ROTARY_CS),1,5)){
    //make wifi connection
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED && GetBit(ReadSpi(ROTARYREADADDR,GPIOB,ROTARY_CS),1,5)) { 
    vTaskDelay(500/portTICK_PERIOD_MS);   
    Serial.println("Connecting to WiFi..");
    wifiLeds();
    }
    Serial.println("Connected to the WiFi network");
  }else{
    Serial.println("not connecting because switch is off");
  }
  
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("trying to connect to mqtt broker");
    
    if (client.connect("esp32EnigmaHanne")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      vTaskDelay(1000/portTICK_PERIOD_MS);
    }
  }
}

//https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/
void mqttMessage(String message){
  if(WiFi.status()==WL_CONNECTED){
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
    if (!client.connected()) {
    reconnect();
    }
    client.loop();

    int lengthWord = message.length();
    char messageCharArray[lengthWord];
    for (size_t i = 0; i < lengthWord; i++)
    {
      messageCharArray[i] = message[i];
    }
    Serial.println(messageCharArray);
    client.publish("topic/enigmaThomasMoreHanne", messageCharArray);
    Serial.println("published MQTT");
  }
}

void postMessage(){
  String fullUrl = originalUrl+textToPost;
  HTTPClient http;   
    
  http.begin(fullUrl);  
  //set the headers
  http.addHeader("Content-Type", "application/json");             
  http.addHeader("x-api-key", xApiKey);
  
  int httpResponseCode = http.POST("post from enigma");
  WiFi.disconnect();
}
//https://randomnerdtutorials.com/esp32-send-messages-whatsapp/
void sendWhatsapp(String message){
  String url = "https://api.callmebot.com/whatsapp.php?phone=" + phoneNumber + "&apikey=" + apiKeyWhatsapp + "&text=" + urlEncode(message);
  HTTPClient http;  
  http.begin(url);

  // Specify content-type header
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  
  // Send HTTP POST request
  int httpResponseCode = http.POST(url);
  if (httpResponseCode == 200){
    Serial.print("Message sent successfully");
  }
  else{
    Serial.println("Error sending the message");
    Serial.print("HTTP response code: ");
    Serial.println(httpResponseCode);
  }

  // Free resources
  http.end();

}

void turnMotorBack(int steps, int motor, int GPIOAB, int wirteAddres, int cs){
  int step1 = (motor==1 || motor==3) ? 0b00000011 : 0b00000011<<4;
  int step2 = (motor==1 || motor==3) ? 0b00000110 : 0b00000110<<4;
  int step3 = (motor==1 || motor==3) ? 0b00001100 : 0b00001100<<4;
  int step4 = (motor==1 || motor==3) ? 0b00001001 : 0b00001001<<4;

  for (int i = 0; i > steps*stepSizeLetter; i--)
    { 
      WriteSpi(wirteAddres,GPIOAB,step1,cs);
      vTaskDelay(10/portTICK_PERIOD_MS);
      WriteSpi(wirteAddres,GPIOAB,step2,cs);
      vTaskDelay(10/portTICK_PERIOD_MS);
      WriteSpi(wirteAddres,GPIOAB,step3,cs);
      vTaskDelay(10/portTICK_PERIOD_MS);
      WriteSpi(wirteAddres,GPIOAB,step4,cs);
      vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

void turnMotor2(){
  WriteSpi(SWITCHBOARD2WRITEADDR,IODIRB,0b00000000,SWITCHBOARD2_CS);
  int limiterSwitch = GetBit(ReadSpi(STEPPERREADADDR,GPIOB,STEPPER_CS),1,2+4);
  for (int j = 0; j < 8000*stepSizeLetter; j++)
    {
      if (limiterSwitch)
      {
        turnMotorBack(-4,2,GPIOB,SWITCHBOARD2WRITEADDR,SWITCHBOARD2_CS);
        Serial.println("turningBack");
        break;
      }
      Serial.println("step1");
      WriteSpi(SWITCHBOARD2WRITEADDR,GPIOB,0b10010000,SWITCHBOARD2_CS);
      vTaskDelay(10/portTICK_PERIOD_MS);
      Serial.println("step2");
      WriteSpi(SWITCHBOARD2WRITEADDR,GPIOB,0b11000000,SWITCHBOARD2_CS);
      vTaskDelay(10/portTICK_PERIOD_MS);
      Serial.println("step3");
      WriteSpi(SWITCHBOARD2WRITEADDR,GPIOB,0b01100000,SWITCHBOARD2_CS);
      vTaskDelay(10/portTICK_PERIOD_MS);
      Serial.println("step4");
      WriteSpi(SWITCHBOARD2WRITEADDR,GPIOB,0b00110000,SWITCHBOARD2_CS);
      vTaskDelay(10/portTICK_PERIOD_MS);
      limiterSwitch = GetBit(ReadSpi(STEPPERREADADDR,GPIOB,STEPPER_CS),1,2+4);
    }
  
}



void turnMotor(int steps, int motor, int GPIOAB, int wirteAddres, int cs){
  FastLED.clear();
  unsigned long timeTurningStarted = millis();// it takes max 19 seconds to turn from A to Z
  unsigned long currentTime = millis();
  WriteSpi(SWITCHBOARD2WRITEADDR,IODIRB,0b00000000,SWITCHBOARD2_CS);
  WriteSpi(STEPPERWRITEADDR,IODIRA,0b00000000,STEPPER_CS);
  WriteSpi(STEPPERWRITEADDR,IODIRB,0b11110000,STEPPER_CS);
  
  //motor 3 and 2 are on gpioA, 1 on gpioB
  //motor 2 is attatched to the last 4 bits of the GPIOA => <<4
  int step1 = (motor==1 || motor==3) ? 0b00000011 : 0b00110000;
  int step2 = (motor==1 || motor==3) ? 0b00000110 : 0b01100000;
  int step3 = (motor==1 || motor==3) ? 0b00001100 : 0b11000000;
  int step4 = (motor==1 || motor==3) ? 0b00001001 : 0b10010000;
  if (motor == 2){
    step1=0b00110000;
    step2=0b01100000;
    step3=0b11000000;
    step4=0b10010000;
  }
  int limiterSwitch = GetBit(ReadSpi(STEPPERREADADDR,GPIOB,STEPPER_CS),1,motor+4);
  Serial.println("limiter switch: "+String(limiterSwitch));

  if (steps <0){
    Serial.println("started this-------------");
    turnMotorBack(steps,motor,GPIOAB,wirteAddres,cs);
    
  } else if (steps >0)
  {
    for (int j = 0; j < steps*stepSizeLetter; j++)
    {
      currentTime = millis();
      if (limiterSwitch || (currentTime-timeTurningStarted >19000))
      {
        if (motor == 1)
        {
          turnMotorBack(-3,motor,GPIOAB,wirteAddres,cs);
        }else if (motor== 2)
        {
          turnMotorBack(-4,motor,GPIOAB,wirteAddres,cs);
        }else if (motor == 3)
        {
          turnMotorBack(-4,motor,GPIOAB,wirteAddres,cs);
        }
        Serial.println("turningBack");
        break;
      }
      WriteSpi(wirteAddres,GPIOAB,step4,cs);
      vTaskDelay(10/portTICK_PERIOD_MS);
      WriteSpi(wirteAddres,GPIOAB,step3,cs);
      vTaskDelay(10/portTICK_PERIOD_MS);
      WriteSpi(wirteAddres,GPIOAB,step2,cs);
      vTaskDelay(10/portTICK_PERIOD_MS);
      WriteSpi(wirteAddres,GPIOAB,step1,cs);
      vTaskDelay(10/portTICK_PERIOD_MS);
      limiterSwitch = GetBit(ReadSpi(STEPPERREADADDR,GPIOB,STEPPER_CS),1,motor+4);
    }
  }
  
}

//keyboard functions
int readButtonInputs(){
  gpioa_stateOriginal = ReadSpi(KEYBOARDREADADDR, GPIOA,KEYBOARD_CS);//read the gpio a register
  if (GetBit(gpioa_stateOriginal,1,7))
  {
    Serial.println("stopbutton7");
    pressedStopButton = true;
  }
  
  gpioa_state = gpioa_stateOriginal;
  vTaskDelay(10/portTICK_PERIOD_MS);

  while (gpioa_state !=0) //as long as the button is pushed keep reading the register
  {
    if(get_index(acceptibleKeyboardPresses,gpioa_state)==30){
      keyboardProblemLed();
    }
    Serial.println("gpio "+ String(gpioa_stateOriginal));
    gpioa_state = ReadSpi(KEYBOARDREADADDR, GPIOA,KEYBOARD_CS);
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
  global_state = gpioa_state;  
  
  return gpioa_stateOriginal;//when the button is released return what was in the register
}

void setButtonOutput(int binairyNumber){
  WriteSpi(KEYBOARDWRITEADDR,GPIOB,binairyNumber,KEYBOARD_CS);  
  vTaskDelay(10/portTICK_PERIOD_MS);
}

vector<unsigned short int> rotateVector(unsigned short int amountRotation, vector<unsigned short int> vectorToRotate){
    for (size_t i = 0; i < amountRotation; i++)
    {
        vectorToRotate.push_back(vectorToRotate[0]);
        vectorToRotate.erase(vectorToRotate.begin()); 
    }
    return vectorToRotate;
}

void turnRotorWithKeyboard(){
  
  rotorRight.currentPosition += 1;
  rotorRight.vectorRotorA = rotateVector(1,rotorRight.vectorRotorA);
  rotorRight.vectorRotorB = rotateVector(1,rotorRight.vectorRotorB);
  rotorRight.vectorRotorC = rotateVector(1,rotorRight.vectorRotorC);
  if (rotorRight.currentPosition >26)
  {
    rotorRight.currentPosition = 1;
    stepSizeLetter = 5;
    turnMotor(8000,3,GPIOA,STEPPERWRITEADDR,STEPPER_CS);
    stepSizeLetter = 18;
  }else{
    turnMotor(-1,3,GPIOA,STEPPERWRITEADDR,STEPPER_CS);
  }
  
  Serial.println("turning rotorRight 1 pos");
  Serial.println("current position "+String(rotorRight.currentPosition)+" position tot turn left rotor "+String(rotorRight.positionTurnRotorToLeft));
  if (rotorRight.currentPosition == rotorRight.positionTurnRotorToLeft)
  {
    //hardware rotate
    
    rotorMid.currentPosition += 1;
    
    //software rotate
    rotorMid.vectorRotorA = rotateVector(1,rotorMid.vectorRotorA);
    rotorMid.vectorRotorB = rotateVector(1,rotorMid.vectorRotorB);
    rotorMid.vectorRotorC = rotateVector(1,rotorMid.vectorRotorC);
    if (rotorMid.currentPosition >26)
    {
      rotorMid.currentPosition = 1;
      stepSizeLetter = 5;
      turnMotor(8000,2,GPIOB,SWITCHBOARD2WRITEADDR,SWITCHBOARD2_CS);
      stepSizeLetter = 18;
    }else{
      turnMotor(-1,2,GPIOB,SWITCHBOARD2WRITEADDR,SWITCHBOARD2_CS);
    }
    
    Serial.println("turning rotorMid 1 pos");
  }
  if (rotorMid.currentPosition == rotorMid.positionTurnRotorToLeft && alreadyTurnedLeft==false)
  {
    
    rotorLeft.currentPosition += 1;
    rotorLeft.vectorRotorA = rotateVector(1,rotorLeft.vectorRotorA);
    rotorLeft.vectorRotorB = rotateVector(1,rotorLeft.vectorRotorB);
    rotorLeft.vectorRotorC = rotateVector(1,rotorLeft.vectorRotorC);
    if (rotorLeft.currentPosition>26)
    {
      rotorLeft.currentPosition = 1;
      stepSizeLetter = 5;
      turnMotor(8000,1,GPIOB,STEPPERWRITEADDR,STEPPER_CS);
      stepSizeLetter = 18;
    }
    else{
      turnMotor(-1,1,GPIOB,STEPPERWRITEADDR,STEPPER_CS);
    }
    alreadyTurnedLeft = true;
    Serial.println("turning rotorLeft 1 pos");
  }
}

int encodeLetter(int letter){
  int outLetterFromSwitchboard;
  int indexSwitchboardA = get_index(switchboardA,letter);
  int indexSwitchboardB = get_index(switchboardB,letter);
  if (indexSwitchboardA != 30){
    outLetterFromSwitchboard = switchboardB[indexSwitchboardA];
  }else if(indexSwitchboardB !=30){
    outLetterFromSwitchboard = switchboardA[indexSwitchboardB];
  }else{
    outLetterFromSwitchboard = letter;
  }
  Serial.println("letter                  : "+String(letter));
  Serial.println("outletterFromSwitchboard: "+String(outLetterFromSwitchboard));

  int positionB = rotorRight.vectorRotorB[outLetterFromSwitchboard-1]-rotorRight.currentPosition+1;
  if(positionB<1){
    positionB+=26;
  }
  Serial.println("rotorRight:   "+String(positionB));
  positionB = rotorMid.vectorRotorB[positionB-1]-rotorMid.currentPosition+1;
  if(positionB<1){
    positionB+=26;
  }
  Serial.println("rotorMid:   "+String(positionB));
  positionB = rotorLeft.vectorRotorB[positionB-1]-rotorLeft.currentPosition+1;
  if(positionB<1){
    positionB+=26;
  }
  Serial.println("rotorLeft:   "+String(positionB));



  
  //went trough all rotors, now trough reflector
  int outLetterFromReflector;
  int indexReflectorA = get_index(reflectora,positionB);
  if(indexReflectorA!= 30){
    outLetterFromReflector = reflectorb[indexReflectorA];
  }else{
    int indexReflextorB = get_index(reflectorb,positionB);
    outLetterFromReflector = reflectora[indexReflextorB];
  }
  Serial.println("outLetterReflector        : "+String(outLetterFromReflector));

  //back trough rotors
  positionB = rotorLeft.vectorRotorC[outLetterFromReflector-1]-rotorLeft.currentPosition+1;
  if(positionB<1){
    positionB+=26;
  }
  Serial.println("rotorLeft:   "+String(positionB));
  
  positionB = rotorMid.vectorRotorC[positionB-1]-rotorMid.currentPosition+1;
  if(positionB<1){
    positionB+=26;
  }
  Serial.println("rotorMid:   "+String(positionB));
  positionB = rotorRight.vectorRotorC[positionB-1]-rotorRight.currentPosition+1;
  if(positionB<1){
    positionB+=26;
  }
  Serial.println("rotorRight:   "+String(positionB));
  

  //went trough all rotors again, now once more trough switchboard
  indexSwitchboardA = get_index(switchboardA,positionB);
  indexSwitchboardB = get_index(switchboardB,positionB);
  if (indexSwitchboardA != 30){
    outLetterFromSwitchboard = switchboardB[indexSwitchboardA];
  }else if(indexSwitchboardB != 30){
    outLetterFromSwitchboard = switchboardA[indexSwitchboardB];
  }else{
    outLetterFromSwitchboard = positionB;
  }
  Serial.println("outletterFromSwitchboard: "+String(outLetterFromSwitchboard));

  return outLetterFromSwitchboard;

}

void printVectors(){
  Serial.println("A vector");
  for (size_t i = 0; i < rotorRight.vectorRotorA.size(); i++)
  {
    Serial.print(rotorRight.vectorRotorA[i]);
    Serial.print(",");
  }
  Serial.println("A vector");
  for (size_t i = 0; i < rotorRight.vectorRotorB.size(); i++)
  {
    Serial.print(rotorRight.vectorRotorB[i]);
    Serial.print(",");
  }
  Serial.println("A vector");
  for (size_t i = 0; i < rotorRight.vectorRotorC.size(); i++)
  {
    Serial.print(rotorRight.vectorRotorC[i]);
    Serial.print(",");
  }
}


void getLetterFromInputs(int column, int row){
  String letter;
  int encodedLetter;
  //collumn is the number in GPIOA
  if (column == 1)
  {
    letter = keyboardPoints[row][0];
    Serial.println("letter: "+ letter);
    printVectors();
    encodedLetter = encodeLetter(keyboardPointsInt[row][0]);
    textToPost += letterNumber[encodedLetter-1];
    turnRotorWithKeyboard();
    Serial.println("the encoded letter went from "+ String(letter)+" to "+String(encodedLetter));
    showPositionLed(encodedLetter);
  } else if (column == 2)
  {
    letter = keyboardPoints[row][1];
    Serial.println("letter: "+ letter);
    printVectors();
    encodedLetter = encodeLetter(keyboardPointsInt[row][1]);
    textToPost += letterNumber[encodedLetter-1];
    turnRotorWithKeyboard();
    Serial.println("the encoded letter went from "+ String(letter)+" to "+String(encodedLetter));
    showPositionLed(encodedLetter);
  } else if (column == 4)
  {
    letter = keyboardPoints[row][2];
    Serial.println("letter: "+ letter);
    printVectors();
    encodedLetter = encodeLetter(keyboardPointsInt[row][2]);
    textToPost += letterNumber[encodedLetter-1];
    turnRotorWithKeyboard();
    Serial.println("the encoded letter went from "+ String(letter)+" to "+String(encodedLetter));
    showPositionLed(encodedLetter);
  } else if (column == 8)
  {
    letter = keyboardPoints[row][3];
    Serial.println("letter: "+ letter);
    printVectors();
    encodedLetter = encodeLetter(keyboardPointsInt[row][3]);
    textToPost += letterNumber[encodedLetter-1];
    turnRotorWithKeyboard();
    Serial.println("the encoded letter went from "+ String(letter)+" to "+String(encodedLetter));
    showPositionLed(encodedLetter);
  }else if (column == 16)
  {
    letter = keyboardPoints[row][4];
    Serial.println("letter: "+ letter);
    printVectors();
    encodedLetter = encodeLetter(keyboardPointsInt[row][4]);
    textToPost += letterNumber[encodedLetter-1];
    turnRotorWithKeyboard();
    Serial.println("the encoded letter went from "+ String(letter)+" to "+String(encodedLetter));
    showPositionLed(encodedLetter);
  }else if (column == 32)
  {
    letter = keyboardPoints[row][5];
    Serial.println("letter: "+ letter);
    printVectors();
    encodedLetter = encodeLetter(keyboardPointsInt[row][5]);
    textToPost += letterNumber[encodedLetter-1];
    turnRotorWithKeyboard();
    Serial.println("the encoded letter went from "+ String(letter)+" to "+String(encodedLetter));
    showPositionLed(encodedLetter);
  }else if (column == 64)
  {
    letter = keyboardPoints[row][6];
    Serial.println("letter: "+ letter);
    printVectors();
    encodedLetter = encodeLetter(keyboardPointsInt[row][6]);
    textToPost += letterNumber[encodedLetter-1];
    turnRotorWithKeyboard();
    Serial.println("the encoded letter went from "+ String(letter)+" to "+String(encodedLetter));
    showPositionLed(encodedLetter);
  }
  
  
}




int readRotary(int registerA, int NrA, int NrB, int motor, int GPIOAB){
  int aState = GetBit(registerA,1,NrA); // Reads the "current" state of the outputA
  int bState = GetBit(registerA,1,NrB);
  int counter = 0;

  if (NrA == 1)
  {
    ALastState = A1LastState;
    counter = rotary1Counter;
    rotorLeft.currentPosition = counter;
  }else if(NrA == 3){
    ALastState = A2LastState;
    counter = rotary2Counter;
    rotorMid.currentPosition = counter;
  }else if(NrA ==5){
    ALastState = A3LastState;
    counter = rotary3Counter;
    rotorRight.currentPosition = counter;
  }



  // If the previous and the current state of the outputA are different, that means a Pulse has occured
  if (aState < ALastState){     
    // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
    if (bState == aState) { 
      counter ++;
      if(counter >26){
        counter = 1;
      }      

    } else {
      counter --;
      if(counter<1){
        counter = 26;
      }

    }
    showPositionLed(counter);
    Serial.print("Position: ");
    Serial.println(counter);
  } 
  ALastState = aState; // Updates the previous state of the outputA with the current state

  if (NrA == 1)
  {
    A1LastState = aState;
    rotary1Counter = counter;
  }else if(NrA == 3){
    A2LastState = aState;
    rotary2Counter = counter;
  }else if(NrA ==5){
    A3LastState = aState;
    rotary3Counter = counter;
  }

  return ALastState;
}

void PushButtonRotary(int buttonRegister, int bitNr, Rotor rotor){
  if (GetBit(buttonRegister,1,bitNr))
  {
    while (GetBit(buttonRegister,1,bitNr))
    {
      buttonRegister = ReadSpi(ROTARYREADADDR,GPIOB,ROTARY_CS);
    }
  
    actualRotor += 1;
    if(actualRotor>5){
      actualRotor = 1;
    }
    // Rotor* pRotorLeft = &rotorLeft;
    int newRotorChoice = rotor.buttonPushed();
    Serial.println(newRotorChoice);
    rotor.rotorChoice = newRotorChoice;
    
    // Serial.println(String(rotorLeft.getVectorA()));
    // Serial.println(rotorLeft.getVectorB());
    rotaryPushed = true;
    Serial.println("rotor= "+String(rotor.rotorChoice));
  }
}

void readSwitchboard(){
  int allInput = 0b11111111;
  int changedOutput = 0b11111110;
  int outputLevel = 0b00000001;
  // int changedOutput;
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
    changedOutput = rotl(changedOutput,i);
    Serial.println(changedOutput);
    //set only 1 as output, all the rest are inputs
    WriteSpi(SWITCHBOARD1WRITEADDR,IODIRA,changedOutput,SWITCHBOARD1_CS);// one after the other the pins become an output
    WriteSpi(SWITCHBOARD1WRITEADDR,IODIRB,allInput,SWITCHBOARD1_CS);
    WriteSpi(SWITCHBOARD2WRITEADDR,IODIRA,allInput,SWITCHBOARD2_CS);
    WriteSpi(SWITCHBOARD2WRITEADDR,IODIRB,allInput,SWITCHBOARD2_CS);

    int inverseChangedOutput = invertBits(changedOutput);

    //set the inverse from changed output(vb11111110) in GPIOA => 00000001
    //so now only 1 pin is high
    WriteSpi(SWITCHBOARD1WRITEADDR,GPIOA,inverseChangedOutput,SWITCHBOARD1_CS);

    //read all the switchboard registers
    reg3A = ReadSpi(SWITCHBOARD1READADDR,GPIOA,SWITCHBOARD1_CS);
    reg3B = ReadSpi(SWITCHBOARD1READADDR,GPIOB,SWITCHBOARD1_CS);

    reg4A = ReadSpi(SWITCHBOARD2READADDR,GPIOA,SWITCHBOARD2_CS);
    reg4B = ReadSpi(SWITCHBOARD2READADDR,GPIOB,SWITCHBOARD2_CS);

    highBitOutput = GetBitHigh(inverseChangedOutput);//need to know which bit was set to high so it can be ignored when searching for the high input

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
      firstLetter = switchboardNumberChange[firstLetter-1];
      secondLetter = switchboardNumberChange[secondLetter-1];
      switchboardA.push_back(firstLetter);
      switchboardB.push_back(secondLetter);
    }
  }
  //spi3 with register B changes output
  for (size_t i = 0; i < 8 ; i++)
  {
    changedOutput = rotl(changedOutput,i);
    //set only 1 as output
    WriteSpi(SWITCHBOARD1WRITEADDR,IODIRA,allInput,SWITCHBOARD1_CS);
    WriteSpi(SWITCHBOARD1WRITEADDR,IODIRB,changedOutput,SWITCHBOARD1_CS);// one after the other the pins become an output
    WriteSpi(SWITCHBOARD2WRITEADDR,IODIRA,allInput,SWITCHBOARD2_CS);
    WriteSpi(SWITCHBOARD2WRITEADDR,IODIRB,allInput,SWITCHBOARD2_CS);

    int inverseChangedOutput = invertBits(changedOutput);
    WriteSpi(SWITCHBOARD1WRITEADDR,GPIOB,inverseChangedOutput,SWITCHBOARD1_CS);

    //read all the switchboard registers
    reg3A = ReadSpi(SWITCHBOARD1READADDR,GPIOA,SWITCHBOARD1_CS);
    reg3B = ReadSpi(SWITCHBOARD1READADDR,GPIOB,SWITCHBOARD1_CS);

    reg4A = ReadSpi(SWITCHBOARD2READADDR,GPIOA,SWITCHBOARD2_CS);
    reg4B = ReadSpi(SWITCHBOARD2READADDR,GPIOB,SWITCHBOARD2_CS);

    highBitOutput = GetBitHigh(inverseChangedOutput);//need to know which bit was set to high so it can be ignored when searching for the high input

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
      firstLetter = switchboardNumberChange[firstLetter-1];
      secondLetter = switchboardNumberChange[secondLetter-1];
      switchboardA.push_back(firstLetter);
      switchboardB.push_back(secondLetter);
    }
  }

  //spi4 with register A changes output
  for (size_t i = 0; i < 8 ; i++)
  {
    changedOutput = rotl(changedOutput,i);
    //set only 1 as output
    WriteSpi(SWITCHBOARD1WRITEADDR,IODIRA,allInput,SWITCHBOARD1_CS);
    WriteSpi(SWITCHBOARD1WRITEADDR,IODIRB,allInput,SWITCHBOARD1_CS);// one after the other the pins become an output
    WriteSpi(SWITCHBOARD2WRITEADDR,IODIRA,changedOutput,SWITCHBOARD2_CS);
    WriteSpi(SWITCHBOARD2WRITEADDR,IODIRB,allInput,SWITCHBOARD2_CS);

    int inverseChangedOutput = invertBits(changedOutput);
    WriteSpi(SWITCHBOARD2WRITEADDR,GPIOA,inverseChangedOutput,SWITCHBOARD2_CS);

    //read all the switchboard registers
    reg3A = ReadSpi(SWITCHBOARD1READADDR,GPIOA,SWITCHBOARD1_CS);
    reg3B = ReadSpi(SWITCHBOARD1READADDR,GPIOB,SWITCHBOARD1_CS);

    reg4A = ReadSpi(SWITCHBOARD2READADDR,GPIOA,SWITCHBOARD2_CS);
    reg4B = ReadSpi(SWITCHBOARD2READADDR,GPIOB,SWITCHBOARD2_CS);

    highBitOutput = GetBitHigh(inverseChangedOutput);//need to know which bit was set to high so it can be ignored when searching for the high input

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
      firstLetter = switchboardNumberChange[firstLetter-1];
      secondLetter = switchboardNumberChange[secondLetter-1];
      switchboardA.push_back(firstLetter);
      switchboardB.push_back(secondLetter);
    }
  }

  //spi4 with register B changes output
  for (size_t i = 0; i < 8 ; i++)
  {
    changedOutput = rotl(changedOutput,i);
    //set only 1 as output
    WriteSpi(SWITCHBOARD1WRITEADDR,IODIRA,allInput,SWITCHBOARD1_CS);
    WriteSpi(SWITCHBOARD1WRITEADDR,IODIRB,allInput,SWITCHBOARD1_CS);// one after the other the pins become an output
    WriteSpi(SWITCHBOARD2WRITEADDR,IODIRA,allInput,SWITCHBOARD2_CS);
    WriteSpi(SWITCHBOARD2WRITEADDR,IODIRB,changedOutput,SWITCHBOARD2_CS);

    int inverseChangedOutput = invertBits(changedOutput);
    WriteSpi(SWITCHBOARD2WRITEADDR,GPIOB,inverseChangedOutput,SWITCHBOARD2_CS);

    //read all the switchboard registers
    reg3A = ReadSpi(SWITCHBOARD1READADDR,GPIOA,SWITCHBOARD1_CS);
    reg3B = ReadSpi(SWITCHBOARD1READADDR,GPIOB,SWITCHBOARD1_CS);

    reg4A = ReadSpi(SWITCHBOARD2READADDR,GPIOA,SWITCHBOARD2_CS);
    reg4B = ReadSpi(SWITCHBOARD2READADDR,GPIOB,SWITCHBOARD2_CS);

    highBitOutput = GetBitHigh(inverseChangedOutput);//need to know which bit was set to high so it can be ignored when searching for the high input
    
   
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
      firstLetter = switchboardNumberChange[firstLetter-1];
      secondLetter = switchboardNumberChange[secondLetter-1];
      switchboardA.push_back(firstLetter);
      switchboardB.push_back(secondLetter);
    }
  }

}

void setupTurnMotor(int previousPosition, int currentPosition, int motor, int GPIOAB, int writeAddres, int cs){
  int difference = previousPosition-currentPosition;
  if (difference<0){
    Serial.println("motor "+String(motor)+" previous "+String(previousPosition)+" currentPosition "+String(currentPosition)+" stepps "+String(difference));
    turnMotor(difference,motor,GPIOAB,writeAddres,cs);
  }
}









///////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////SETUP/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);    
  pinMode(KEYBOARD_CS,OUTPUT);
  pinMode(STEPPER_CS,OUTPUT);
  pinMode(SWITCHBOARD1_CS,OUTPUT);
  pinMode(SWITCHBOARD2_CS,OUTPUT);
  pinMode(ROTARY_CS,OUTPUT);
  pinMode(21,INPUT);

  WriteSpi(ROTARYWRITEADDR,IODIRA,0b11111111,ROTARY_CS);
  A1LastState = GetBit(ReadSpi(ROTARYREADADDR,GPIOA,ROTARY_CS),1,1);
  A2LastState = GetBit(ReadSpi(ROTARYREADADDR,GPIOA,ROTARY_CS),1,3);
  A3LastState = GetBit(ReadSpi(ROTARYREADADDR,GPIOA,ROTARY_CS),1,5);

  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(10);


  SPI.begin();
    
}
///////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////MAIN//////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

void loop() {
rotorLeft.currentPosition = 1;
rotorLeft.previousPositionSetup = 1;
rotorMid.currentPosition = 1;
rotorMid.previousPositionSetup = 1;
rotorRight.currentPosition = 1;
rotorRight.previousPositionSetup = 1;

WriteSpi(STEPPERWRITEADDR,IODIRA,0b00000000,STEPPER_CS);
WriteSpi(STEPPERWRITEADDR,IODIRB,0b11110000,STEPPER_CS);
WriteSpi(SWITCHBOARD2WRITEADDR,IODIRB,0b00000000,SWITCHBOARD2_CS);

// wifiConnect();
stepSizeLetter = 5;

//on startup turn all rotors to A
turnMotor(8000,1,GPIOB,STEPPERWRITEADDR,STEPPER_CS);//8000 is just a number bigger than a full turn so the limiter switch will be pushed
// turnMotor2();
turnMotor(8000,2,GPIOB,SWITCHBOARD2WRITEADDR,SWITCHBOARD2_CS);
turnMotor(8000,3,GPIOA,STEPPERWRITEADDR,STEPPER_CS);
stepSizeLetter = 18;
///////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////SETUP ENIGMA//////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//rotary encoders are read to setup te rotors
//push on the rotary changes selection of rotor, turning the rotary turns the rotor
//the setup continues until the start button is pressed
WriteSpi(ROTARYWRITEADDR,IODIRA,0b11111111,ROTARY_CS);
WriteSpi(ROTARYWRITEADDR,IODIRB,0b11111111,ROTARY_CS);
WriteSpi(STEPPERWRITEADDR,IODIRA,0b00000000,STEPPER_CS);
WriteSpi(STEPPERWRITEADDR,IODIRB,0b11110000,STEPPER_CS);


Serial.println("started rotary read");
rotaryLeds();
textToPost = "";

while (pressedStartButton == false)
{
  int registerA = ReadSpi(ROTARYREADADDR,GPIOA,ROTARY_CS);
  // Serial.println(registerA,GetBit(registerA,1,1));
  A1LastState = readRotary(registerA,1,2,1,GPIOA);
  A2LastState = readRotary(registerA,3,4,2,GPIOA);
  A3LastState = readRotary(registerA,5,6,3,GPIOB);
  
  //check if the buttons from the rotary are pushed
  int buttonsRotary = ReadSpi(ROTARYREADADDR,GPIOB,ROTARY_CS);
  actualRotor = rotorLeft.rotorChoice;
  PushButtonRotary(buttonsRotary,1,rotorLeft);
  rotorLeft.rotorChoice = actualRotor;
  actualRotor = rotorMid.rotorChoice;
  PushButtonRotary(buttonsRotary,2,rotorMid);
  rotorMid.rotorChoice = actualRotor;
  actualRotor = rotorRight.rotorChoice;
  PushButtonRotary(buttonsRotary,3,rotorRight);
  rotorRight.rotorChoice = actualRotor;
  if(rotaryPushed){
    rotaryPushed = false;
    showLedsRotorChoice(rotorLeft.rotorChoice,rotorMid.rotorChoice,rotorRight.rotorChoice);
  }
  

  //check if start message button is pressed

  pressedStartButton= GetBit(ReadSpi(STEPPERREADADDR,GPIOB,STEPPER_CS),1,8);
}
pressedStartButton = false;
Serial.println("ended rotary read");


//turn the motors
WriteSpi(STEPPERWRITEADDR,IODIRA,0b00000000,STEPPER_CS);
WriteSpi(STEPPERWRITEADDR,IODIRB,0b11110000,STEPPER_CS);
// WriteSpi(STEPPERWRITEADDR,GPIOA,0b00001111,STEPPER_CS);
Serial.println("current position rotor left"+String(rotorLeft.currentPosition));
Serial.println("current position rotor mid"+String(rotorMid.currentPosition));
Serial.println("current position rotor right"+String(rotorRight.currentPosition));


setupTurnMotor(rotorLeft.previousPositionSetup,rotorLeft.currentPosition,1,GPIOB,STEPPERWRITEADDR,STEPPER_CS);
setupTurnMotor(rotorMid.previousPositionSetup,rotorMid.currentPosition,2,GPIOB,SWITCHBOARD2WRITEADDR,SWITCHBOARD2_CS);
setupTurnMotor(rotorRight.previousPositionSetup,rotorRight.currentPosition,3,GPIOA,STEPPERWRITEADDR,STEPPER_CS);



Serial.println("done turning");

//now that the rotors are chosen, assiciate the correct vectors with it
rotorLeft.vectorRotorA = possibleRotors[(rotorLeft.rotorChoice*3)-3];
rotorLeft.vectorRotorB = possibleRotors[(rotorLeft.rotorChoice*3)-2]; 
rotorLeft.vectorRotorC = possibleRotors[(rotorLeft.rotorChoice*3)-1];

rotorMid.vectorRotorA = possibleRotors[(rotorMid.rotorChoice*3)-3];
rotorMid.vectorRotorB = possibleRotors[(rotorMid.rotorChoice*3)-2]; 
rotorMid.vectorRotorC = possibleRotors[(rotorMid.rotorChoice*3)-1];

rotorRight.vectorRotorA = possibleRotors[(rotorRight.rotorChoice*3)-3];
rotorRight.vectorRotorB = possibleRotors[(rotorRight.rotorChoice*3)-2]; 
rotorRight.vectorRotorC = possibleRotors[(rotorRight.rotorChoice*3)-1];

for (size_t i = 0; i < rotorRight.vectorRotorA.size(); i++)
{
  Serial.print(" "+String(rotorRight.vectorRotorA[i]));
}
Serial.println();
for (size_t i = 0; i < rotorRight.vectorRotorB.size(); i++)
{
  Serial.print(" "+String(rotorRight.vectorRotorB[i]));
}
Serial.println();

rotorLeft.vectorRotorA = rotateVector(rotorLeft.currentPosition-1,rotorLeft.vectorRotorA);
rotorLeft.vectorRotorB = rotateVector(rotorLeft.currentPosition-1,rotorLeft.vectorRotorB);
rotorLeft.vectorRotorC = rotateVector(rotorLeft.currentPosition-1,rotorLeft.vectorRotorC);
rotorMid.vectorRotorA = rotateVector(rotorMid.currentPosition-1,rotorMid.vectorRotorA);
rotorMid.vectorRotorB = rotateVector(rotorMid.currentPosition-1,rotorMid.vectorRotorB);
rotorMid.vectorRotorC = rotateVector(rotorMid.currentPosition-1,rotorMid.vectorRotorC);
rotorRight.vectorRotorA = rotateVector(rotorRight.currentPosition-1,rotorRight.vectorRotorA);
rotorRight.vectorRotorB = rotateVector(rotorRight.currentPosition-1,rotorRight.vectorRotorB);
rotorRight.vectorRotorC = rotateVector(rotorRight.currentPosition-1,rotorRight.vectorRotorC);

     
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////SWITCHBOARD READ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

Serial.println("startedSwitchboard");
readSwitchboard();
String switchboardAString = "";
String switchboardBString = "";
Serial.println("A vector");
for (size_t i = 0; i < switchboardA.size(); i++)
{
  switchboardAString+=switchboardA[i]+",";
  Serial.print(switchboardA[i]);
  Serial.print(",");
}
Serial.println("B vector");
for (size_t i = 0; i < switchboardB.size(); i++)
{
  switchboardBString+=switchboardB[i]+",";
  Serial.print(switchboardB[i]);
  Serial.print(",");
}

Serial.println("endedSwitchboard");







ledsMoving();
  

////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////ENIGMA USE///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
  WriteSpi(KEYBOARDWRITEADDR,IODIRA,0b11111111,KEYBOARD_CS);
  WriteSpi(KEYBOARDWRITEADDR,IODIRB,0b11000000,KEYBOARD_CS);

  Serial.println("started reading keyboard");
  vTaskDelay(500/portTICK_PERIOD_MS);
  WriteSpi(KEYBOARDWRITEADDR,IODIRA,0b11111111,KEYBOARD_CS);
  WriteSpi(KEYBOARDWRITEADDR,IODIRB,0b11000000,KEYBOARD_CS);
  while (pressedStopButton == false)// condition is as long as the message end button isn't pressed
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
    //check if stop button is pressed
    pressedStopButton = GetBit(ReadSpi(KEYBOARDREADADDR, GPIOB,KEYBOARD_CS),1,7);

  }
  pressedStopButton = false;
  Serial.println("stopped reading keyboard");

  String settings = "rotor:"+String(rotorLeft.rotorChoice)+",position:"+String(rotorLeft.currentPosition)+";rotor:"+String(rotorMid.rotorChoice)+",position:"+String(rotorMid.currentPosition)+";rotor:"+String(rotorRight.rotorChoice)+",position:"+String(rotorRight.currentPosition)+";message:"+textToPost;
////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////SEND MESSAGE//////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
  vTaskDelay(1000/portTICK_PERIOD_MS);
  Serial.println("wifi connecting");
  wifiConnect();
  Serial.println("wifi done");
  
  sendWhatsapp(settings);
  postMessage();

  FastLED.clear();
  FastLED.show();





  rotorLeft.previousPositionSetup = rotorLeft.currentPosition;
  rotorMid.previousPositionSetup = rotorMid.currentPosition;
  rotorRight.previousPositionSetup = rotorRight.currentPosition;



}