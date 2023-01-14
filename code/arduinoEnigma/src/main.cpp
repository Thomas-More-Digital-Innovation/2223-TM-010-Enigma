#include <Arduino.h>
#include <SPI.h>
#include <vector>
using namespace std;

class Rotor{
  public:
    int currentPosition;
    int positionTurnRotorToLeft;
    int rotorChoice;
    vector<unsigned short int> vectorRotorA;
    vector<unsigned short int> vectorRotorB;
    //constructor
    Rotor(int currentPosition,int positionTurnRotorToLeft, int rotorChoice,vector<unsigned short int> vectorRotorA,vector<unsigned short int> vectorRotorB){
      this->currentPosition = currentPosition;
      this->positionTurnRotorToLeft = positionTurnRotorToLeft;
      this->rotorChoice = rotorChoice;
      this->vectorRotorA = vectorRotorA;
      this->vectorRotorB = vectorRotorB;
    }
    




    //methods
    int buttonPushed(){
      
      rotorChoice +=1;
      if (rotorChoice>5){
        rotorChoice = 1;
      }
      Serial.println(rotorChoice);
      
      // setRotorChoice(rotorChoice);
      return rotorChoice;
    }
    // void buttonPushed(Rotor *rotor){
    //   Rotor rotor2 = *rotor;


    //   rotorChoice +=1;
    //   if (rotorChoice>5){
    //     rotorChoice = 1;
    //   }
      
    //   rotor2.rotorChoice=rotorChoice;
    //   // Serial.println(rotorChoice);
      
    //   // setRotorChoice(rotorChoice);
    //   // return rotorChoice;
    // }


};

vector<unsigned short int> vector_rotor_lefta;
vector<unsigned short int> vector_rotor_leftb;

vector<unsigned short int> vector_rotor_mida;
vector<unsigned short int> vector_rotor_midb;

vector<unsigned short int> vector_rotor_righta;
vector<unsigned short int> vector_rotor_rightb;

vector<unsigned short int> rotor1a({1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26});
vector<unsigned short int> rotor1b({16,5,12,26,7,1,11,15,24,22,2,17,14,8,19,20,18,21,3,23,25,10,4,6,13,9});

vector<unsigned short int> rotor2a({1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26});
vector<unsigned short int> rotor2b({10,25,22,13,12,16,23,2,1,8,11,15,26,17,18,21,4,20,3,24,5,6,7,9,14,19});

vector<unsigned short int> rotor3a({1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26});
vector<unsigned short int> rotor3b({1,13,16,6,15,14,12,18,23,25,3,19,7,21,10,11,19,24,8,2,20,4,5,22,26,9});

vector<unsigned short int> rotor4a({1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26});
vector<unsigned short int> rotor4b({9,6,2,8,13,20,19,3,11,22,16,15,14,23,25,17,26,1,12,21,10,4,24,5,18,7});

vector<unsigned short int> rotor5a({1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26});
vector<unsigned short int> rotor5b({1,11,18,12,22,19,21,2,8,10,5,15,14,16,20,25,23,3,4,24,9,17,7,26,13,6});

vector<unsigned short int> reflectora({8,5,6,16,7,9,2,26,10,14,25,12,15});
vector<unsigned short int> reflectorb({20,21,11,1,13,24,3,4,22,18,23,17,19});

vector<vector<unsigned short int>> possibleRotors{rotor1a,rotor1b,rotor2a,rotor2b,rotor3a,rotor3b,rotor4a,rotor4b,rotor5a,rotor5b};

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

char keyboardPoints[4][7] = {{'q','w','e','r','t','z','u'}, {'a','s','d','f','g','h','j'},{'p','y','x','c','v','b','n'},{'i','o','k','m','l','1','2'}};//'1' en '2' is enkel om [4][7] te doen kloppen.

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

Rotor rotorLeft(1,7,1,possibleRotors[0],possibleRotors[1]);
Rotor rotorMid(1,8,2,possibleRotors[2],possibleRotors[3]);
Rotor rotorRight(1,8,3,possibleRotors[4],possibleRotors[5]);

int actualRotor;
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


void getLetterFromInputs(int column, int row){
  String letter;
  //collumn is the number in GPIOA
  if (column == 1)
  {
    letter = keyboardPoints[row][0];
    Serial.println("letter: "+ letter);
  } else if (column == 2)
  {
    letter = keyboardPoints[row][1];
    Serial.println("letter: "+ letter);
  } else if (column == 4)
  {
    letter = keyboardPoints[row][2];
    Serial.println("letter: "+ letter);
  } else if (column == 8)
  {
    letter = keyboardPoints[row][3];
    Serial.println("letter: "+ letter);
  }else if (column == 16)
  {
    letter = keyboardPoints[row][4];
    Serial.println("letter: "+ letter);
  }else if (column == 32)
  {
    letter = keyboardPoints[row][5];
    Serial.println("letter: "+ letter);
  }else if (column == 64)
  {
    letter = keyboardPoints[row][6];
    Serial.println("letter: "+ letter);
  }
  
  
}

void turnMotor(int steps, int motor, int GPIOAB){
  //motor 1 and 2 are on gpioA, 3 on gpioB
  //motor 2 is attatched to the last 4 bits of the GPIOA => <<4
  int step1 = (motor==1 || motor==3) ? 0b00000011 : 0b00000011<<4;
  int step2 = (motor==1 || motor==3) ? 0b00000110 : 0b00000110<<4;
  int step3 = (motor==1 || motor==3) ? 0b00001100 : 0b00001100<<4;
  int step4 = (motor==1 || motor==3) ? 0b00001001 : 0b00001001<<4;

  Serial.println("step1 "+String(step1));
  Serial.println("step2 "+String(step2));
  Serial.println("step3 "+String(step3));
  Serial.println("step4 "+String(step4));

  if (steps >0){
    for (size_t i = 0; i < steps; i++)
    {
      WriteSpi(STEPPERWRITEADDR,GPIOAB,step1,STEPPER_CS);
      vTaskDelay(10/portTICK_PERIOD_MS);
      WriteSpi(STEPPERWRITEADDR,GPIOAB,step2,STEPPER_CS);
      vTaskDelay(10/portTICK_PERIOD_MS);
      WriteSpi(STEPPERWRITEADDR,GPIOAB,step3,STEPPER_CS);
      vTaskDelay(10/portTICK_PERIOD_MS);
      WriteSpi(STEPPERWRITEADDR,GPIOAB,step4,STEPPER_CS);
      vTaskDelay(10/portTICK_PERIOD_MS);
      Serial.println("step1 "+String(step1));
      Serial.println("step2 "+String(step2));
      Serial.println("step3 "+String(step3));
      Serial.println("step4 "+String(step4));
    }
  } else if (steps <0)
  {
    for (int j = 0; j > steps; j--)
    {
      WriteSpi(STEPPERWRITEADDR,GPIOAB,step4,STEPPER_CS);
      vTaskDelay(10/portTICK_PERIOD_MS);
      WriteSpi(STEPPERWRITEADDR,GPIOAB,step3,STEPPER_CS);
      vTaskDelay(10/portTICK_PERIOD_MS);
      WriteSpi(STEPPERWRITEADDR,GPIOAB,step2,STEPPER_CS);
      vTaskDelay(10/portTICK_PERIOD_MS);
      WriteSpi(STEPPERWRITEADDR,GPIOAB,step1,STEPPER_CS);
      vTaskDelay(10/portTICK_PERIOD_MS);
    }
  }
  
}

void turnMotor2(int steps){
  for (size_t i = 0; i < steps; i++)
  {
    WriteSpi(STEPPERWRITEADDR,GPIOA,0b00000011,STEPPER_CS);
    vTaskDelay(10/portTICK_PERIOD_MS);
    WriteSpi(STEPPERWRITEADDR,GPIOA,0b00000110,STEPPER_CS);
    vTaskDelay(10/portTICK_PERIOD_MS);
    WriteSpi(STEPPERWRITEADDR,GPIOA,0b00001100,STEPPER_CS);
    vTaskDelay(10/portTICK_PERIOD_MS);
    WriteSpi(STEPPERWRITEADDR,GPIOA,0b00001001,STEPPER_CS);
    vTaskDelay(10/portTICK_PERIOD_MS);
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
    if (bState != aState) { 
      counter ++;
      // turnMotor(10,motor,GPIOAB);
    } else {
      counter --;
      // turnMotor(-10,motor,GPIOAB);
    }
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
    rotor.vectorRotorA = possibleRotors[rotor.rotorChoice-1];//Haalt nog niet de juiste uit de lijst
    rotor.vectorRotorB = possibleRotors[rotor.rotorChoice];

    
    // Serial.println(String(rotorLeft.getVectorA()));
    // Serial.println(rotorLeft.getVectorB());

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
      switchboardA.push_back(firstLetter);
      switchboardB.push_back(secondLetter);
    }
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


  SPI.begin();
}
///////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////MAIN//////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

void loop() {
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

  //check if start message button is pressed

  pressedStartButton= GetBit(ReadSpi(STEPPERREADADDR,GPIOB,STEPPER_CS),1,8);
}
pressedStartButton = false;
Serial.println("ended rotary read");

//turn the motors
WriteSpi(STEPPERWRITEADDR,IODIRA,0b00000000,STEPPER_CS);
WriteSpi(STEPPERWRITEADDR,IODIRB,0b11110000,STEPPER_CS);
// WriteSpi(STEPPERWRITEADDR,GPIOA,0b00001111,STEPPER_CS);
Serial.println(rotorLeft.currentPosition);
// turnMotor2(80);
turnMotor(512,1,GPIOA);
turnMotor(rotorMid.currentPosition,2,GPIOA);
turnMotor(rotorRight.currentPosition,3,GPIOB);


     
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////SWITCHBOARD READ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

Serial.println("startedSwitchboard");
readSwitchboard();
for (size_t i = 0; i < switchboardA.size(); i++)
{
  Serial.println(switchboardA[i]);
}
Serial.println("B vector");
for (size_t i = 0; i < switchboardB.size(); i++)
{
  Serial.println(switchboardB[i]);
}
Serial.println("endedSwitchboard");

  

////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////ENIGMA USE///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
  WriteSpi(KEYBOARDWRITEADDR,IODIRA,0b11111111,KEYBOARD_CS);
  WriteSpi(KEYBOARDWRITEADDR,IODIRB,0b11000000,KEYBOARD_CS);

  Serial.println("started reading keyboard");
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

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////SEND MESSAGE//////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
  vTaskDelay(10000/portTICK_PERIOD_MS);



}