#include <Arduino.h>
#include <unity.h>
#include <vector>
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

vector<unsigned short int> rotor1a({ 1,2, 3, 4,5,6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26});
#define KEYBOARD_CS 14



void test_rotor_class(){
    Rotor testRotor(6,10,1,1,rotor1a,rotor1a,rotor1a);
    TEST_ASSERT_EQUAL(6, testRotor.currentPosition);
}

void test_rotor_choice(){
    Rotor testRotor(6,10,1,1,rotor1a,rotor1a,rotor1a);
    int newRotorChoice = testRotor.buttonPushed();
    testRotor.rotorChoice = newRotorChoice;
    TEST_ASSERT_EQUAL(2, testRotor.rotorChoice);
}

void test_cs_low(){
    digitalWrite(KEYBOARD_CS,LOW);
    TEST_ASSERT_EQUAL(LOW, digitalRead(KEYBOARD_CS));
}

void test_cs_high(){
    digitalWrite(KEYBOARD_CS,HIGH);
    TEST_ASSERT_EQUAL(HIGH, digitalRead(KEYBOARD_CS));
}


void setup()
{
  // NOTE!!! Wait for >2 secs
  // if board doesn't support software reset via Serial.DTR/RTS
  delay(2000);
  pinMode(KEYBOARD_CS, OUTPUT);
  

  UNITY_BEGIN(); // IMPORTANT LINE!
  RUN_TEST(test_rotor_class);
  RUN_TEST(test_rotor_choice);
  RUN_TEST(test_cs_high);
  RUN_TEST(test_cs_low);
}

void loop()
{
    UNITY_END(); // stop unit testing
}