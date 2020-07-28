#pragma once
#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <string.h>
// #include <iostream>
using namespace std;

#ifndef STEPPER595_DEBUG
#define STEPPER595_DEBUG true
#endif
enum StepperDirection
{
    CW = 0,
    CC = 1
};
class Stepper595
{
private:
    const uint8_t fullRotationSteps = 4095; //4095 is a whole rotation
    uint8_t _pin1, _pin2, _pin3, _pin4;
    byte sequance[8]; // = {B01000, B01100, B00100, B00110, B00010, B00011, B00001, B01001};
    StepperDirection direction;

    uint8_t _mcuLatchPin = 6; // Latch pin of 74HC595 is connected to Digital pin 5
    uint8_t _mcuClockPin = 5; // Clock pin of 74HC595 is connected to Digital pin 6
    uint8_t _mcuDataPin = 4;  // Data pin of 74HC595 is connected to Digital pin 4
    long _time;               // store lasttime

    int8_t currentStep = 0;

    long stepperSpeed = 1000;  // milliseconds
    uint8_t stepperStepsJump = 1; // stages

    void buildSequence(bool reverese);
    void updateShiftRegister(byte data);
    void updateTime();
    byte getNextSequenceStep();
   

public:
   
    Stepper595(byte pin1, byte pin2, byte pin3, byte pin4, byte latchPin, byte clockPin, byte dataPin);
    ~Stepper595();
    void step(int8_t steps);
    void stop();
     void setSpeed(long whatSpeed);
};

Stepper595::Stepper595(byte pin1, byte pin2, byte pin3, byte pin4, byte latchPin, byte clockPin, byte dataPin)
{
    _pin1 = pin1;
    _pin2 = pin2;
    _pin3 = pin3;
    _pin4 = pin4;

    _mcuLatchPin = latchPin;
    _mcuClockPin = clockPin;
    _mcuDataPin = dataPin;

    this->setSpeed(5); // revs per minute
   
}
Stepper595::~Stepper595()
{
}

void Stepper595::buildSequence(bool reverese)
{
    if (reverese)
    {
        Serial.println("reverse");
        byte tmp = _pin4;
        _pin4 = _pin3;
        _pin3 = tmp;
    }
    sequance[0] = 0x00 | _pin1;
    sequance[1] = 0x00 | (_pin1 | _pin2);
    sequance[2] = 0x00 | _pin2;
    sequance[3] = 0x00 | (_pin2 | _pin3);
    sequance[4] = 0x00 | _pin3;
    sequance[5] = 0x00 | (_pin3 | _pin4);
    sequance[6] = 0x00 | _pin4;
    sequance[7] = 0x00 | (_pin1 | _pin4);
    /*
    sequance[0] = 0x00 | (_pin2 | _pin3 | _pin4); // step 1
    sequance[1] = 0x00 | (_pin3 | _pin4);         // step 2
    sequance[2] = 0x00 | (_pin1 | _pin3 | _pin4); // step 3
    sequance[3] = 0x00 | (_pin1 | _pin4);         // step 4
    sequance[4] = 0x00 | (_pin1 | _pin2 | _pin4); // step 5
    sequance[5] = 0x00 | (_pin1 | _pin2);         // step 6
    sequance[6] = 0x00 | (_pin1 | _pin2 | _pin3); // step 7
    sequance[7] = 0x00 | (_pin2 | _pin3);         // step 8
    */
}
void Stepper595::step(int8_t steps)
{
    StepperDirection prvDir = this->direction;
    this->direction = steps > 0 ? StepperDirection::CW : StepperDirection::CC;
    this->buildSequence(prvDir != this->direction);

    // set initial step
    if (this->direction == StepperDirection::CW)
        this->currentStep = 8;
    else
        this->currentStep = -1;
    
    if (steps != 0)
    {
        for (size_t i = 0; i < abs(steps); i++)
        {
            for (size_t j = 0; j < 8; j++)
            {
                updateShiftRegister(this->getNextSequenceStep());
                delay(this->stepperSpeed);
            }
        }
    }
    this->stop();
    delay(10);
}
/**
 *  Steps are between 0 - 7
 * **/
byte Stepper595::getNextSequenceStep()
{
    if (this->direction == StepperDirection::CW)
    {
        //Serial.println("going clockwise");
        this->currentStep++;
        if (this->currentStep > 7)
            this->currentStep = 0;
    }
    else
    {
        //Serial.println("going counter-clockwise");
        this->currentStep--;
        if (this->currentStep < 0)
            this->currentStep = 7;
    }
    return this->sequance[this->currentStep];
}
/*
 * updateShiftRegister() - This function sets the latchPin to low, then calls the Arduino function 'shiftOut' to shift out contents of variable 'leds' in the shift register before putting the 'latchPin' high again.
 */
void Stepper595::updateShiftRegister(byte data)
{

    digitalWrite(_mcuLatchPin, LOW); // set for writing

    //if (STEPPER595_DEBUG)
    //Serial.println("LATCH LOW");

    shiftOut(_mcuDataPin, _mcuClockPin, MSBFIRST, (byte)data); // set shift

    if (STEPPER595_DEBUG)
    {
        //Serial.println(data, BIN);
        //Serial.println(this->currentStep);
    }

    digitalWrite(_mcuLatchPin, HIGH); // write to 595

    //if (STEPPER595_DEBUG)
    //Serial.println("LATCH HIGH");
}

void Stepper595::stop()
{
    this->updateShiftRegister(0x00);
}

void Stepper595::updateTime()
{
    this->_time = micros();
}

/*
 * Sets the speed in revs per minute
 */
void Stepper595::setSpeed(long whatSpeed)
{
  this->stepperSpeed = 60L * 1000L * 1000L / this->fullRotationSteps / whatSpeed;
}
