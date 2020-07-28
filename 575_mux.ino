#include "Stepper595.h"
byte lnA   =  0x01; //blue
byte lnB   =  0x02; // pink
byte lnC   =  0x04; //yellow
byte lnD   =  0x08; //orange
uint8_t latchPin = 6;  // Latch pin of 74HC595 is connected to Digital pin 5
uint8_t clockPin = 5; // Clock pin of 74HC595 is connected to Digital pin 6
uint8_t dataPin = 4;  // Data pin of 74HC595 is connected to Digital pin 4
byte sqq[8]; /*= {
              B01110000,
              B01100000,
              B00111000,
              B01001000,
              B01011000,
              B00011000,
              B00111000,
              B00110000
            };*/

/*
 * setup() - this function runs once when you turn your Arduino on
 * We initialize the serial connection with the computer
 */
 Stepper595 * stt = new Stepper595(lnA,lnB,lnC,lnD,latchPin,clockPin,dataPin);
 String readString;
void setup() 
{
  Serial.begin(9600); // open the serial port at 9600 bps:
 
  /*sqq[0] = lnA;
  sqq[1] = lnA|lnB;
  sqq[2] = lnB;
  sqq[3] = lnB|lnC;
  sqq[4] = lnC;
  sqq[5] = lnC|lnD;
  sqq[6] = lnD;
  sqq[7] = lnA|lnD;*/
  stt->setSpeed(10);
  // Set all the pins of 74HC595 as OUTPUT
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);  
  pinMode(clockPin, OUTPUT);


  
  
}

/*
 * loop() - this function runs over and over again
 */
void loop() 
{
  stt->step(10);
  delay(50);
  stt->step(-10);
   if (Serial.available() > 0) {
    int inByte = Serial.read();
    
 
    switch (inByte) {
      case 'c':
        Serial.print("cc\n");
        stt->step(-200);
        Serial.print("-----------\n");
        break;
      case 'w':
        Serial.print("cw\n");
        stt->step(200);
        Serial.print("-----------\n");
        break;
      
    }
    
  }
 
  
}
