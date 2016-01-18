/*************************************************************
NewSoftSerialST32 Test Program
Multi-instance Library/C++ Object for STM32Duino/ AVR Arduino
Copyright 2015 Ron Curry, InSyte Technologies

Notes:
- This is a companion app for testing SoftSerialSTM32. It can
be compiled for either AVR or an STM32 board with 
SOFTSERIALINT = 1. With SOFTSERIALINT = 0 it uses hardware 
serial port 3 on the Maple to enable as a remote test
vehicle connected to another board running the same app with
SOFTSERIALINT = 1.


The latest version of this software can always be found at
https://github.com/wingspinner

License
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use, copy,
* modify, merge, publish, distribute, sublicense, and/or sell copies
* of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
**************************************************************/

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <Arduino.h>

#define SOFTSERIALINT 1

#if SOFTSERIALINT

#include "SoftSerialSTM32.h"    
SoftSerialSTM32 SWSerial0(8,9);

#else

// Set for Maple (STM32F103) hardware serial port
#define SWSerial0 Serial3

#endif

// lDelay - Simple delay function independent of systick or interrupts
// Total delay is approximately 1ms per iteration @ 72mhz clock
// delay overhead is ~+1.487usec. one loop = ~137ns * 7300.
// Therefore, total delay is ~ (delay * 0.137us * 7300) + 1.487us
inline void lDelay(uint32_t delay) { 
  uint32_t i, j;

  j = delay * 7300;
  for (i = 0; i < j; i++) {
    asm volatile(
      "nop \n\t"
     ::);
  }
  
}


void setup() {
  // put your setup code here, to run once:

  Serial.begin(230400);

#if SOFTSERIALINT
  SWSerial0.begin(115200);
#else
  SWSerial0.begin(115200, SERIAL_8N1);
#endif
  

}


uint32_t baudrate[] = {300, 1200,2400, 4800,9600,14400,19200,28800,31250,38400,57600, 115200};
int16_t currentBaud = 11;
bool receiveFlag = false;
bool silentFlag = true;
int txFlag = 0;
int txTestFlag = 0;
int  exFlag = 0;
int  txSourceFlag = 0;
uint16_t stepSize = 1;
char testString[] = "Testing tx...\n";

void loop() {
  // put your main code here, to run repeatedly:
  char inChar;

  lDelay (3000);
  Serial.println("Starting now....");
//  while (1){};
  while (1) {
      
    if (exFlag) {
        while (1) {
          while (!SWSerial0.available()) {
            if (Serial.available())
              goto jOut;
          }
              
          inChar = SWSerial0.read();
          SWSerial0.write(inChar);
        }
    }
jOut:      

    if (txFlag) {
      if (txTestFlag == 0) {
          SWSerial0.write(testString);
          lDelay(10);
      } else if (txTestFlag == 1){
        SWSerial0.write(0x55);
        lDelay(10);
      } else if (txTestFlag == 2) {
        SWSerial0.write(0x15);
        lDelay(10);        
      } else if (txTestFlag == 3) {
        SWSerial0.write(0x0f);
        lDelay(10);        
      } else if (txTestFlag == 4) {
        SWSerial0.write(0xf0);
        lDelay(10);        
      }
    }
    
    if (receiveFlag)
      if (SWSerial0.available()) {
          inChar = SWSerial0.read();
          if (!silentFlag) Serial.write(inChar);

          #if SOFTSERIALINT
          if (SWSerial0.overflow())
            Serial.println("\nOVERFLOW"); 
          #endif
       }
  
    if (Serial.available()) {
  
      inChar = Serial.read();
  
      switch (inChar) {
        case 'x':
          exFlag ^= 1;
          Serial.print("\nSerial Exchange Test = "); Serial.println(exFlag, DEC);
          break;
        case '?':
          Serial.println("Commands....");
          Serial.println("R - Toggle on/off receive test");
          Serial.println("p - Toggle on/off print received bytes to console");
          Serial.println("T - Toggle on/off transmit test");
          Serial.println("5 - Toggle between sending a string or 0x55");
          Serial.println("x - Toggle on/off round robin send/receive test.");
          Serial.println("    (Requires SoftSerialInt running on seperate device)");
          Serial.println("B - Bump baud rate higher");
          Serial.println("b - Bump baud rate lower");
          Serial.println("s = Print status");
          break;
        case 's':
          Serial.print("\n\nR - "); Serial.print("RX Test off(0) or on(1) = "); Serial.println(receiveFlag, DEC);
          Serial.print("p - "); Serial.print("Print to console off(0) or on(1) = "); Serial.println(silentFlag, DEC);
          Serial.print("T - "); Serial.print("TX Test off(0) or on(1) = "); Serial.println(txFlag, DEC);
          Serial.print("5 - "); if (txTestFlag) Serial.println("Sending string"); else Serial.println("Sending 0x55");
          Serial.print("Baud = "); Serial.println(baudrate[currentBaud], DEC);

          #if SOFTSERIALINT
          Serial.print("Bitperiod = "); Serial.println(SWSerial0.getRXIntrabit(), DEC);
          Serial.print("Centering = "); Serial.println(SWSerial0.getRXCentering(), DEC);
          Serial.print("Stepsize = "); Serial.println(stepSize, DEC);
          #endif

          Serial.print("Available = "); Serial.println(SWSerial0.available(), DEC);
          break;
        case 'R':
          receiveFlag ^= true;
          break;
        case 'p':
          silentFlag ^= 1;
          break;
        case 'B':
          // bump up baud to next rate
          SWSerial0.end();
          currentBaud++;
          if (currentBaud > 11)
              currentBaud = 0;
            
          #if SOFTSERIALINT         
          SWSerial0.begin(baudrate[currentBaud]);
          #else
          SWSerial0.begin(baudrate[currentBaud], SERIAL_8N1);
          #endif
          
          Serial.println(); Serial.println(baudrate[currentBaud], DEC);
          break;
        case 'b':
          // bump up baud to next rate
          SWSerial0.end();
          currentBaud--;
          if (currentBaud < 0)
            currentBaud = 11;
            
          #if SOFTSERIALINT         
          SWSerial0.begin(baudrate[currentBaud]);
          #else
          SWSerial0.begin(baudrate[currentBaud], SERIAL_8N1);
          #endif
          
          Serial.println(); Serial.println(baudrate[currentBaud], DEC);
          break;

        #if SOFTSERIALINT
        case 'I':
          // bump up rx intrabit delay value
          SWSerial0.setRXIntrabit(SWSerial0.getRXIntrabit() + stepSize);
          break;
        case 'i':
          // bump up rx intrabit delay value
          SWSerial0.setRXIntrabit(SWSerial0.getRXIntrabit() - stepSize);
          break;
        case 'C':
          // bump up rx start bit delay value
          SWSerial0.setRXCentering(SWSerial0.getRXCentering() + stepSize);
          break;
        case 'c':
          // bump up rx start bit delay value
          SWSerial0.setRXCentering(SWSerial0.getRXCentering() - stepSize);
          break;
        case '0':
          // set step size to 1
          stepSize = 1;
          break;
        case '1':
          // set step size to 10
          stepSize = 10;
          break;
        case '2':
          // set step size to 100
          stepSize = 100;
          break;
        #endif
        
        case 'T':
          // do tx test
          txFlag ^= 1;
          break;
        case '5':
          txTestFlag++;
          if (txTestFlag > 4)
            txTestFlag = 0;
          break;
        default:
          break;      
      }   
    }    
  }
}
