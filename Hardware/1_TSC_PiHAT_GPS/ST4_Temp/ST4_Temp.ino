
// this code is part of "TSC", a free control software for astronomical telescopes
// Copyright (C)  2016-18, wolfgang birkfellner
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//---------------------------------------------------
// arduino sketch for monitoring ST4 and reading the temperature sensor

#include <SPI.h>
#include <stdlib.h>

float temp;
int decTemp;
int nSwitch = 1; // move north button
int eSwitch = 1; 
int sSwitch = 1;
int wSwitch = 1;
int northIsUp, westIsUp, eastIsUp, southIsUp; 
int switchStateChanged = 0, tempDeg;
char tempDeg1, tempDeg2, isPositive, st4state = '0';
bool debuggingIsOn = false, readTemp = true;
char reply, readCommand;
String dHelper, sHelper;
char buf[32];
volatile byte pos;
volatile boolean process_it;
float VRef = 4.52; // this is important - this should be 5.0 V, but due to the diode D1 it can be less. here, it is set to 4.6 V, which is 
            // sufficient to drive the arduino, but temperature readings are off if one does not take this into account 

//-----------------------------------------------------------------------

void setup() {
  if (debuggingIsOn == true) {
    Serial.begin(9600);
  }
  pinMode(MISO, OUTPUT); // have to send on master in, *slave out*
  SPCR |= _BV(SPE);  // turn on SPI in slave mode
  SPI.attachInterrupt();   // now turn on interrupts
  process_it = false;
  temp = (analogRead(4)*VRef*0.9765625-500)*0.1;
  if (debuggingIsOn) {
    Serial.println("TSC HAT Arduino Mini Pro is up...");
  }
}

//-----------------------------------------------------------------------

void loop() {
  // reading five analog inputs, the ST4 switches and the temperature sensor
  if (readTemp == true) { // read analog sensor only if temperature is not read out
    temp += (analogRead(4)*VRef*0.9765625-500)*0.1;
    temp = temp*0.5; // compute a running average
    if (debuggingIsOn) {
      Serial.print("Temperature: ");
      Serial.println(temp);
    }
  }
  
  if (analogRead(1) < 200) { // if the voltage drops below 1 V, the switch is closed ...
    northIsUp = 1;
  } else {
    northIsUp = 0;  
  }
  if (northIsUp != nSwitch) {
    nSwitch = northIsUp;
    switchStateChanged=1;
  }
  if (analogRead(0) < 200) {
    eastIsUp = 1;
  } else {
    eastIsUp = 0;  
  }
  if (eastIsUp != eSwitch) {
    eSwitch = eastIsUp;
    switchStateChanged=1;
  }
  if (analogRead(2) < 200) {
    southIsUp = 1;
  } else {
    southIsUp = 0;  
  }
  if (southIsUp != sSwitch) {
    sSwitch = southIsUp;
    switchStateChanged=1;
  }
  if (analogRead(3) < 200) {
    westIsUp = 1;
  } else {
    westIsUp = 0;  
  }
  if (westIsUp != wSwitch) {
    wSwitch = westIsUp;
    switchStateChanged=1;
  }
  if (switchStateChanged == 1) {
    if ((northIsUp == 1) && (eastIsUp == 1)) {
      st4state = '5';
      goto st4stateIsDeterminedLabel;
    }
    if ((northIsUp == 1) && (westIsUp == 1)) {
      st4state = '6';
      goto st4stateIsDeterminedLabel;
    }
    if ((southIsUp == 1) && (eastIsUp == 1)) {
      st4state = '7';
      goto st4stateIsDeterminedLabel;
    }
    if ((southIsUp == 1) && (westIsUp == 1)) {
      st4state = '8';
      goto st4stateIsDeterminedLabel;
    }
    if (northIsUp == 1) {
      st4state = '1';
      goto st4stateIsDeterminedLabel;
    }
    if (eastIsUp == 1) {
      st4state = '2';
      goto st4stateIsDeterminedLabel;
    }
    if (southIsUp == 1) {
      st4state = '3';
      goto st4stateIsDeterminedLabel;
    }    
    if (westIsUp == 1) {
      st4state = '4';
      goto st4stateIsDeterminedLabel;
    }
    if ((northIsUp == 0) && (eastIsUp == 0) && (southIsUp == 0) && (westIsUp == 0)) {
      st4state = '0';
    }
    st4stateIsDeterminedLabel: // couldn't help myself in that case :/
    if (debuggingIsOn) {
      Serial.print("State of ST4: ");
      Serial.println(st4state);
    }
  }
  switchStateChanged=0;
  // now convert the temperature to bytes
  if (temp > 0) {
    isPositive = '+';
  } else {
    isPositive = '-';
  }
  
  tempDeg=round(abs(temp));
  dHelper = String(tempDeg);
  if (tempDeg < 10) {
    tempDeg1 = '0'; 
    tempDeg2 = dHelper[0];
  } else {
    tempDeg1 = dHelper[0];
    tempDeg2 = dHelper[1];
  }
  
  if (process_it) { // got a string via SPI - process it accordingly
    pos = 0;
    readCommand=buf[0];
    process_it = false;
    switch (readCommand) {
      case 's': 
        reply = st4state; 
        break;
      case 'p': // ask whether temperature is positive and stop temperature readings
        readTemp = false;
        reply = isPositive;
        if (debuggingIsOn) {
          Serial.print("Temperature: ");
          Serial.println(temp);
        }
        break;
      case 'b': // ask for first digit of temperature in celsius
        reply = tempDeg1;
        break; 
      case 'l': // ask for second digit of temperature in celsius
        reply = tempDeg2;
        break;              
      case 'g': // this is sent when reading the temperature is terminated; reply is again set to "st4state" . temperature reading starts again with new averaging
        readTemp = true;
        temp = (analogRead(4)*VRef*0.9765625-500)*0.1;
        reply = st4state;       
    } 
  }
}

//--------------------------------------------------------------

ISR(SPI_STC_vect) { // SPI interrupt routine
byte c = SPDR;  // grab byte from SPI Data Register
  SPDR=reply;
  
  if (pos < sizeof buf) {
    if (c != 0x00) {     
      buf [pos++] = c;
    } else {
      buf [pos++] = 0x00;
      process_it = true;
    }
  }  
}  
