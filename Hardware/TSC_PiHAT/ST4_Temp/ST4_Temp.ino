
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
int switchStateChanged = 0;
byte tempDeg, tempSub, isPositive;
bool debuggingIsOn = true;
char reply, readCommand;
String thelper, dhelper;
volatile boolean process_it;

//-----------------------------------------------------------------------

void setup() {
  if (debuggingIsOn == true) {
    Serial.begin(9600);
  }
  SPI.attachInterrupt();   // now turn on interrupts
  process_it = false;
}

//-----------------------------------------------------------------------

void loop() {
  // reading five analog inputs, the ST4 switches and the temperature sensor 
  temp = (analogRead(4)*4.8828-500)*0.1;
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
    if (debuggingIsOn) {
      Serial.print(nSwitch);
      Serial.print(eSwitch);
      Serial.print(sSwitch);
      Serial.println(wSwitch);
      Serial.println(temp);
      Serial.print("Temperature in bytes (isPositive - int - decimal: ");
      Serial.print(isPositive);
      Serial.print(" - ");
      Serial.print(tempDeg);
      Serial.print(" - ");
      Serial.println(decTemp);
    }
  }
  delay(5);
  switchStateChanged=0;

  // now convert the temperature to bytes
  if (temp > 0) {
    isPositive = 0x0001;
  } else {
    isPositive = 0x0000;
  }
  tempDeg=round(abs(temp));
  decTemp=round((temp-tempDeg)*10);

  if (process_it) { // got a string via SPI - process it accordingly
    process_it = false;
    switch (readCommand) {
      case 'n':
        if (nSwitch == 1) {
          reply = '1';
        } else {
          reply = '0';  
        }
        break;
      case 'e':  
        if (eSwitch == 1) {
          reply = '1';
        } else {
          reply = '0';  
        }
        break;
      case 's':
        if (sSwitch == 1) {
          reply = '1';
        } else {
          reply = '0';  
        }
        break;
      case 'w':
        if (wSwitch == 1) {
          reply = '1';
        } else {
          reply = '0';  
        }
        break; 
      case 'p': // ask whether temperature is positive
          reply = isPositive;
        break;
      case 'i': // ask for integer temperature in celsius
          reply = tempDeg;
        break;        
      case 'd': // the 1/10 of a degree is requested here
          reply = decTemp;
        break;           
    } 
  }
}

//--------------------------------------------------------------

ISR(SPI_STC_vect) { // SPI interrupt routine
byte c = SPDR;  // grab byte from SPI Data Register

  SPDR=reply;
  readCommand=c;
  process_it = true;  
}    
