// sketch for controlling an Pololu AMIS 30543 stepper controller board with an Adafruit ItsyBitsyM4 ATMWEGA ATSAMD51 Cortex M4 microcontroller.
// wolfgang birkfellner, 2018. wbirkfellner@gmail.com

#include <SPI.h>
#include <AMIS30543.h>
#include <AccelStepper.h>
#include <stdlib.h>

const uint8_t amisDirPin = 7;
const uint8_t amisStepPin = 12;
const uint8_t amisSlaveSelect = 9;
const uint8_t amisErrPin = 11;
const uint8_t amisSLAPin = A2;
const uint8_t inputVoltagePin = A3;
const uint8_t resetPin = 10;
const bool inputVoltageReadEnabled = false; // set to true if the bridge to A3 is closed
const float inputVoltageCalibrationFactor = 10.889; // the PCB has a voltage divider; set the correct value for converting the read voltage (< 3.3 V) to the input voltage for the motor

AMIS30543 stepper;
AccelStepper accelStepper(AccelStepper::DRIVER, amisStepPin, amisDirPin);

struct kinematicParametersStruct {
  long steps;   // number of steps to be carried out
  long stepsDone; // steps done at the time being
  long maxSpeedInMicrosteps; // maximum speed in msteps/s
  long acceleration; // acceleration in msteps/s
  bool isActive; // flag whether the drive is moving
  long current; // maximum current per coil given in milliAmpere
  short stepMode; // microstepping ratio: 1, 2, 4, 8, 16, 32, 64 or 128 
};

struct kinematicParametersStruct driveParams;
char commandIdentifier; // the command syntax is a character followed by a numerical value
char usbCommand[63]; // command received via usb; maximum size is 64 bytes
char outputString[63]; // a string holding the answer
String outputFloat;

//------------------------------------------------------------
void setup() {
  SPI.begin(); // start SPI for setting up the AMIS board
  Serial.begin(10000000); // communication is native USB - speed is set to maximum 10 Mbit/s should be ok
  stepper.init(amisSlaveSelect); // initialize the AMIS board while giving the SPI slave select line
  delay(1);

  driveParams.steps = 20000; // default parameters
  driveParams.stepsDone = 0;
  driveParams.maxSpeedInMicrosteps = 1000;
  driveParams.acceleration = 1000;
  driveParams.isActive = false;
  driveParams.current = 800;
  driveParams.stepMode = 16;
  
  stepper.resetSettings();
  stepper.setCurrentMilliamps(driveParams.current);
  stepper.setStepMode(driveParams.stepMode); // setting the AMIS parameters via SPI

  accelStepper.setMaxSpeed(driveParams.maxSpeedInMicrosteps);
  accelStepper.setAcceleration(driveParams.acceleration);
  accelStepper.setCurrentPosition(0); // initialising the accelstepper library
}

//------------------------------------------------------------
void loop() {
  long numVal, charsAvailable, chCounter;

  accelStepper.run(); // buffer motion parameters for the stepper
  charsAvailable=Serial.available(); // check USB input
  if (charsAvailable != 0) {  // got a string via USB
    accelStepper.run();
    commandIdentifier=Serial.read(); // the first character is the command identifier
    charsAvailable--;
    if (charsAvailable != 0) {
      for (chCounter = 0; chCounter <= charsAvailable; chCounter++) {
        usbCommand[chCounter]=Serial.read();
      }
      usbCommand[chCounter+1]='\0';
    } // assemble a numerical value from the remainder of the input
    numVal = strtol(usbCommand, NULL, 10);
    accelStepper.run();
    switch (commandIdentifier) {
    case 0x06: // ACK ... responds with an identifier for the drive addressed
      replyWithDriveID(); 
      break;
    case 'a': // set acceleration in msteps/(s*s)
      setAcc(numVal);   
      break;
    case 'c': // set current
      setCurrent(numVal);      
      break; 
    case 'e': // enable the drive; it is automatically activated when an "start drive" command (= 'o') is sent
      enableDrive(numVal); 
      break;
    case 'f':
      reportAMISStates(numVal); // report conditions. 0 = is drive moving, 1 = error on AMIS board reproted, 2 = error on SPI settings, 3 = input voltage for motor, 4 = voltage on SLA pin of the AMIS, 5 = steps carried out at the given time,  
      break;                    // 6 = microstepping ratio, 7 = maximum speed in msteps/s, 8 = acceleration in msteps/(s*s), 9 = maximum current per coil in milliAmpere, 10 = number of steps set
    case 'm': 
      setMicrosteps(numVal);    // set microstepping mode: 1, 2, 4, 8, 16, 32, 64 and 128 are permitted    
      break;
    case 'o': 
      startDrive(); // engage the drive to carry out a defined number of microsteps     
      break;
    case 'r':
      resetAMIS(); // reset the AMIS via its CLR pin
      break;
    case 's': 
      setSteps(numVal); // set the number of steps to be carried out
      break;
    case 'v': 
      setVelocity(numVal); // set maximum speed in msteps/s      
      break;
    case 'x': 
      stopDrive(); // stop drive
      break;
    default:
      break;
    }
    accelStepper.run();
  }

  driveParams.stepsDone = driveParams.steps - accelStepper.distanceToGo(); // update the current position
  accelStepper.run();
  if (accelStepper.isRunning() == false) {   // check if drives are moving - if they just stopped, disable them ...
    driveParams.isActive = false;
    stepper.disableDriver();
  } 
}

//----------------------------------------------------------------------------------
// respond with an identifier for the drive when receiving the <ACK> character

inline void replyWithDriveID(void) {
  Serial.write("TSC_RADR");
}

//----------------------------------------------------------------------------------
// enable the driver without moving it

inline void enableDrive(long enableDrive) {
  if (enableDrive == 1) {
    stepper.enableDriver();
    Serial.write("Stepper enabled");
  } else {
    stepper.disableDriver();    
    Serial.write("Stepper disabled");
  }
}

//-----------------------------------------------------------------------------------
// set acceleration in msteps/(s*s)

inline void setAcc(long acceleration) {

  if ((acceleration > 0) && (acceleration < 100000)) {
    driveParams.acceleration = acceleration;
    accelStepper.setAcceleration(driveParams.acceleration);
    Serial.write("Acceleration set");
  } else {
    Serial.write("Acceleration value not permitted");
  } 
}

//------------------------------------------------------------------------------------
// set microstepping ratio

inline void setMicrosteps(long ratio) {
  
  switch(ratio) {
    case 1: driveParams.stepMode = 1; stepper.setStepMode(driveParams.stepMode); break;
    case 2: driveParams.stepMode = 2; stepper.setStepMode(driveParams.stepMode); break;   
    case 4: driveParams.stepMode = 4; stepper.setStepMode(driveParams.stepMode); break; 
    case 8: driveParams.stepMode = 8; stepper.setStepMode(driveParams.stepMode); break;
    case 16: driveParams.stepMode = 16; stepper.setStepMode(driveParams.stepMode); break; 
    case 32: driveParams.stepMode = 32; stepper.setStepMode(driveParams.stepMode); break;
    case 64: driveParams.stepMode = 64; stepper.setStepMode(driveParams.stepMode); break;
    case 128: driveParams.stepMode = 128; stepper.setStepMode(driveParams.stepMode); break;    
    default: Serial.write("Invalid microstep parameter"); return;
  }
  if (stepper.verifySettings() == true) {
    Serial.write("Microsteps set. AMIS settings ok");  
  } else {
    Serial.write("Microsteps set. Error in AMIS settings");  
  }
}

//------------------------------------------------------------------------------------
// set maximum speed in msteps/s

inline void setVelocity(long sspeed) {
  
  if ((sspeed >= 0) && (sspeed < 100000)) {
    driveParams.maxSpeedInMicrosteps = sspeed;
    accelStepper.setMaxSpeed(driveParams.maxSpeedInMicrosteps);  
    Serial.write("Speed set");
  } else {
    Serial.write("Speed value not permitted");
  } 
}

//-------------------------------------------------------------------------------------
// set number of steps to be carried out. this does not start the drive.

inline void setSteps(long ssteps) {
  
  driveParams.steps = ssteps;
  Serial.write("Steps set");
}

//-------------------------------------------------------------------------------------
// stop the drive: the de-acceleration ramp is carried out

inline void stopDrive(void) {
    accelStepper.stop();
    Serial.write("Drive stopped");
}

//-------------------------------------------------------------------------------------
// start the drive to carry out a defined number of steps

inline void startDrive(void) {
  
  stepper.enableDriver();
  accelStepper.setCurrentPosition(0);
  accelStepper.moveTo(driveParams.steps);
  driveParams.isActive = true;
  Serial.write("Drive started");
}

//-------------------------------------------------------------------------------------
// set the maximum current per coil in milliAmpere; maximum value is 3 A

inline void setCurrent(long curr) {
  
  if ((curr > 10) && (curr < 3000)) {
    driveParams.current = curr;
    stepper.setCurrentMilliamps(driveParams.current);
    if (stepper.verifySettings() == true) {
      Serial.write("Current set. AMIS settings ok");  
    } else {
      Serial.write("Current set. Error in AMIS settings");  
    }
  } else {
    Serial.write("Current value not permitted");
  }
}

//-------------------------------------------------------------------------------------
// report the condition of the driver

inline void reportAMISStates(long what) {
  float analogReading, iVoltage;

  accelStepper.run();
  switch(what) {
  case 0: // report whether drive is moving
    if (driveParams.isActive == true) {
      Serial.write('1');
    } else {
      Serial.write('0');
    }
    break;
  case 1: // report the state of the internal ERR pin of the AMIS
    if (digitalRead(amisErrPin) == HIGH) {
      Serial.write("1"); 
    } else {
      Serial.write("0");
    }
    break;
  case 2: // report whether the settings are correct as set on the AMIS via SPI
    if (stepper.verifySettings() == true) {
      Serial.write("1");  
    } else {
      Serial.write("0");  
    }
    break;
    case 3: // report the input voltage if enabled on the PCB
      if (inputVoltageReadEnabled == true) {
        analogReading = analogRead(inputVoltagePin)/1024.0*3.3;
        iVoltage = analogReading*inputVoltageCalibrationFactor;
        outputFloat=String(iVoltage,2);
        accelStepper.run();
        Serial.write(outputFloat.c_str());    
      } else {
        Serial.write("Voltage not available");
      }
      break;
    case 4: // report the  voltage from the SLA pin
        analogReading = analogRead(amisSLAPin)/1024.0*3.3;
        outputFloat=String(analogReading,3);
        accelStepper.run();
        Serial.write(outputFloat.c_str());    
      break;
    case 5: // report current number of microsteps carried out
        sprintf(outputString,"%d",driveParams.stepsDone);
        accelStepper.run();
        Serial.write(outputString);    
      break;
    case 6: // report the microstepping ratio
      sprintf(outputString,"%d",driveParams.stepMode);
      accelStepper.run();
      Serial.write(outputString);    
      break;
    case 7: // report the maximum speed in microsteps/s
      sprintf(outputString,"%d",driveParams.maxSpeedInMicrosteps);
      accelStepper.run();
      Serial.write(outputString);    
      break;  
    case 8: // report the acceleration in microsteps/(s*s)
      sprintf(outputString,"%d",driveParams.acceleration);
      accelStepper.run();
      Serial.write(outputString);    
      break;  
    case 9: // report the maximum coil current in millAmpere
      sprintf(outputString,"%d",driveParams.current);
      accelStepper.run();
      Serial.write(outputString);    
      break;    
    case 10: // report steps set
     sprintf(outputString,"%d",driveParams.steps);
      accelStepper.run();
      Serial.write(outputString);    
      break;     
    default: 
      accelStepper.run();
      Serial.write("-1");
      break;  
  }
  accelStepper.run();
  outputString[0] = '\0';
}

//--------------------------------------------------------------------------------------
// sets the CLR pin of the AMIS to HI for 0.5 seconds and resets the buffer of the AMIS

inline void resetAMIS(void) { 
  digitalWrite(resetPin, HIGH);
  delay(500);
  digitalWrite(resetPin, LOW);  
  delay(10);
  stepper.resetSettings();
  stepper.setCurrentMilliamps(driveParams.current);
  stepper.setStepMode(driveParams.stepMode);
  Serial.write("1");
}

//---------------------------------------------------------------------------------------



