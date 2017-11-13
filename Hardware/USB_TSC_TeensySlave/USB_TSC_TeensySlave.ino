#include <AccelStepper.h>
#include <stdlib.h>
// #include <Stream.h>

AccelStepper raStepper(AccelStepper::DRIVER,A18,A14); // pin A18 connected to STEP, pin A_14 connected to DIR
AccelStepper deStepper(AccelStepper::DRIVER,A19,A15); // pin A19 connected to STEP, pin A15 connected to DIR
AccelStepper aux1Stepper(AccelStepper::DRIVER,A15,A14); // pin A21 connected to STEP, pin A17 connected to DIR
AccelStepper aux2Stepper(AccelStepper::DRIVER,A17,A16); // pin A20 connected to STEP, pin A16 connected to DIR
char readCommand;
short sign;
struct kinematicParametersStruct {
  long steps;
  long maxSpeed;
  long acceleration;
  bool isActive;
};

struct kinematicParametersStruct raDriveParams; 
struct kinematicParametersStruct deDriveParams;
struct kinematicParametersStruct aux1Params;
struct kinematicParametersStruct aux2Params;

String buf = String(64);
const bool showDebug = true; // if set to false, serial output is supressed ... good for performance
const char whatMainDriver = 'R'; // the DRV 8825 or A 4998 (= 'D') and the RAPS 128 (='R') are 
                                 // supported. insert the letter applicable. RAPS has different logic on the enable pin.
const char whatAuxDriver = 'A';  // focuser and main drives can have different drivers                             

//--------------------------------------------------------------

void setup (void) {   
  if (showDebug == true) {
    Serial.begin (115200); 
  }
  pinMode(A5,OUTPUT); // connected to M0
  pinMode(A4,OUTPUT); // connected to M1 
  pinMode(A3,OUTPUT); // connected to M2 of the drv 8825 - sets microstepping for both main drives
  digitalWrite(A5,HIGH);
  digitalWrite(A4,HIGH);    
  digitalWrite(A3,LOW);  // LLL=full,HLL=half,LHL=1/4,HHL=1/8,LLH=1/16,HLH=LHH=HHH=1/32 for the DRV8825
                         // LLL=full,HLL=half,LHL=1/4,HHL=1/8,HHH= 1/16 for the A4988
                         // LLL=full,HLL=half,LHL=1/4,HHL=1/8,LLH=1/16,HLH=1/32,LHH=1/64,HHH=1/128 for the RAPS    
  pinMode(A22,OUTPUT); // connected to ENABLE pin of main drive in RA
  pinMode(A0,OUTPUT); // connected to ENABLE pin of main drive in Decl
  if (whatMainDriver == 'R') {
    digitalWrite(A22,LOW);
    digitalWrite(A0,LOW); // for the RAPS, both main drives are now disabled ... 
  } else {
    digitalWrite(A22,HIGH);
    digitalWrite(A0,HIGH);
  }
  pinMode(A2,OUTPUT); // connected to ENABLE pin of aux drive 1
  pinMode(A1,OUTPUT); // connected to ENABLE pin of aux drive 2
  if (whatAuxDriver == 'R') {
    digitalWrite(A2,LOW);
    digitalWrite(A1,LOW); // for the RAPS, both main drives are now disabled ... 
  } else {
    digitalWrite(A2,HIGH);
    digitalWrite(A1,HIGH);
  }
   
  raDriveParams.steps = 5000000;
  raDriveParams.maxSpeed = 500;
  raDriveParams.acceleration = 5000; 
  raDriveParams.isActive = false; 
  deDriveParams.steps = 50000;
  deDriveParams.maxSpeed = 500;
  deDriveParams.acceleration = 5000;
  deDriveParams.isActive = false;   
  raStepper.setMaxSpeed(raDriveParams.maxSpeed); 
  raStepper.setAcceleration(raDriveParams.acceleration); 
  deStepper.setMaxSpeed(deDriveParams.maxSpeed); 
  deStepper.setAcceleration(deDriveParams.acceleration); // setting initial parameters for main drives

  aux1Params.steps = 5000;
  aux1Params.maxSpeed = 550;
  aux1Params.acceleration = 500; 
  aux1Params.isActive = false; 
  aux2Params.steps = 5000;
  aux2Params.maxSpeed = 500;
  aux2Params.acceleration = 500;
  aux2Params.isActive = false;   
  aux1Stepper.setMaxSpeed(aux1Params.maxSpeed); 
  aux1Stepper.setAcceleration(aux1Params.acceleration); 
  aux2Stepper.setMaxSpeed(aux2Params.maxSpeed); 
  aux2Stepper.setAcceleration(aux2Params.acceleration); // setting initial parameters for aux drives

  Serial.setTimeout(10); // limit the waiting period for Serial.readString to 10 ms
}  

//--------------------------------------------------------------

void loop (void) { // main loop - wait for flag set in interrupt routine
long numVal;

  raStepper.run();
  deStepper.run();
  aux1Stepper.run();
  aux2Stepper.run();
  if (Serial.available() != 0) {
    buf = Serial.readString();
    switch (buf[0]) {
      case 'e': // enable drives 0/1 (main ra/decl) or 2/3 (aux1/2)
        enableDrive(buf[1],buf[2]); 
        break;
      case 'a': // set acceleration for drive 0/1 (main ra/decl) or 2/3 (aux1/2)
        numVal = convertBufToLParam();
        setAcc(buf[1],numVal);       
        break;
      case 'm': // set microstepping rate for main drives (0) or auxdrives(1)
        numVal = convertBufToLParam();
        setMicrosteps(buf[1],numVal);        
        break;
      case 'v': // set velocity for for drive 0/1 (main ra/decl) or 2/3 (aux1/2)
        numVal = convertBufToLParam();
        setVelocity(buf[1],numVal);      
        break;
      case 's': // set steps for drive for drive 0/1 (main ra/decl) or 2/3 (aux1/2)
        numVal = convertBufToLParam();
        setSteps(buf[1],numVal);     
        break;
      case 't': // if the master wants to know whether there is a device connected, it sends 't'
        if (showDebug == true) {
          Serial.println("Ping received");
        }       
        break;
      case 'x': // stops drive 0/1 (main ra/decl) or 2/3 (aux1/2)
        stopDrive(buf[1]);     
        break;
      case 'o': 
        startDrive(buf[1]); // stops drive 0/1 (main ra/decl) or 2/3 (aux1/2)    
        break; 
       case 'd':
         // report whether drive is at rest
        break;
    }
  }  
  if ((raDriveParams.isActive == true) &&  (raStepper.isRunning() == false)) {
    raDriveParams.isActive = false;
    enableDrive('0','0');
  }
  if ((deDriveParams.isActive == true) &&  (deStepper.isRunning() == false)) {
    deDriveParams.isActive = false;
    enableDrive('1','0');
  }
  if ((aux1Params.isActive == true) &&  (aux1Stepper.isRunning() == false)) {
    aux1Params.isActive = false;
    enableDrive('2','0');
  }
  if ((aux2Params.isActive == true) &&  (aux2Stepper.isRunning() == false)) {
    aux2Params.isActive = false;
    enableDrive('3','0');
  }
} // end of loop
//--------------------------------------------------------------

long convertBufToLParam(void) { //makes a long out of the buffer starting from pos 2 ...
  long param;
  char subBuf[81];
  char sChar;
  short bCounter=2;

  raStepper.run();
  deStepper.run();
  do {
    sChar=buf[bCounter];
    subBuf[bCounter-2]=sChar;
    bCounter++; 
  } while (sChar != '\0');
  param = strtol(subBuf,NULL, 10);
  return param;
}

//--------------------------------------------------------------

void enableDrive(char whatDrive, char setEnabled) { // reacts to "exy" where x is 0/1/2/3 in dependence of the drive, and y is a boolean
  if (showDebug == true) {
    Serial.println("Enable drive ");
    Serial.print(whatDrive);
    Serial.print(" ");
    Serial.println(setEnabled);
    Serial.println ("--------");
  }
    switch (whatDrive) {
      case '0':
        if (whatMainDriver == 'R') {
          if (setEnabled == '1') {
            digitalWrite(A22,HIGH);
          } else {
            digitalWrite(A22,LOW);
          }
        } else {
          if (setEnabled == '1') {
            digitalWrite(A22,LOW);
          } else {
            digitalWrite(A22,HIGH);
          }
        }  // the RAPS driver has an inverted logic on the enable pin ...
      break;
      case '1':
        if (whatMainDriver == 'R') {
          if (setEnabled == '1') {
            digitalWrite(A0,HIGH);
          } else {
            digitalWrite(A0,LOW);
          }
        } else {
          if (setEnabled == '1') {
            digitalWrite(A0,LOW);
          } else {
            digitalWrite(A0,HIGH);
          }          
        }
      break;
      case '2':
        if (whatAuxDriver == 'R') {
          if (setEnabled == '1') {
            digitalWrite(A2,HIGH);
          } else {
            digitalWrite(A2,LOW);
          }
        } else {
          if (setEnabled == '1') {
            digitalWrite(A2,LOW);
          } else {
            digitalWrite(A2,HIGH);
          }
        }
        break;
      default:
        if (whatAuxDriver == 'R') {
          if (setEnabled == '1') {
            digitalWrite(A1,HIGH);
          } else {
            digitalWrite(A1,LOW);
          }
        } else {
           if (setEnabled == '1') {
            digitalWrite(A1,LOW);
          } else {
            digitalWrite(A1,HIGH);
          }         
        }
        break;
    }
}
  
//--------------------------------------------------------------

void setAcc(char whatDrive, long value) { // reacts to axy, where x is 0/1 - the drive, ynd y is a string representing a long - the acceleration in microsteps/(s*s)
  if (showDebug == true) {
    Serial.print("Set Acceleration ");
    Serial.print(whatDrive);
    Serial.print(" ");
    Serial.println(value);
    Serial.println ("--------");
  }
  switch (whatDrive) {
    case '0':
      raDriveParams.acceleration = value;  
      raStepper.setAcceleration(raDriveParams.acceleration); 
      break;
    case '1':
      deDriveParams.acceleration = value;
      deStepper.setAcceleration(deDriveParams.acceleration); 
      break;
    case '2':
      aux1Params.acceleration = value;
      aux1Stepper.setAcceleration(aux1Params.acceleration); 
      break;
    default:
      aux2Params.acceleration = value;
      aux2Stepper.setAcceleration(aux2Params.acceleration); 
      break;
  }
}

//--------------------------------------------------------------

void setVelocity(char whatDrive, long value) {
  if (showDebug == true) {
    Serial.print("Set Velocity ");
    Serial.print(whatDrive);
    Serial.print(" ");
    Serial.println(value);
    Serial.println ("--------"); 
  }
  switch (whatDrive) {
    case '0':
    raDriveParams.maxSpeed = value;  
    raStepper.setMaxSpeed(raDriveParams.maxSpeed); 
      break;
    case '1':
      deDriveParams.maxSpeed = value;
      deStepper.setMaxSpeed(deDriveParams.maxSpeed);
      break;
    case '2':
      aux1Params.maxSpeed = value;
      aux1Stepper.setMaxSpeed(aux1Params.maxSpeed);
      break;
    default:
      aux2Params.maxSpeed = value;
      aux2Stepper.setMaxSpeed(aux2Params.maxSpeed);
      break;
  }
}

//--------------------------------------------------------------

void setSteps(char whatDrive, long value) {
  if (showDebug == true) {
    Serial.print("Set Number Of Steps ");
    Serial.print(whatDrive);
    Serial.print(" ");
    Serial.println(value);
    Serial.println ("--------"); 
  }
  switch (whatDrive) {
    case '0':
      raDriveParams.steps = value; 
      break;
    case '1':
      deDriveParams.steps = value; 
      break;
    case '2':
      aux1Params.steps = value; 
      break;
    default:
      aux2Params.steps = value; 
      break;
  }
}

//--------------------------------------------------------------

void setMicrosteps(char whatDriveGroup, long value) { // reacts to m xxx where xxx is either 001, 002, 004, 008, 016, 032, 064 or 128. the second space is free as microsteps can only set for both drives.
  if (showDebug == true) {
    Serial.print("Set Microsteps 1/");
    Serial.println(value);
    Serial.println ("--------");
  }
  if (whatDriveGroup == '0') {
    switch (value) {
      case 1: digitalWrite(A5,LOW);
              digitalWrite(A4,LOW);    
              digitalWrite(A3,LOW);
              break;
      case 2: digitalWrite(A5,HIGH);
              digitalWrite(A4,LOW);    
              digitalWrite(A3,LOW);
              break;
      case 4: digitalWrite(A5,LOW);
              digitalWrite(A4,HIGH);    
              digitalWrite(A3,LOW);
              break;
      case 8: digitalWrite(A5,HIGH);
              digitalWrite(A4,HIGH);    
              digitalWrite(A3,LOW);
              break;              
      case 16: if (whatMainDriver == 'A') {
                digitalWrite(A5,HIGH);
                digitalWrite(A4,HIGH);    
                digitalWrite(A3,HIGH);
              } else {
                digitalWrite(A5,LOW);
                digitalWrite(A4,LOW);    
                digitalWrite(A3,HIGH);
              }
              break;            
      case 32: digitalWrite(A5,HIGH);
              digitalWrite(A4,LOW);    
              digitalWrite(A13,HIGH);
              break;            
      case 64: digitalWrite(A5,LOW);
              digitalWrite(A4,HIGH);    
              digitalWrite(A3,HIGH);
              break;
      case 128: digitalWrite(A5,HIGH);
              digitalWrite(A4,HIGH);    
              digitalWrite(A3,HIGH);
            break;        
      default: digitalWrite(A5,LOW);
              digitalWrite(A4,LOW);    
              digitalWrite(A3,LOW);
            break;
    }
  } else {
    switch (value) {
      case 1: digitalWrite(A8,LOW);
              digitalWrite(A7,LOW);    
              digitalWrite(A6,LOW);
              break;
      case 2: digitalWrite(A8,HIGH);
              digitalWrite(A7,LOW);    
              digitalWrite(A6,LOW);
              break;
      case 4: digitalWrite(A8,LOW);
              digitalWrite(A7,HIGH);    
              digitalWrite(A6,LOW);
              break;
      case 8: digitalWrite(A8,HIGH);
              digitalWrite(A7,HIGH);    
              digitalWrite(A6,LOW);
              break;            
      case 16:  if (whatMainDriver == 'A') {
                digitalWrite(A8,HIGH);
                digitalWrite(A7,HIGH);    
                digitalWrite(A6,HIGH);
              } else {
                digitalWrite(A8,LOW);
                digitalWrite(A7,LOW);    
                digitalWrite(A6,HIGH);
              }
              break;            
      case 32: digitalWrite(A8,HIGH);
              digitalWrite(A7,LOW);    
              digitalWrite(A6,HIGH);
              break;            
      case 64: digitalWrite(A8,LOW);
              digitalWrite(A7,HIGH);    
              digitalWrite(A6,HIGH);
              break;
      case 128: digitalWrite(A8,HIGH);
              digitalWrite(A7,HIGH);    
              digitalWrite(A6,HIGH);
            break;        
      default: digitalWrite(A8,LOW);
              digitalWrite(A7,LOW);    
              digitalWrite(A6,LOW);
            break;  
    } 
  }
}

//--------------------------------------------------------------

void stopDrive(char whatDrive) { // stops drive 1/2 immediately; reacts on 'x'
  if (showDebug == true) {
    Serial.print("Stop Drive #");
    Serial.println((char)whatDrive);
    Serial.println ("--------");
  }
  switch (whatDrive) {
    case '0':
      raStepper.stop();
      raDriveParams.isActive = false; 
      break;
    case '1':
      deStepper.stop();
      deDriveParams.isActive = false;  
      break;
    case '2':
      aux1Stepper.stop();
      aux1Params.isActive = false;  
      break;
    default:
      aux2Stepper.stop();
      aux2Params.isActive = false;   
      break;
  }
}

//--------------------------------------------------------------

void startDrive(char whatDrive) { // sets drive in motion; reacts on 'o'. distance must be set in de- or raDriveParams.steps 
  if (showDebug == true) {
    Serial.print("Start Drive #");
    Serial.println((char)whatDrive);
    Serial.println ("--------");
  }
  switch (whatDrive) {
    case '0':
      raStepper.setCurrentPosition(0);
      raStepper.moveTo(raDriveParams.steps);
      raDriveParams.isActive = true;
      break;
    case '1':
      deStepper.setCurrentPosition(0);
      deStepper.moveTo(deDriveParams.steps);
      deDriveParams.isActive = true;
      break;
    case '2':
      aux1Stepper.setCurrentPosition(0);
      aux1Stepper.moveTo(aux1Params.steps);
      aux1Params.isActive = true; 
      break;
    default:
      aux2Stepper.setCurrentPosition(0);
      aux2Stepper.moveTo(aux2Params.steps);
      aux2Params.isActive = true;    
      break;
  }
}
