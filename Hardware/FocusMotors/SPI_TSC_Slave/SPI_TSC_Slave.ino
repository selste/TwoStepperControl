// arduino sketch for driving the auxiliary focuser drives of TSC - runs with the hardware from Motorboard.fzz. W. Birkfellner 2017
#include <SPI.h>
#include <AccelStepper.h>
#include <stdlib.h>

AccelStepper aux1Stepper(AccelStepper::DRIVER,3,2); // pin 3 connected to STEP, pin 2 connected to DIR
AccelStepper aux2Stepper(AccelStepper::DRIVER,5,4); // pin 5 connected to STEP, pin 4 connected to DIR
char readCommand;
short sign;
struct kinematicParametersStruct {
  long steps;
  long maxSpeed;
  long acceleration;
  bool isActive;
};

struct kinematicParametersStruct aux1DriveParams; 
struct kinematicParametersStruct aux2DriveParams;

char buf[32];
volatile byte pos;
volatile boolean process_it;
const bool showDebug = false; // if set to false, serial output is supressed ... good for performance
const char whatDriver = 'D'; 

//--------------------------------------------------------------

void setup (void) {
  if (showDebug == true) {
    Serial.begin (115200); 
  }
  pinMode(8,OUTPUT); // connected to M0
  pinMode(7,OUTPUT); // connected to M1 
  pinMode(6,OUTPUT); // connected to M2 of the drv 8825 - sets microstepping for both drives
  digitalWrite(8,HIGH);
  digitalWrite(7,HIGH);    
  digitalWrite(6,HIGH); // LLL=full, HLL=half,LHL=1/4,HHL=1/8,LLH=1/16,HLH=LHH=HHH=1/32 for the DRV8825
  pinMode(A0,OUTPUT); // connected to ENABLE pin of drive 1
  pinMode(A1,OUTPUT); // connected to ENABLE pin of drive 2
  digitalWrite(A0,HIGH);
  digitalWrite(A1,HIGH);
  
  aux1DriveParams.steps = 5000;
  aux1DriveParams.maxSpeed = 500;
  aux1DriveParams.acceleration = 500; 
  aux1DriveParams.isActive = false; 
  aux2DriveParams.steps = 5000;
  aux2DriveParams.maxSpeed = 500;
  aux2DriveParams.acceleration = 500;
  aux2DriveParams.isActive = false;   
  aux1Stepper.setMaxSpeed(aux1DriveParams.maxSpeed); 
  aux1Stepper.setAcceleration(aux1DriveParams.acceleration); 
  aux2Stepper.setMaxSpeed(aux2DriveParams.maxSpeed); 
  aux2Stepper.setAcceleration(aux2DriveParams.acceleration); 
  
  pinMode(MISO, OUTPUT); // have to send on master in, *slave out*
  SPCR |= _BV(SPE);  // turn on SPI in slave mode
  pos = 0;   // buffer empty
  process_it = false;
  SPI.attachInterrupt();   // now turn on interrupts
}  

//--------------------------------------------------------------

void loop (void) { // main loop - wait for flag set in interrupt routine
long numVal;
short bCounter;

  aux1Stepper.run();
  aux2Stepper.run();
  if (process_it) { // got a string via SPI 
    pos = 0;
    readCommand=buf[0];
    process_it = false;
    switch (readCommand) {
      case 'e': // enable drives 1 or 2
        enableDrive(buf[1],buf[2]); 
        break;
      case 'a': // set acceleration for drive 1 or 2
        numVal = convertBufToLParam(); 
        setAcc(buf[1],numVal);       
        break;
      case 'm': // set microstepping rate for drive 1 or 2 
        numVal = convertBufToLParam();
        setMicrosteps(numVal);        
        break;
      case 'v': // set velocity for drive 1 or 2
        numVal = convertBufToLParam();
        setVelocity(buf[1],numVal);       
        break;
      case 's': // set steps for drive 1 or 2
        numVal = convertBufToLParam();
        setSteps(buf[1],numVal);       
        break;
      case 't': // if the master wants to know whether there is a slave connected, it sends 't'
        if (showDebug == true) {
          Serial.println("Ping received");
        }        
        break;
      case 'x': // stops drive 1 or 2 
        stopDrive(buf[1]);       
        break;
      case 'o': 
        startDrive(buf[1]);      
        break; 
       case 'd':
         // report whether drive is at rest
        break;
    }
    for (bCounter = 0; bCounter < 32; bCounter++) {
      buf[bCounter]='#';
    }
  }  
  if ((aux1DriveParams.isActive == true) &&  (aux1Stepper.isRunning() == false)) {
    aux1DriveParams.isActive = false;
    enableDrive('0','0');
  }
  if ((aux2DriveParams.isActive == true) &&  (aux2Stepper.isRunning() == false)) {
    aux2DriveParams.isActive = false;
    enableDrive('1','0');
  }
} // end of loop

//--------------------------------------------------------------

ISR(SPI_STC_vect) { // SPI interrupt routine
byte c = SPDR;  // grab byte from SPI Data Register

  SPDR=whatDriver;
  if (pos < sizeof buf) {
    if (c != 0x00) {     
      buf [pos++] = c;
    } else {
      buf [pos++] = 0x00;
      process_it = true;
    }
  }  
}    

//--------------------------------------------------------------

long convertBufToLParam(void) { //makes a long out of the buffer starting from pos 2 ...
  long param;
  char subBuf[29];
  char sChar;
  short bCounter=2;

  do {
    sChar=buf[bCounter];
    subBuf[bCounter-2]=sChar;
    bCounter++; 
  } while (sChar != '\0');
  param = strtol(subBuf,NULL, 10);
  return param;
}

//--------------------------------------------------------------

void enableDrive(char whatDrive, char setEnabled) { // reacts to "exy" where x is 0/1 in dependence of the drive, and y is a boolean
  if (showDebug == true) {
    Serial.println("Enable drive ");
    Serial.print(whatDrive);
    Serial.print(" ");
    Serial.println(setEnabled);
    Serial.println ("--------");
  }
  if (whatDrive == '0') {
    if (setEnabled == '1') {
      digitalWrite(A0,LOW);
    } else {
      digitalWrite(A0,HIGH);
    }
  } else { 
    if (setEnabled == '1') {
      digitalWrite(A1,LOW);
    } else {
      digitalWrite(A1,HIGH);
    }
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
  if (whatDrive == '0') {
    aux1DriveParams.acceleration = value;  
    aux1Stepper.setAcceleration(aux1DriveParams.acceleration); 
  } else {
    aux2DriveParams.acceleration = value;
    aux2Stepper.setAcceleration(aux2DriveParams.acceleration); 
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
  if (whatDrive == '0') {
    aux1DriveParams.maxSpeed = value;  
    aux1Stepper.setMaxSpeed(aux1DriveParams.maxSpeed); 
  } else {
    aux2DriveParams.maxSpeed = value;
    aux2Stepper.setMaxSpeed(aux2DriveParams.maxSpeed); 
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
  if (whatDrive == '0') {
    aux1DriveParams.steps = value;  
  } else {
    aux2DriveParams.steps = value;  
  } 
}

//--------------------------------------------------------------

void setMicrosteps(long value) { // reacts to m xxx where xxx is either 001, 002, 004, 008, 016, 032, 064 or 128. the second space is free as microsteps can only set for both drives.
  if (showDebug == true) {
    Serial.print("Set Microsteps 1/");
    Serial.println(value);
    Serial.println ("--------");
  }
  switch (value) {
    case 1: digitalWrite(8,LOW);
            digitalWrite(7,LOW);    
            digitalWrite(6,LOW);
            break;
    case 2: digitalWrite(8,HIGH);
            digitalWrite(7,LOW);    
            digitalWrite(6,LOW);
            break;
    case 4: digitalWrite(8,LOW);
            digitalWrite(7,HIGH);    
            digitalWrite(6,LOW);
            break;
    case 8: digitalWrite(8,HIGH);
            digitalWrite(7,HIGH);    
            digitalWrite(6,LOW);
            break;            
    case 16: digitalWrite(8,LOW);
            digitalWrite(7,LOW);    
            digitalWrite(6,HIGH);
            break;            
    case 32: digitalWrite(8,HIGH);
            digitalWrite(7,LOW);    
            digitalWrite(6,HIGH);
            break;            
    case 64: digitalWrite(8,LOW);
            digitalWrite(7,HIGH);    
            digitalWrite(6,HIGH);
            break;
    case 128: digitalWrite(8,HIGH);
            digitalWrite(7,HIGH);    
            digitalWrite(6,HIGH);
            break;        
    default: digitalWrite(8,LOW);
            digitalWrite(7,LOW);    
            digitalWrite(6,LOW);
            break;
  }    
}

//--------------------------------------------------------------

void stopDrive(char whatDrive) { // stops drive 1/2 immediately; reacts on 'x'
  if (showDebug == true) {
    Serial.print("Stop Drive #");
    Serial.println((char)whatDrive);
    Serial.println ("--------");
  }
  if (whatDrive == '0') {
    aux1Stepper.stop();
    aux1DriveParams.isActive = false;    
  } else {
    aux2Stepper.stop();
    aux2DriveParams.isActive = false;
  }
}

//--------------------------------------------------------------

void startDrive(char whatDrive) { // sets drive in motion; reacts on 'o'. distance must be set in de- or raDriveParams.steps 
  if (showDebug == true) {
    Serial.print("Start Drive #");
    Serial.println((char)whatDrive);
    Serial.println ("--------");
  }
  if (whatDrive == '0') {
    aux1Stepper.setCurrentPosition(0);
    aux1Stepper.moveTo(aux1DriveParams.steps);
    aux1DriveParams.isActive = true;
  } else {
    aux2Stepper.setCurrentPosition(0);
    aux2Stepper.moveTo(aux2DriveParams.steps);
    aux2DriveParams.isActive = true;
  }
}
