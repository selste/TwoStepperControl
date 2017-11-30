#include <AccelStepper.h>
AccelStepper raStepper(AccelStepper::DRIVER,3,2); // pin 3 connected to STEP, pin 2 connected to DIR
AccelStepper deStepper(AccelStepper::DRIVER,5,4); // pin 5 connected to STEP, pin 4 connected to DIR

struct kinematicParametersStruct {
  long steps;
  long maxSpeed;
  long acceleration;
};

struct kinematicParametersStruct raDriveParams; 
struct kinematicParametersStruct deDriveParams;

//--------------------------------------------------------

void setup() {  
   pinMode(8,OUTPUT); // connected to M0
   pinMode(7,OUTPUT); // connected to M1 
   pinMode(6,OUTPUT); // connected to M2 of the drv 8825 - sets microstepping for both drives
   pinMode(A0, OUTPUT);
   pinMode(A1,OUTPUT);
   digitalWrite(A0, LOW);
   digitalWrite(A1, LOW);
   digitalWrite(8,HIGH);
   digitalWrite(7,LOW);    
   digitalWrite(6,LOW); // LLL=full, HLL=half,LHL=1/4,HHL=1/8,LLH=1/16,HLH=LHH=HHH=1/32 for the DRV8825
   
   raDriveParams.steps = 3000;
   raDriveParams.maxSpeed = 300;
   raDriveParams.acceleration = 300;
   deDriveParams.steps = 3000;
   deDriveParams.maxSpeed = 300;
   deDriveParams.acceleration = 300;
   
   raStepper.setMaxSpeed(raDriveParams.maxSpeed); 
   raStepper.setAcceleration(raDriveParams.acceleration); 
   raStepper.moveTo(raDriveParams.steps);
   deStepper.setMaxSpeed(deDriveParams.maxSpeed); 
   deStepper.setAcceleration(deDriveParams.acceleration); 
   deStepper.moveTo(deDriveParams.steps);
}

//---------------------------------------------------------

void loop() {  
  raStepper.run();
  deStepper.run();
  if (raStepper.isRunning() == false) {
    raDriveParams.steps *= -1;
    raStepper.moveTo(raDriveParams.steps); 
  }
  if (deStepper.isRunning() == false) {
    deDriveParams.steps *= -1;
    deStepper.moveTo(deDriveParams.steps); 
  }
}
