#include <AccelStepper.h>

AccelStepper deStepper(AccelStepper::DRIVER,A4,A5);
AccelStepper raStepper(AccelStepper::DRIVER,A7,A8);
AccelStepper aux1Stepper(AccelStepper::DRIVER,A21,A22);
AccelStepper aux2Stepper(AccelStepper::DRIVER,A18,A19);


void setup() {
   pinMode(A0,OUTPUT);// enable de
   digitalWrite(A0,HIGH);
   pinMode(A6,OUTPUT);// enable ra
   digitalWrite(A0,HIGH);
   pinMode(A14,OUTPUT);// enable aux1
   digitalWrite(A14,HIGH);
   pinMode(A20,OUTPUT); //enable aux2
   digitalWrite(A20,HIGH);

   pinMode(A1,OUTPUT); // connected to M0
   pinMode(A2,OUTPUT); // connected to M1 
   pinMode(A3, OUTPUT);// connected to M2 of the drv 8825 - sets microstepping for both main drives
   digitalWrite(A1,HIGH);
   digitalWrite(A2,HIGH);
   digitalWrite(A3,LOW); // LLL=full, HLL=half,LHL=1/4,HHL=1/8,LLH=1/16,HLH=LHH=HHH=1/32 for the DRV8825  
   
   pinMode(A15,OUTPUT); // connected to M0
   pinMode(A16,OUTPUT); // connected to M1 
   pinMode(A17, OUTPUT);// connected to M2 of the drv 8825 - sets microstepping for both aux drives
   digitalWrite(A15,HIGH);
   digitalWrite(A16,HIGH);
   digitalWrite(A17,LOW); // LLL=full, HLL=half,LHL=1/4,HHL=1/8,LLH=1/16,HLH=LHH=HHH=1/32 for the DRV8825    

   deStepper.setMaxSpeed(3000); 
   deStepper.setAcceleration(5000); 
   deStepper.setCurrentPosition(0);
   digitalWrite(A0,LOW);
   deStepper.moveTo(50000);
   
   raStepper.setMaxSpeed(3000); 
   raStepper.setAcceleration(5000); 
   raStepper.setCurrentPosition(0);
   digitalWrite(A6,LOW);
   raStepper.moveTo(50000);
   
   aux1Stepper.setMaxSpeed(3000); 
   aux1Stepper.setAcceleration(5000); 
   aux1Stepper.setCurrentPosition(0);
   digitalWrite(A14,LOW);
   aux1Stepper.moveTo(50000);
   
   aux2Stepper.setMaxSpeed(3000); 
   aux2Stepper.setAcceleration(5000); 
   aux2Stepper.setCurrentPosition(0);
   digitalWrite(A20,LOW);
   aux2Stepper.moveTo(50000);
}

void loop() {
  deStepper.run();
  raStepper.run();
  aux1Stepper.run();
  aux2Stepper.run();
  if (aux2Stepper.isRunning() == false) {
     digitalWrite(A20,HIGH);  
  }
  if (aux1Stepper.isRunning() == false) {
     digitalWrite(A14,HIGH);  
  }
    if (deStepper.isRunning() == false) {
     digitalWrite(A0,HIGH);  
  }
  if (raStepper.isRunning() == false) {
     digitalWrite(A6,HIGH);  
  }

}
