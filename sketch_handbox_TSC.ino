#include <SoftwareSerial.h>
int nSwitch = LOW; // move north button
int eSwitch = LOW; 
int sSwitch = LOW; 
int wSwitch = LOW;
int speedSwitch; 
int switchStateChanged = 0;
SoftwareSerial tscHB(7,8); //RX, TX
  
void setup() {
  // put your setup code here, to run once:
  pinMode(5,INPUT); // north
  pinMode(3,INPUT); // east
  pinMode(2,INPUT); // south
  pinMode(4,INPUT); // west
  pinMode(6,INPUT); // speed selection switch
  if (digitalRead(6) == LOW) {
    speedSwitch=LOW;
  } else {
    speedSwitch=HIGH;
  }
  Serial.begin(9600);
  while (!Serial) {}
  tscHB.begin(9600);
//  while (!tscHB) {}
}

void loop() {
if (digitalRead(5) != nSwitch) {
  nSwitch = digitalRead(2);
  switchStateChanged=1;
}
if (digitalRead(3) != eSwitch) {
  eSwitch = digitalRead(3);
  switchStateChanged=1;
}
if (digitalRead(2) != sSwitch) {
  sSwitch = digitalRead(5);
  switchStateChanged=1;
}
if (digitalRead(4) != wSwitch) {
  wSwitch = digitalRead(4);
  switchStateChanged=1;
}
if (digitalRead(6) != speedSwitch) {
  speedSwitch = digitalRead(6);
  switchStateChanged=1;
}
if (switchStateChanged == 1) {
  Serial.print(nSwitch);
  Serial.print(eSwitch);
  Serial.print(sSwitch);
  Serial.print(wSwitch);
  Serial.println(speedSwitch);
  tscHB.print(nSwitch);
  tscHB.print(eSwitch);
  tscHB.print(sSwitch);
  tscHB.print(wSwitch);
  tscHB.println(speedSwitch);
}
delay(50);
switchStateChanged=0;
}
