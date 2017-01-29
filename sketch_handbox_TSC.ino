#include <SoftwareSerial.h>
int nSwitch = LOW; // move north button
int eSwitch = LOW; 
int sSwitch = LOW; 
int wSwitch = LOW; 
int switchStateChanged = 0;
SoftwareSerial tscHB(10,11); //RX, TX
  
void setup() {
  // put your setup code here, to run once:
  pinMode(2,INPUT); // north
  pinMode(4,INPUT); // east
  pinMode(7,INPUT); // south
  pinMode(12,INPUT); // west
  Serial.begin(9600);
  while (!Serial) {}
  tscHB.begin(9600);
  while (!tscHB) {}
}

void loop() {
if (digitalRead(2) != nSwitch) {
  nSwitch = digitalRead(2);
  switchStateChanged=1;
}
if (digitalRead(4) != eSwitch) {
  eSwitch = digitalRead(4);
  switchStateChanged=1;
}
if (digitalRead(7) != sSwitch) {
  sSwitch = digitalRead(7);
  switchStateChanged=1;
}
if (digitalRead(12) != wSwitch) {
  wSwitch = digitalRead(12);
  switchStateChanged=1;
}
if (switchStateChanged == 1) {
  Serial.print(nSwitch);
  Serial.print(eSwitch);
  Serial.print(sSwitch);
  Serial.println(wSwitch);
  tscHB.print(nSwitch);
  tscHB.print(eSwitch);
  tscHB.print(sSwitch);
  tscHB.println(wSwitch);
}
delay(50);
switchStateChanged=0;
}
