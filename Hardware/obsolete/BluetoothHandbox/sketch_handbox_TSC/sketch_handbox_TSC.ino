 // arduino sketch for the wireless handbox of TSC. W. Birkfellner 2017
#include <SoftwareSerial.h>
int nSwitch = LOW; // move north button
int eSwitch = LOW; 
int sSwitch = LOW; 
int wSwitch = LOW;
int ffull = LOW;
int f5th = LOW;
int f20th = LOW;
int speedSwitch;
int fSelect;
int fFwd;
int switchStateChanged = 0;
SoftwareSerial tscHB(7,8); //RX, TX
  
void setup() {
  // put your setup code here, to run once:
  pinMode(4,INPUT); // north
  pinMode(5,INPUT); // east
  pinMode(3,INPUT); // south
  pinMode(2,INPUT); // west
  pinMode(A0, INPUT); // full focuser step
  pinMode(7, INPUT); // 1/5 focuser step
  pinMode(6, INPUT); // 1/20 focuser step
  pinMode(A3,INPUT); // speed selection switch 
  pinMode(A2, INPUT); // select focuser drive
  pinMode(A1, INPUT); // select focuser direction
  if (digitalRead(A3) == LOW) {
    speedSwitch=LOW;
  } else {
    speedSwitch=HIGH;
  }
  if (digitalRead(A2) == LOW) {
    fSelect=LOW;
  } else {
    fSelect=HIGH;
  }
  if (digitalRead(A1) == LOW) {
    fFwd=LOW;
  } else {
    fFwd=HIGH;
  }
  
  Serial.begin(9600);
  while (!Serial) {}
  tscHB.begin(9600);
//  while (!tscHB) {}
}

void loop() {
if (digitalRead(4) != nSwitch) {
  nSwitch = digitalRead(4);
  switchStateChanged=1;
}
if (digitalRead(5) != eSwitch) {
  eSwitch = digitalRead(5);
  switchStateChanged=1;
}
if (digitalRead(3) != sSwitch) {
  sSwitch = digitalRead(3);
  switchStateChanged=1;
}
if (digitalRead(2) != wSwitch) {
  wSwitch = digitalRead(2);
  switchStateChanged=1;
}
if (digitalRead(A3) != speedSwitch) {
  speedSwitch = digitalRead(A3);
  switchStateChanged=1;
}
if (digitalRead(A2) != fSelect) {
  fSelect = digitalRead(A2);
  switchStateChanged=1;
}
if (digitalRead(A1) != fFwd) {
  fFwd = digitalRead(A1);
  switchStateChanged=1;
}
if (digitalRead(A0) != ffull) {
  ffull = digitalRead(A0);
  switchStateChanged=1;
}
if (digitalRead(7) != f5th) {
  f5th = digitalRead(7);
  switchStateChanged=1;
}
if (digitalRead(6) != f20th) {
  f20th = digitalRead(6);
  switchStateChanged=1;
}

if (switchStateChanged == 1) {
  Serial.print(nSwitch);
  Serial.print(eSwitch);
  Serial.print(sSwitch);
  Serial.print(wSwitch);
  Serial.print(speedSwitch);
  Serial.print(fSelect);
  Serial.print(fFwd);
  Serial.print(ffull);
  Serial.print(f5th);
  Serial.println(f20th);
  tscHB.print(nSwitch);
  tscHB.print(eSwitch);
  tscHB.print(sSwitch);
  tscHB.print(wSwitch);
  tscHB.print(speedSwitch);
  tscHB.print(fSelect);
  tscHB.print(fFwd);
  tscHB.print(ffull);
  tscHB.print(f5th);
  tscHB.println(f20th);
}
delay(50);
switchStateChanged=0;
}
