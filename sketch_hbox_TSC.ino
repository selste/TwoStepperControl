int nSwitch = LOW; // move north button
int eSwitch = LOW; 
int sSwitch = LOW; 
int wSwitch = LOW; 
int switchStateChanged = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(2,INPUT); // north
  pinMode(3,INPUT); // east
  pinMode(4,INPUT); // south
  pinMode(5,INPUT); // west
  Serial.begin(9600);
}

void loop() {
if (digitalRead(2) != nSwitch) {
  nSwitch = digitalRead(2);
  switchStateChanged=1;
}
if (digitalRead(3) != eSwitch) {
  eSwitch = digitalRead(3);
  switchStateChanged=1;
}
if (digitalRead(4) != sSwitch) {
  sSwitch = digitalRead(4);
  switchStateChanged=1;
}
if (digitalRead(5) != wSwitch) {
  wSwitch = digitalRead(5);
  switchStateChanged=1;
}
if (switchStateChanged == 1) {
  Serial.print(nSwitch);
  Serial.print(eSwitch);
  Serial.print(sSwitch);
  Serial.println(wSwitch);
}
delay(10);
switchStateChanged=0;
}
