const int pinX1 = A3;
const int pinX2 = A2;
const int pinX3 = A1;

const int pinY1 = A9;
const int pinY2 = A8;
const int pinY3 = A7;

const int pinZ1 = A6;
const int pinZ2 = A10;
const int pinZ3 = A0;

void setup() {
  pinMode(pinX1, INPUT);
  pinMode(pinX2, INPUT);
  pinMode(pinX3, INPUT);
  pinMode(pinY1, INPUT);
  pinMode(pinY2, INPUT);
  pinMode(pinY3, INPUT);
  pinMode(pinZ1, INPUT);
  pinMode(pinZ2, INPUT);
  pinMode(pinZ3, INPUT);
  Serial.begin(9600);
}
 
void loop() {
  int X1 = analogRead(pinX1);
  int X2 = analogRead(pinX2);
  int X3 = analogRead(pinX3);
  int Y1 = analogRead(pinY1);
  int Y2 = analogRead(pinY2);
  int Y3 = analogRead(pinY3);
  int Z1 = analogRead(pinZ1);
  int Z2 = analogRead(pinZ2);
  int Z3 = analogRead(pinZ3);
  Serial.print(X1);
  Serial.print("_");
  Serial.print(X2);
  Serial.print("_");
  Serial.print(X3);
  Serial.print("_");
  Serial.print(Y1);
  Serial.print("_");
  Serial.print(Y2);
  Serial.print("_");
  Serial.print(Y3);
  Serial.print("_");
  Serial.print(Z1);
  Serial.print("_");
  Serial.print(Z2);
  Serial.print("_");
  Serial.println(Z3);
  delay(10);
}
