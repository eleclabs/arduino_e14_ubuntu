int a0 = a0;
int a1 = a1;
int a2 = a2;


void setup() {
  pinMode(a0, OUTPUT);
  pinMode(a1, OUTPUT);
  pinMode(a2, OUTPUT);

}

void loop() {
  digitalWrite(a0, HIGH);
  digitalWrite(a1, HIGH);
  digitalWrite(a2, HIGH);
  delay(1000);
  digitalWrite(a0, LOW);
  digitalWrite(a1, LOW);
  digitalWrite(a2, LOW);
  delay(1000);

}
