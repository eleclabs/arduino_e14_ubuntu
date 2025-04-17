int led[] = { 16, 17, 18, 19 };
void setup() {
  setpin();
}

void loop() {
  blinkF();
  blinkR();
  aaa();
}

void setpin() {
  for (int i = 0; i <= 3; i++) {
    pinMode(led[i], OUTPUT);
  }
}

void blinkF() {
  for (int i = 0; i <= 3; i++) {
    digitalWrite(led[i], HIGH);
    delay(200);
    digitalWrite(led[i], LOW);
    delay(200);
  }
}

void blinkR() {
  for (int i = 2; i >= 0; i--) {
    digitalWrite(led[i], HIGH);
    delay(200);
    digitalWrite(led[i], LOW);
    delay(200);
  }
}

void aaa() {
  digitalWrite(led[0], HIGH);
  digitalWrite(led[3], HIGH);
  digitalWrite(led[1], LOW);
  digitalWrite(led[2], LOW);
  delay(200);
  digitalWrite(led[0], LOW);
  digitalWrite(led[3], LOW);
  digitalWrite(led[1], HIGH);
  digitalWrite(led[2], HIGH);
  delay(200);
  digitalWrite(led[0], LOW);
  digitalWrite(led[3], LOW);
  digitalWrite(led[1], LOW);
  digitalWrite(led[2], LOW);
  delay(200);
}
