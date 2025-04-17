int led = 16;

void setup() {
  pinMode(led, OUTPUT);
}

void loop() {
 mark2(10);  //จำนวนรอบ
 delay(3000);
}

void mark(int a) {
  for (int i = 1; i <= a; i++) {
    digitalWrite(led,HIGH);
    delay(200);
      digitalWrite(led,LOW);
    delay(200);
  }
}

void mark2(int a) {
  for (int i = a; i >= 0; i--) {
    digitalWrite(led,HIGH);
    delay(200);
      digitalWrite(led,LOW);
    delay(200);
  }
}