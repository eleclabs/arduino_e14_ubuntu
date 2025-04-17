int led = 25;

void setup() {
  Serial.begin(115200);
  pinMode(led, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    char ch = Serial.read();
    Serial.println(ch);
    if (ch == 'w') {
      digitalWrite(led, HIGH);
      delay(200);
      digitalWrite(led, LOW);
      delay(200);
    } else if (ch == 'a') {
      for (int i = 0; i < 5; i++) {
        digitalWrite(led, HIGH);
        delay(200);
        digitalWrite(led, LOW);
        delay(200);
      }
    }
  }
}

