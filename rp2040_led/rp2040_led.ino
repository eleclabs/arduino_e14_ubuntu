int LED[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };

void setup() {
  Serial.begin(115200);

  for (int i = 0; i <= 15; i++) {
    pinMode(LED[i], OUTPUT);
  }

}

void loop() {
  for(int i = 0; i <= 15; i++){
    digitalWrite(LED[i], HIGH);
    delay(200);
    digitalWrite(LED[i], LOW);
    delay(200);
    Serial.println(i);
  }
}
