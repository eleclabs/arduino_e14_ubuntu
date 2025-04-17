int led = 2;
int vr = 4;

void setup() {
  Serial.begin(115200);
  pinMode(led,OUTPUT);

}

void loop() {
  int var = analogRead(vr);
  Serial.println(var);
  digitalWrite(led, HIGH);
  delay(var);
  digitalWrite(led, LOW);
  delay(var);

}
