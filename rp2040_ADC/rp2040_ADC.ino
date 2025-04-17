int vr = 26 ;
int led = 16 ;
void setup() {
 Serial.begin(9600);
pinMode(led,OUTPUT);
}

void loop() {
  int ADC = analogRead(vr);
  Serial.println(ADC);
  digitalWrite(led,HIGH);
  delay(ADC);
  digitalWrite(led,LOW);
  delay(ADC);
}
