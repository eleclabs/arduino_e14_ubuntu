int led = 16;

void setup() {
  Serial.begin(9600);
  pinMode(led, OUTPUT);

}

void loop() {
  int i = 1;
  while ( i<=10) {
    Serial.println(i);
    digitalWrite(led,HIGH);
    delay(500);
    digitalWrite(led,LOW);
    delay(500); 
    i++;
  }
delay(3000);
  
}
