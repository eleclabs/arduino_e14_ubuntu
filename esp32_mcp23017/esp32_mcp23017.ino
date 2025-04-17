#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22

// กำหนด GPIO ที่ใช้ควบคุม A0, A1, A2
#define A0_PIN 14
#define A1_PIN 12
#define A2_PIN 13

void setMCPAddress(bool a2, bool a1, bool a0) {
  digitalWrite(A2_PIN, a2);
  digitalWrite(A1_PIN, a1);
  digitalWrite(A0_PIN, a0);
  delay(10);  // รอให้สัญญาณนิ่ง
}

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);

  // กำหนดให้ขา A0, A1, A2 เป็น OUTPUT
  pinMode(A0_PIN, OUTPUT);
  pinMode(A1_PIN, OUTPUT);
  pinMode(A2_PIN, OUTPUT);

  // กำหนดที่อยู่ I²C เป็น 0x21 (A0=1, A1=0, A2=0)
  setMCPAddress(LOW, LOW, HIGH);

  // ตั้งค่า MCP23017 เป็น OUTPUT (ทุกขา)
  Wire.beginTransmission(0x21);
  Wire.write(0x00); // IODIRA register
  Wire.write(0x00); // ตั้งทุกขาเป็น OUTPUT
  Wire.endTransmission();
}

void loop() {
  // เปิดไฟ LED บน MCP23017
  Wire.beginTransmission(0x21);
  Wire.write(0x12); // GPIOA register
  Wire.write(0xFF); // เปิดทุกขา (11111111)
  Wire.endTransmission();
  delay(200);

  // ปิดไฟ LED บน MCP23017
  Wire.beginTransmission(0x21);
  Wire.write(0x12);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(200);
}
