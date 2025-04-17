#include <Wire.h>

int sw1 = 34;
int sw2 = 35;
int vr = 32;

#define SDA_PIN 21
#define SCL_PIN 22

// ที่อยู่ I²C ของ MCP23017 (8 ตัว)
const uint8_t MCP23017_ADDR[8] = {0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27};

// เก็บสถานะ LED ของแต่ละ MCP23017 (16 บิตต่อ 1 ตัว)
uint16_t ledState[8] = {0};

void writeMCP(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void updateLEDs() {
  for (uint8_t i = 0; i < 8; i++) {
    writeMCP(MCP23017_ADDR[i], 0x12, ledState[i] & 0xFF);  // GPIOA
    writeMCP(MCP23017_ADDR[i], 0x13, (ledState[i] >> 8) & 0xFF); // GPIOB
  }
}

void setup() {
  pinMode(sw1, INPUT);
  pinMode(sw2, INPUT);
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);

  // ตั้งค่า MCP23017 ทั้ง 8 ตัวเป็น OUTPUT
  for (uint8_t i = 0; i < 8; i++) {
    writeMCP(MCP23017_ADDR[i], 0x00, 0x00); // IODIRA: ตั้งเป็น OUTPUT
    writeMCP(MCP23017_ADDR[i], 0x01, 0x00); // IODIRB: ตั้งเป็น OUTPUT
  }
}

void loop() {

  while(1){
    if(digitalRead(sw1) == LOW){
      break;
    }
  }

  // ไล่เปิด LED 1 → 128
  for (uint8_t led = 0; led < 128; led++) {
    int val = map(analogRead(vr), 0, 4095, 0, 1000);
    Serial.println(val);

    uint8_t chipIndex = led / 16;  // เลือก MCP23017 (0 - 7)
    uint8_t bitIndex = led % 16;   // เลือก GPIO ภายใน MCP23017

    ledState[chipIndex] |= (1 << bitIndex); // เปิด LED เพิ่มขึ้นทีละดวง
    updateLEDs();

    delay(val);
  }

  delay(1000); // ค้างไว้ 1 วินาที

  // กระพริบทั้งหมด 3 ครั้ง
  for (int i = 0; i < 3; i++) {
    memset(ledState, 0, sizeof(ledState)); // ปิดทุกดวง
    updateLEDs();
    delay(300); // ไฟดับ 0.3 วินาที

    for (uint8_t i = 0; i < 8; i++) ledState[i] = 0xFFFF; // เปิดทุกดวง
    updateLEDs();
    delay(300); // ไฟติด 0.3 วินาที
  }


  while(1){
    if(digitalRead(sw2) == LOW){
      break;
    }
  } 

  // ไล่ดับ LED 128 → 1
  for (int led = 127; led >= 0; led--) {
    int val = map(analogRead(vr), 0, 4095, 0, 1000);
    Serial.println(val);

    uint8_t chipIndex = led / 16;  // เลือก MCP23017 (0 - 7)
    uint8_t bitIndex = led % 16;   // เลือก GPIO ภายใน MCP23017

    ledState[chipIndex] &= ~(1 << bitIndex); // ปิด LED ทีละดวง
    updateLEDs();

    delay(val);
  }

  delay(1000); // ค้างไว้ 1 วินาที แล้วทำซ้ำ
}
