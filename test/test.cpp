// ทดสอบ IR ตรวจพื้นกับ ESP32
const int IR_PIN = 13;  // เปลี่ยนให้ตรงกับขาที่ต่อจริง

void setup() {
  Serial.begin(115200);
  pinMode(IR_PIN, INPUT);

  Serial.println("เริ่มทดสอบ IR ตรวจพื้น");
}

void loop() {
  int value = digitalRead(IR_PIN);

  Serial.print("IR = ");
  Serial.println(value);

  delay(200);
}