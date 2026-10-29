#include <Arduino.h>

// เลือก environment: motor_test เพื่อ Upload โปรแกรมนี้
// L298N ต้องใส่จัมเปอร์ ENA/ENB และมีไฟเลี้ยงมอเตอร์
// เปิดเครื่องแล้วสั่งหมุนทันที: ยกล้อพ้นพื้นก่อนทดสอบ
namespace {
constexpr uint8_t LEFT_IN1 = 32;
constexpr uint8_t LEFT_IN2 = 33;
constexpr uint8_t RIGHT_IN1 = 25;
constexpr uint8_t RIGHT_IN2 = 26;
constexpr uint8_t MOTOR_PINS[] = {LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2};

// เปลี่ยนเป็น true หากล้อข้างนั้นหมุนกลับทิศ
constexpr bool LEFT_REVERSED = true;
constexpr bool RIGHT_REVERSED = false;
bool running = false;
uint32_t lastLogAt = 0;

void stopMotors() {
  for (uint8_t pin : MOTOR_PINS) digitalWrite(pin, LOW);
  running = false;
}

void runMotors() {
  // ตั้ง HIGH/LOW ค้าง: สั่งกำลังเต็ม ไม่ต้องตั้งค่า PWM หรืออ่านเซนเซอร์
  stopMotors();
  digitalWrite(LEFT_REVERSED ? LEFT_IN2 : LEFT_IN1, HIGH);
  digitalWrite(RIGHT_REVERSED ? RIGHT_IN2 : RIGHT_IN1, HIGH);
  running = true;
}

void printStatus() {
  // รายงานคำสั่งที่ GPIO ไม่ใช่ค่าป้อนกลับว่ามอเตอร์หมุนจริง
  Serial.printf("CMD=%s | LEFT: GPIO32=%d GPIO33=%d | RIGHT: GPIO25=%d GPIO26=%d\n",
                running ? "RUN" : "STOP", digitalRead(LEFT_IN1), digitalRead(LEFT_IN2),
                digitalRead(RIGHT_IN1), digitalRead(RIGHT_IN2));
}
} // namespace

void setup() {
  for (uint8_t pin : MOTOR_PINS) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  runMotors(); // ไม่รอ Serial หรือหน่วงเวลาเริ่มต้น

  Serial.begin(115200);
  Serial.println("MOTOR TEST: both motors commanded ON at full power.");
  Serial.println("L298N: ENA/ENB enabled, motor supply connected, common GND.");
  Serial.println("s = stop | g = run again | no sensor inputs used");
  printStatus();
}

void loop() {
  for (uint8_t i = 0; i < 16 && Serial.available() > 0; ++i) {
    const char command = Serial.read();
    if (command == 's' || command == 'S') {
      stopMotors();
      printStatus();
    } else if (command == 'g' || command == 'G') {
      runMotors();
      printStatus();
    }
  }

  const uint32_t now = millis();
  if (now - lastLogAt >= 1000) {
    lastLogAt = now;
    printStatus();
  }
}
