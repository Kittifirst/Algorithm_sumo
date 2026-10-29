#include <Arduino.h>

// เลือก environment: sensor_check เพื่อ Build/Upload โปรแกรมเช็กเซนเซอร์
// อ่านขา digital OUT ทั้ง 7 ตัว พร้อมหยุดมอเตอร์ L298N ตลอดการทดสอบ
namespace {
constexpr uint32_t PRINT_INTERVAL_MS = 500;
constexpr uint8_t MOTOR_PINS[] = {32, 33, 25, 26};

struct Sensor {
  const char *name;
  uint8_t pin;
  bool isFloor;
};

// มองจากด้านบนของหุ่น
constexpr Sensor SENSORS[] = {
    {"IR_SIDE_L", 12, false},  // ข้างซ้าย
    {"IR_FRONT_L", 14, false}, // หน้าซ้าย
    {"IR_FRONT_C", 27, false}, // หน้ากลาง
    {"IR_FRONT_R", 21, false}, // หน้าขวา
    {"IR_SIDE_R", 19, false},  // ข้างขวา
    {"FLOOR_L", 13, true},    // พื้นซ้าย
    {"FLOOR_R", 18, true},    // พื้นขวา
};
constexpr size_t SENSOR_COUNT = sizeof(SENSORS) / sizeof(SENSORS[0]);
uint32_t lastPrintAt = 0;
bool printedOnce = false;

void printSensors(uint32_t now) {
  // เก็บค่าทั้งหมดก่อนพิมพ์ เพื่อให้อยู่ในรอบการอ่านเดียวกัน
  uint8_t values[SENSOR_COUNT];
  for (size_t i = 0; i < SENSOR_COUNT; ++i) {
    values[i] = digitalRead(SENSORS[i].pin);
  }

  Serial.printf("[%lu ms]", static_cast<unsigned long>(now));
  for (size_t i = 0; i < SENSOR_COUNT; ++i) {
    const bool active = values[i] == LOW;
    const char *meaning = SENSORS[i].isFloor ? (active ? "WHITE" : "BLACK")
                                           : (active ? "DETECTED" : "CLEAR");
    Serial.printf(" %s[%u]=%d(%s)", SENSORS[i].name,
                  static_cast<unsigned>(SENSORS[i].pin), values[i], meaning);
  }
  Serial.println();
}
} // namespace

void setup() {
  // ขา IN ทั้งสี่เป็น LOW: ไม่สั่งมอเตอร์วิ่ง แม้ใส่จัมเปอร์ ENA/ENB
  for (uint8_t pin : MOTOR_PINS) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  for (const Sensor &sensor : SENSORS) pinMode(sensor.pin, INPUT);

  Serial.begin(115200);
  Serial.println("=== SENSOR CHECK: 7 digital inputs / motors stopped ===");
  Serial.println("IR: 0=DETECTED, 1=CLEAR | FLOOR: 0=WHITE, 1=BLACK");
  Serial.println("L=left, C=center, R=right; GPIO is shown in [brackets].");
  Serial.println("Readings repeat every 500 ms. No automatic motor start.");
}

void loop() {
  const uint32_t now = millis();
  if (!printedOnce || now - lastPrintAt >= PRINT_INTERVAL_MS) {
    lastPrintAt = now;
    printedOnce = true;
    printSensors(now);
  }
}
