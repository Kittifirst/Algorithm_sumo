#include <Arduino.h>
#include <esp_arduino_version.h>

// เลือก PlatformIO environment: motor_test
// ใส่จัมเปอร์ ENA/ENB ไว้ทั้งคู่ และยกล้อพ้นพื้นก่อน Upload
// เทสเฉพาะเดินหน้า ไม่อ่านเซนเซอร์ และไม่แก้ main.cpp
// กลับทิศซ้ายแล้วตามผลทดสอบ: ซ้ายใช้ GPIO33, ขวาใช้ GPIO25
constexpr uint8_t LEFT_PWM_PIN = 33;
constexpr uint8_t LEFT_LOW_PIN = 32;
constexpr uint8_t RIGHT_PWM_PIN = 25;
constexpr uint8_t RIGHT_LOW_PIN = 26;
constexpr uint8_t LEFT_CHANNEL = 0;
constexpr uint8_t RIGHT_CHANNEL = 1;
constexpr uint32_t PWM_HZ = 1000;
constexpr uint8_t PWM_BITS = 8;

struct Step {
  uint8_t duty;
  uint32_t durationMs;
};

// หยุดก่อนทุกความเร็ว เพื่อทดสอบว่าเริ่มหมุนจากหยุดนิ่งได้หรือไม่
constexpr Step STEPS[] = {
    {0, 1000}, {255, 3000},
    {0, 1000}, {200, 3000},
    {0, 1000}, {140, 3000},
};
constexpr size_t STEP_COUNT = sizeof(STEPS) / sizeof(STEPS[0]);
size_t stepIndex = 0;
uint32_t stepStartedAt = 0;
bool pwmReady = false;
bool paused = false;

bool attachPwm(uint8_t pin, uint8_t channel) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  if (!ledcAttachChannel(pin, PWM_HZ, PWM_BITS, channel)) return false;
  ledcWrite(pin, 0);
#else
  if (ledcSetup(channel, PWM_HZ, PWM_BITS) == 0) return false;
  ledcAttachPin(pin, channel);
  ledcWrite(channel, 0);
#endif
  return true;
}

void setDuty(uint8_t duty) {
  if (!pwmReady) return;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(LEFT_PWM_PIN, duty);
  ledcWrite(RIGHT_PWM_PIN, duty);
#else
  ledcWrite(LEFT_CHANNEL, duty);
  ledcWrite(RIGHT_CHANNEL, duty);
#endif
}

void startStep(uint32_t now) {
  stepStartedAt = now;
  setDuty(STEPS[stepIndex].duty);
  Serial.printf("PWM_TEST: duty=%u/255 | L=GPIO33 R=GPIO25 | duration=%lu ms\n",
                static_cast<unsigned>(STEPS[stepIndex].duty),
                static_cast<unsigned long>(STEPS[stepIndex].durationMs));
}

void setup() {
  const uint8_t pins[] = {LEFT_PWM_PIN, LEFT_LOW_PIN, RIGHT_PWM_PIN, RIGHT_LOW_PIN};
  for (uint8_t pin : pins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  Serial.begin(115200);
  const bool leftReady = attachPwm(LEFT_PWM_PIN, LEFT_CHANNEL);
  const bool rightReady = attachPwm(RIGHT_PWM_PIN, RIGHT_CHANNEL);
  pwmReady = leftReady && rightReady;
  Serial.println("FIRMWARE: L298N_PWM_TEST_1KHZ_8BIT");
  if (!pwmReady) {
    Serial.println("STOP: LEDC PWM initialization failed.");
    return;
  }
  Serial.println("Keep ENA/ENB jumpers ON. Lift wheels before testing.");
  Serial.println("STOP 1s -> 255 3s -> STOP 1s -> 200 3s -> STOP 1s -> 140 3s; repeat.");
  Serial.println("s = stop | g = restart test after 1s stop");
  startStep(millis());
}

void loop() {
  if (!pwmReady) return;
  const uint32_t now = millis();
  for (uint8_t i = 0; i < 16 && Serial.available() > 0; ++i) {
    const char command = Serial.read();
    if (command == 's' || command == 'S') {
      paused = true;
      setDuty(0);
      Serial.println("PWM_TEST: stopped; g = restart");
    } else if (command == 'g' || command == 'G') {
      paused = false;
      stepIndex = 0;
      startStep(now);
    }
  }
  if (paused) return;
  if (now - stepStartedAt >= STEPS[stepIndex].durationMs) {
    stepIndex = (stepIndex + 1) % STEP_COUNT;
    startStep(now);
  }
}
