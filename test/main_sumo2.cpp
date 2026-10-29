#include <Arduino.h>
#include <esp_arduino_version.h>

// สำเนาโปรแกรมซูโม่: คัดลอกทั้งไฟล์ไป src/main.cpp เมื่อต้องการใช้งาน
// ESP32 + L298N: ใส่จัมเปอร์ ENA/ENB
// ค่าล่าสุด: IR เจอ=0 ไม่เจอ=1 / พื้นขาว=1 ดำ=0
const uint8_t IR_PINS[] = {12, 14, 27, 21, 19};
const uint8_t FLOOR_LEFT = 13;
const uint8_t FLOOR_RIGHT = 18;
// ทิศตั้งต้นจากโปรแกรมเทส PWM: ยังต้องตรวจว่าล้อเดินหน้าจริง
const uint8_t LEFT_FORWARD_PIN = 33;
const uint8_t LEFT_REVERSE_PIN = 32;
const uint8_t RIGHT_FORWARD_PIN = 25;
const uint8_t RIGHT_REVERSE_PIN = 26;
const uint8_t LEFT_FORWARD_CHANNEL = 0;
const uint8_t RIGHT_FORWARD_CHANNEL = 1;
const uint8_t LEFT_REVERSE_CHANNEL = 2;
const uint8_t RIGHT_REVERSE_CHANNEL = 3;
const uint32_t PWM_HZ = 1000;
const uint8_t PWM_BITS = 8;
const uint8_t FLOOR_WHITE_LEVEL = 0; // ขาว=0, ดำ=1
const uint8_t IR_DETECTED_LEVEL = 0; // เจอ=0, ไม่เจอ=1
// ถ้าสั่ง CMD บวกแล้วล้อข้างใดถอย ให้เปลี่ยนของข้างนั้นเป็น true
const bool LEFT_DIRECTION_REVERSED = false;
const bool RIGHT_DIRECTION_REVERSED = false;

// ความเร็ว 0..255
const int SEARCH_SPEED = 140;      // ไม่เจอ: เดินหน้าค้นหา
const int ATTACK_SPEED = 255;      // เจอ IR ตัวใดตัวหนึ่ง: เดินหน้าเต็มสปีด
const int BACK_SPEED = 140;
const int TURN_SPEED = 170;
const uint32_t START_WAIT_MS = 1000; // รอก่อนเริ่ม 1 วินาที
const uint32_t TURN_BACK_MS = 550;  // ปรับจนหันกลับประมาณ 180 องศา

enum State { WAITING, RUNNING, BACKING, TURNING, STOPPED };
State state = WAITING;
uint32_t stateStartedAt = 0;
uint32_t lastLogAt = 0;
bool turnRight = true;
int targetTurn = 1;
int leftCommand = 0;
int rightCommand = 0;
bool pwmReady = false;
const char *action = "WAIT";

// LEDC 1 kHz / 8 bit แบบเดียวกับโปรแกรมเทสที่หมุนได้แล้ว
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

void writePwm(uint8_t pin, uint8_t channel, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

void setMotor(uint8_t forwardPin, uint8_t reversePin,
              uint8_t forwardChannel, uint8_t reverseChannel, int speed) {
  speed = constrain(speed, -255, 255);
  // ปิดขาที่ไม่ใช้ก่อน แล้วส่ง PWM ให้ขาที่ใช้
  if (speed >= 0) {
    writePwm(reversePin, reverseChannel, 0);
    writePwm(forwardPin, forwardChannel, speed);
  } else {
    writePwm(forwardPin, forwardChannel, 0);
    writePwm(reversePin, reverseChannel, -speed);
  }
}

// บวก=เดินหน้า ลบ=ถอยหลัง หลังชดเชยทิศล้อแล้ว
void drive(int left, int right) {
  if (!pwmReady) return;
  leftCommand = left;
  rightCommand = right;
  setMotor(LEFT_FORWARD_PIN, LEFT_REVERSE_PIN,
           LEFT_FORWARD_CHANNEL, LEFT_REVERSE_CHANNEL,
           LEFT_DIRECTION_REVERSED ? -left : left);
  setMotor(RIGHT_FORWARD_PIN, RIGHT_REVERSE_PIN,
           RIGHT_FORWARD_CHANNEL, RIGHT_REVERSE_CHANNEL,
           RIGHT_DIRECTION_REVERSED ? -right : right);
}

void enterState(State next, uint32_t now) {
  state = next;
  stateStartedAt = now;
}

void updateRobot(bool whiteLeft, bool whiteRight, const uint8_t ir[5], uint32_t now) {
  if (!pwmReady) {
    state = STOPPED;
    action = "PWM_FAIL";
    return;
  }
  if (state == STOPPED) {
    drive(0, 0); // กด s แล้วต้องกด g จึงเริ่มใหม่
    return;
  }

  // รอเริ่มครั้งเดียว หลังจากนั้นเดินหาเองโดยไม่ต้องเห็นศัตรูก่อน
  if (state == WAITING) {
    drive(0, 0);
    action = "WAIT";
    if (now - stateStartedAt < START_WAIT_MS) return;
    enterState(RUNNING, now);
  }

  // ถอยขณะยังเห็นขาว เมื่อดำทั้งคู่ให้หมุนกลับทันที ไม่หยุดรอศัตรู
  if (state == BACKING) {
    if (whiteLeft || whiteRight) {
      action = "EDGE_BACK";
      drive(-BACK_SPEED, -BACK_SPEED);
      return;
    }
    enterState(TURNING, now);
  }

  // เปิดมาบนขาวก็ถอยตามขั้นตอนนี้ ขอบสนามสำคัญกว่าคู่ต่อสู้เสมอ
  if (whiteLeft || whiteRight) {
    turnRight = whiteLeft;
    enterState(BACKING, now);
    action = "EDGE_BACK";
    drive(-BACK_SPEED, -BACK_SPEED);
    return;
  }

  if (state == TURNING) {
    if (now - stateStartedAt < TURN_BACK_MS) {
      action = turnRight ? "TURN_RIGHT" : "TURN_LEFT";
      drive(turnRight ? TURN_SPEED : -TURN_SPEED,
            turnRight ? -TURN_SPEED : TURN_SPEED);
      return;
    }
    enterState(RUNNING, now);
  }

  const bool sideLeft = ir[0] == IR_DETECTED_LEVEL;
  const bool frontLeft = ir[1] == IR_DETECTED_LEVEL;
  const bool frontCenter = ir[2] == IR_DETECTED_LEVEL;
  const bool frontRight = ir[3] == IR_DETECTED_LEVEL;
  const bool sideRight = ir[4] == IR_DETECTED_LEVEL;

  if (frontCenter || (frontLeft && frontRight)) {
    action = "ATTACK";
    drive(ATTACK_SPEED, ATTACK_SPEED);
  } else if (frontLeft) {
    targetTurn = -1;
    action = "AIM_LEFT";
    drive(SEARCH_SPEED, ATTACK_SPEED);
  } else if (frontRight) {
    targetTurn = 1;
    action = "AIM_RIGHT";
    drive(ATTACK_SPEED, SEARCH_SPEED);
  } else if (sideLeft || sideRight) {
    if (sideLeft && !sideRight) targetTurn = -1;
    if (sideRight && !sideLeft) targetTurn = 1;
    action = targetTurn < 0 ? "SIDE_LEFT" : "SIDE_RIGHT";
    drive(targetTurn * TURN_SPEED, -targetTurn * TURN_SPEED);
  } else {
    action = "SEARCH";
    drive(SEARCH_SPEED, SEARCH_SPEED);
  }
}

void setup() {
  const uint8_t motorPins[] = {LEFT_FORWARD_PIN, LEFT_REVERSE_PIN,
                             RIGHT_FORWARD_PIN, RIGHT_REVERSE_PIN};
  for (uint8_t pin : motorPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  for (uint8_t pin : IR_PINS) pinMode(pin, INPUT);
  pinMode(FLOOR_LEFT, INPUT);
  pinMode(FLOOR_RIGHT, INPUT);

  Serial.begin(115200);
  const bool leftForwardReady = attachPwm(LEFT_FORWARD_PIN, LEFT_FORWARD_CHANNEL);
  const bool rightForwardReady = attachPwm(RIGHT_FORWARD_PIN, RIGHT_FORWARD_CHANNEL);
  const bool leftReverseReady = attachPwm(LEFT_REVERSE_PIN, LEFT_REVERSE_CHANNEL);
  const bool rightReverseReady = attachPwm(RIGHT_REVERSE_PIN, RIGHT_REVERSE_CHANNEL);
  pwmReady = leftForwardReady && rightForwardReady && leftReverseReady && rightReverseReady;

  Serial.println("FIRMWARE: SUMO_COPY_WHITE1_BLACK0_V6");
  Serial.println("SUMO: wait 1s -> search -> aim toward IR -> front detected = full speed.");
  Serial.println("FLOOR: 1=WHITE 0=BLACK | IR: 0=DETECTED 1=CLEAR");
  Serial.println("White -> reverse until both floors black -> turn -> search/attack.");
  Serial.println("s = stop | g = restart with 1s wait");
  Serial.println("STATE: 0=WAIT 1=RUN 2=BACK 3=TURN 4=STOP; CMD=L,R");
  if (!pwmReady) Serial.println("STOP: LEDC PWM initialization failed.");
  stateStartedAt = millis();
}

void loop() {
  const uint32_t now = millis();
  // สั่งหยุดได้ทุกช่วง รวมถึงตอนรอ ถอย และหมุน
  for (uint8_t i = 0; i < 16 && Serial.available() > 0; ++i) {
    const char command = Serial.read();
    if (command == 's' || command == 'S') {
      drive(0, 0);
      enterState(STOPPED, now);
      action = "USER_STOP";
    } else if (command == 'g' || command == 'G') {
      drive(0, 0);
      enterState(WAITING, now);
      action = "WAIT";
    }
  }

  uint8_t ir[5];
  for (uint8_t i = 0; i < 5; ++i) {
    ir[i] = digitalRead(IR_PINS[i]);
  }
  const int floorLeft = digitalRead(FLOOR_LEFT);
  const int floorRight = digitalRead(FLOOR_RIGHT);
  updateRobot(floorLeft == FLOOR_WHITE_LEVEL, floorRight == FLOOR_WHITE_LEVEL, ir, now);

  static State lastLoggedState = STOPPED;
  if ((state != lastLoggedState || now - lastLogAt >= 500) && Serial.availableForWrite() >= 96) {
    lastLogAt = now;
    lastLoggedState = state;
    Serial.printf("IR[12,14,27,21,19]=%d,%d,%d,%d,%d FLOOR[13,18]=%d,%d STATE=%d ACT=%s CMD=%d,%d\n",
                  ir[0], ir[1], ir[2], ir[3], ir[4], floorLeft, floorRight,
                  static_cast<int>(state), action, leftCommand, rightCommand);
  }
}
