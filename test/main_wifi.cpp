#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_arduino_version.h>
#include <esp_system.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

// ESP32 + L298N. Fit ENA/ENB jumpers; PWM is applied to the IN pins.
// Preserve the working pin assignment and sensor polarity from the old firmware.
const uint8_t IR_PINS[] = {12, 14, 27, 21, 19};
const uint8_t FLOOR_LEFT = 13, FLOOR_RIGHT = 18;
const uint8_t LEFT_FORWARD_PIN = 33, LEFT_REVERSE_PIN = 32;
const uint8_t RIGHT_FORWARD_PIN = 25, RIGHT_REVERSE_PIN = 26;
const uint8_t LEFT_FORWARD_CHANNEL = 0, RIGHT_FORWARD_CHANNEL = 1;
const uint8_t LEFT_REVERSE_CHANNEL = 2, RIGHT_REVERSE_CHANNEL = 3;
const uint8_t FLOOR_WHITE_LEVEL = 1, IR_DETECTED_LEVEL = 0;
const bool LEFT_DIRECTION_REVERSED = false, RIGHT_DIRECTION_REVERSED = false;
const uint32_t PWM_HZ = 1000;
const uint8_t PWM_BITS = 8;

// ESP32 and control PC both join this existing 2.4 GHz hotspot/router.
// DHCP assigns the robot's IP; the PC discovers it with UDP PING/PONG on 4210.
const bool ENABLE_WIFI = true;
const char *WIFI_SSID = "Kittifirst";
const char *WIFI_PASSWORD = "12345678";
const char *WIFI_HOSTNAME = "sumo-robot";
const uint32_t WIFI_RETRY_MS = 15000, WIFI_STATUS_LOG_MS = 5000;
const uint32_t UDP_RETRY_MS = 1000;
const uint16_t UDP_PORT = 4210;
const uint32_t EDGE_MIN_BACK_MS = 120, EDGE_MAX_BACK_MS = 550;
const uint32_t EDGE_TURN_MS = 260;
const int EDGE_BACK_PWM = 120, EDGE_TURN_PWM = 130;

enum class Source { USB, UDP };
enum class EdgeState { CLEAR, BACK, TURN };
WiFiUDP udp;
bool udpReady = false, pwmReady = false, armed = false;
bool wifiWasConnected = false;
IPAddress wifiLocalIP;
uint32_t lastWifiAttemptAt = 0, lastWifiLogAt = 0, lastUdpAttemptAt = 0;
Source owner = Source::USB;
IPAddress ownerIP;
uint16_t ownerPort = 0;
uint32_t challenge = 0, session = 0, sequence = 0;
uint32_t lastDriveAt = 0, commandTTL = 250, lastTelemetryAt = 0;
uint32_t edgeStartedAt = 0;
EdgeState edgeState = EdgeState::CLEAR;
bool turnRight = true;
int requestedLeft = 0, requestedRight = 0, leftCommand = 0, rightCommand = 0;
const char *action = "BOOT_STOP";
char serialLine[128];
size_t serialLength = 0;
bool serialOverflow = false;

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
  if (speed >= 0) {
    writePwm(reversePin, reverseChannel, 0);
    writePwm(forwardPin, forwardChannel, speed);
  } else {
    writePwm(forwardPin, forwardChannel, 0);
    writePwm(reversePin, reverseChannel, -speed);
  }
}

// Signed wheel API: forward, reverse, pivot and curved flank approaches.
void drive(int left, int right) {
  leftCommand = pwmReady ? constrain(left, -255, 255) : 0;
  rightCommand = pwmReady ? constrain(right, -255, 255) : 0;
  if (!pwmReady) return;
  setMotor(LEFT_FORWARD_PIN, LEFT_REVERSE_PIN, LEFT_FORWARD_CHANNEL,
           LEFT_REVERSE_CHANNEL, LEFT_DIRECTION_REVERSED ? -leftCommand : leftCommand);
  setMotor(RIGHT_FORWARD_PIN, RIGHT_REVERSE_PIN, RIGHT_FORWARD_CHANNEL,
           RIGHT_REVERSE_CHANNEL, RIGHT_DIRECTION_REVERSED ? -rightCommand : rightCommand);
}

void invalidateChallenge() {
  challenge = esp_random();
  if (challenge == 0) challenge = 1;
}

void stopRobot(const char *why) {
  armed = false;
  requestedLeft = requestedRight = 0;
  edgeState = EdgeState::CLEAR;
  drive(0, 0);
  action = why;
  invalidateChallenge();  // Delayed ARM from before STOP is invalid.
}

const char *wifiStatusName(wl_status_t status) {
  switch (status) {
    case WL_IDLE_STATUS: return "IDLE";
    case WL_NO_SSID_AVAIL: return "SSID_NOT_FOUND";
    case WL_SCAN_COMPLETED: return "SCAN_COMPLETE";
    case WL_CONNECTED: return "CONNECTED_WAITING_IP";
    case WL_CONNECT_FAILED: return "CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "CONNECTION_LOST";
    case WL_DISCONNECTED: return "DISCONNECTED";
    default: return "UNKNOWN";
  }
}

void updateWifi(uint32_t now) {
  if (!ENABLE_WIFI) return;
  wl_status_t status = WiFi.status();
  IPAddress ip = WiFi.localIP();
  const bool connected = status == WL_CONNECTED && ip != IPAddress(0, 0, 0, 0);

  if (!connected) {
    if (wifiWasConnected || udpReady || armed) {
      // Stop before processing any more commands. Reconnection always needs ARM.
      stopRobot("WIFI_LOST");
      udp.stop();
      udpReady = false;
      wifiWasConnected = false;
      wifiLocalIP = IPAddress(0, 0, 0, 0);
      Serial.printf("WIFI DISCONNECTED status=%s | motors=STOP | UDP=OFF\n", wifiStatusName(status));
      lastWifiLogAt = now;
    }
    if (now - lastWifiLogAt >= WIFI_STATUS_LOG_MS) {
      lastWifiLogAt = now;
      Serial.printf("WIFI CONNECTING SSID=%s status=%s | check hotspot is on, 2.4GHz and password\n",
                    WIFI_SSID, wifiStatusName(status));
    }
    if (now - lastWifiAttemptAt >= WIFI_RETRY_MS) {
      lastWifiAttemptAt = now;
      // This initiates connection; there is no wait loop or delay here.
      WiFi.reconnect();
      Serial.printf("WIFI RETRY SSID=%s\n", WIFI_SSID);
    }
    return;
  }

  const bool newConnection = !wifiWasConnected || ip != wifiLocalIP;
  if (newConnection) {
    stopRobot("WIFI_READY");
    udp.stop();
    udpReady = false;
    wifiWasConnected = true;
    wifiLocalIP = ip;
    Serial.printf("WIFI CONNECTED SSID=%s HOST=%s IP=%s GATEWAY=%s RSSI=%d\n",
                  WIFI_SSID, WIFI_HOSTNAME, ip.toString().c_str(),
                  WiFi.gatewayIP().toString().c_str(), WiFi.RSSI());
  }
  if (!udpReady && (newConnection || now - lastUdpAttemptAt >= UDP_RETRY_MS)) {
    lastUdpAttemptAt = now;
    udpReady = udp.begin(UDP_PORT) == 1;
    if (udpReady) {
      Serial.printf("UDP READY IP=%s PORT=%u | PING/PONG discovery enabled | motors=STOP\n",
                    ip.toString().c_str(), UDP_PORT);
    } else {
      stopRobot("UDP_FAIL");
      Serial.printf("UDP FAILED PORT=%u | retrying in %lu ms | motors=STOP\n",
                    UDP_PORT, (unsigned long)UDP_RETRY_MS);
    }
  }
}

void reply(Source source, IPAddress ip, uint16_t port, const char *message) {
  if (source == Source::USB) {
    if (Serial.availableForWrite() >= static_cast<int>(strlen(message) + 1)) Serial.println(message);
  } else if (udpReady) {
    udp.beginPacket(ip, port);
    udp.write(reinterpret_cast<const uint8_t *>(message), strlen(message));
    udp.write('\n');
    udp.endPacket();
  }
}

bool sameOwner(Source source, IPAddress ip, uint16_t port) {
  return source == owner && (source == Source::USB || (ip == ownerIP && port == ownerPort));
}

bool unsignedNumber(const char *text, uint32_t &value) {
  if (!text || !*text) return false;
  for (const char *p = text; *p; ++p) if (*p < '0' || *p > '9') return false;
  errno = 0;
  char *end = nullptr;
  unsigned long number = strtoul(text, &end, 10);
  if (errno || !end || *end) return false;
  value = number;
  return true;
}

bool wheelNumber(const char *text, int &value) {
  if (!text || !*text) return false;
  const char *digits = (*text == '-') ? text + 1 : text;
  uint32_t number;
  if (!unsignedNumber(digits, number) || number > 255) return false;
  value = (*text == '-') ? -static_cast<int>(number) : static_cast<int>(number);
  return true;
}

void handleLine(char *line, Source source, IPAddress ip, uint16_t port, uint32_t now) {
  char *parts[8], *saved = nullptr;
  size_t count = 0;
  for (char *part = strtok_r(line, " \r\n\t", &saved); part; part = strtok_r(nullptr, " \r\n\t", &saved)) {
    if (count >= 8) return;
    parts[count++] = part;
  }
  if (!count) return;
  char response[96];
  if (count == 1 && (!strcmp(parts[0], "STOP") || !strcmp(parts[0], "s"))) {
    stopRobot("USER_STOP");
    reply(source, ip, port, "STOPPED");
    return;
  }
  uint32_t token = 0;
  if (count < 2 || !unsignedNumber(parts[1], token) || token == 0) return;
  // Robot control is Wi-Fi only. Serial remains available for logs and Stop.
  if (source != Source::UDP) return;
  if (count == 2 && !strcmp(parts[0], "PING")) {
    // Link verification does not arm the robot or renew its motor watchdog.
    snprintf(response, sizeof(response), "PONG %lu 1 %lu %d", (unsigned long)token,
             (unsigned long)now, armed ? 1 : 0);
    reply(source, ip, port, response);
  } else if (count == 2 && !strcmp(parts[0], "HELLO")) {
    if (armed) return;
    snprintf(response, sizeof(response), "READY %lu %lu 1", (unsigned long)token, (unsigned long)challenge);
    reply(source, ip, port, response);
  } else if (count == 3 && !strcmp(parts[0], "ARM")) {
    uint32_t receivedChallenge;
    if (armed || !pwmReady || !unsignedNumber(parts[2], receivedChallenge) || receivedChallenge != challenge) return;
    if (digitalRead(FLOOR_LEFT) == FLOOR_WHITE_LEVEL || digitalRead(FLOOR_RIGHT) == FLOOR_WHITE_LEVEL) {
      snprintf(response, sizeof(response), "ERROR %lu FLOOR_WHITE", (unsigned long)token);
      reply(source, ip, port, response);
      return;
    }
    session = token;
    sequence = 0;
    owner = source;
    ownerIP = ip;
    ownerPort = port;
    requestedLeft = requestedRight = 0;
    edgeState = EdgeState::CLEAR;
    armed = true;
    action = "ARMED_IDLE";
    commandTTL = 250;
    lastDriveAt = now;
    invalidateChallenge();
    snprintf(response, sizeof(response), "ARMED %lu", (unsigned long)session);
    reply(source, ip, port, response);
  } else if (count == 6 && !strcmp(parts[0], "DRIVE")) {
    uint32_t nextSequence, ttl;
    int left, right;
    if (!armed || token != session || !sameOwner(source, ip, port)) return;
    if (!unsignedNumber(parts[2], nextSequence) || nextSequence <= sequence ||
        !wheelNumber(parts[3], left) || !wheelNumber(parts[4], right) ||
        !unsignedNumber(parts[5], ttl) || ttl < 80 || ttl > 350) return;
    sequence = nextSequence;
    requestedLeft = left;
    requestedRight = right;
    commandTTL = ttl;
    lastDriveAt = now;
  }
}

void readCommands(uint32_t now) {
  for (size_t i = 0; i < 256 && Serial.available(); ++i) {
    char c = Serial.read();
    if (c == 's' && serialLength == 0 && !serialOverflow) {
      stopRobot("USER_STOP");  // Legacy lowercase s, no Enter required.
      continue;
    }
    if (c == '\n' || c == '\r') {
      if (!serialOverflow && serialLength) {
        serialLine[serialLength] = '\0';
        handleLine(serialLine, Source::USB, IPAddress(), 0, now);
      }
      serialLength = 0;
      serialOverflow = false;
    } else if (!serialOverflow) {
      if (serialLength + 1 < sizeof(serialLine) && c >= 32 && c <= 126) serialLine[serialLength++] = c;
      else serialOverflow = true;
    }
  }
  if (!udpReady) return;
  for (uint8_t i = 0; i < 8; ++i) {
    int size = udp.parsePacket();
    if (!size) break;
    IPAddress ip = udp.remoteIP();
    uint16_t port = udp.remotePort();
    char packet[128];
    if (size >= static_cast<int>(sizeof(packet))) {
      while (udp.available()) udp.read();
      continue;
    }
    int length = udp.read(packet, sizeof(packet) - 1);
    if (length <= 0) continue;
    packet[length] = '\0';
    handleLine(packet, Source::UDP, ip, port, now);
  }
}

void updateRobot(bool whiteLeft, bool whiteRight, uint32_t now) {
  if (!pwmReady || !armed) { drive(0, 0); return; }
  if (now - lastDriveAt >= commandTTL) { stopRobot("WATCHDOG"); return; }
  if (edgeState == EdgeState::BACK) {
    if (now - edgeStartedAt >= EDGE_MAX_BACK_MS) { stopRobot("EDGE_TIMEOUT"); return; }
    if (now - edgeStartedAt < EDGE_MIN_BACK_MS || whiteLeft || whiteRight) {
      action = "EDGE_BACK";
      drive(-EDGE_BACK_PWM, -EDGE_BACK_PWM);
      return;
    }
    edgeState = EdgeState::TURN;
    edgeStartedAt = now;
  }
  if (whiteLeft || whiteRight) {
    turnRight = whiteLeft;
    edgeState = EdgeState::BACK;
    edgeStartedAt = now;
    action = "EDGE_BACK";
    drive(-EDGE_BACK_PWM, -EDGE_BACK_PWM);
    return;
  }
  if (edgeState == EdgeState::TURN) {
    if (now - edgeStartedAt < EDGE_TURN_MS) {
      action = "EDGE_TURN";
      drive(turnRight ? EDGE_TURN_PWM : -EDGE_TURN_PWM,
            turnRight ? -EDGE_TURN_PWM : EDGE_TURN_PWM);
      return;
    }
    edgeState = EdgeState::CLEAR;
  }
  action = (requestedLeft || requestedRight) ? "DRIVE" : "HOLD";
  drive(requestedLeft, requestedRight);
}

void setup() {
  const uint8_t motorPins[] = {LEFT_FORWARD_PIN, LEFT_REVERSE_PIN, RIGHT_FORWARD_PIN, RIGHT_REVERSE_PIN};
  for (uint8_t pin : motorPins) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
  for (uint8_t pin : IR_PINS) pinMode(pin, INPUT);
  pinMode(FLOOR_LEFT, INPUT);
  pinMode(FLOOR_RIGHT, INPUT);
  Serial.begin(115200);
  bool a = attachPwm(LEFT_FORWARD_PIN, LEFT_FORWARD_CHANNEL);
  bool b = attachPwm(RIGHT_FORWARD_PIN, RIGHT_FORWARD_CHANNEL);
  bool c = attachPwm(LEFT_REVERSE_PIN, LEFT_REVERSE_CHANNEL);
  bool d = attachPwm(RIGHT_REVERSE_PIN, RIGHT_REVERSE_CHANNEL);
  pwmReady = a && b && c && d;
  stopRobot(pwmReady ? "BOOT_STOP" : "PWM_FAIL");
  Serial.println("FIRMWARE SUMO_TOPVIEW_WIFI_STA_V3 | boot=STOP | control=WiFi_UDP:4210 | serial=logs/stop");
  if (ENABLE_WIFI) {
    WiFi.mode(WIFI_STA);
    bool hostnameReady = WiFi.setHostname(WIFI_HOSTNAME);
    WiFi.setAutoReconnect(true);
    lastWifiAttemptAt = lastWifiLogAt = millis();
    Serial.printf("WIFI CONNECTING SSID=%s MODE=STA HOST=%s | hotspot must support 2.4GHz\n",
                  WIFI_SSID, WIFI_HOSTNAME);
    if (!hostnameReady) Serial.println("WIFI WARNING: hostname setup failed; connection will still be attempted");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  } else {
    Serial.println("WIFI DISABLED | motors remain stopped");
  }
  Serial.println("PING token -> PONG token 1 uptime armed (does not start motors)");
  Serial.println("HELLO token -> READY token challenge 1; ARM token challenge -> ARMED token");
  Serial.println("DRIVE token seq left right ttl_ms (80..350); STOP or lowercase s = stop");
}

void loop() {
  uint32_t now = millis();
  updateWifi(now);
  readCommands(now);
  now = millis();
  bool whiteLeft = digitalRead(FLOOR_LEFT) == FLOOR_WHITE_LEVEL;
  bool whiteRight = digitalRead(FLOOR_RIGHT) == FLOOR_WHITE_LEVEL;
  updateRobot(whiteLeft, whiteRight, now);
  if (session && now - lastTelemetryAt >= 100) {
    lastTelemetryAt = now;
    uint8_t ir = 0;
    for (uint8_t i = 0; i < 5; ++i) if (digitalRead(IR_PINS[i]) == IR_DETECTED_LEVEL) ir |= 1 << i;
    char message[128];
    snprintf(message, sizeof(message), "TEL %lu %lu %d %s %d %d %d %d %u %lu",
             (unsigned long)session, (unsigned long)sequence, armed ? 1 : 0, action,
             leftCommand, rightCommand, whiteLeft ? 1 : 0, whiteRight ? 1 : 0, ir, (unsigned long)now);
    reply(owner, ownerIP, ownerPort, message);
  }
  delay(1);
}
