#include <Arduino.h>
#include <esp_arduino_version.h>

#if ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 3, 11)
#error "Install esp32 by Espressif Systems 3.3.11 or newer."
#endif

// Moves the servo slowly to GOAL in position mode and holds it there with
// torque on, printing position and load. Used to park the horn in the
// middle of the sensed range while the pendulum arm is mounted pointing
// straight down.
//
// Serial commands, one per line:
//   r        release: torque off
//   h        hold where it is now (torque on)
//   <count>  move to that position (0..1023) and hold

constexpr int DATA_PIN = 33;  // G33; use 26 for Grove yellow.
constexpr uint32_t SERVO_BAUD = 1000000;
constexpr uint8_t SERVO_ID = 1;

constexpr uint16_t GOAL = 513;      // True down (holds fit); near the middle of 0..1023.
constexpr uint16_t MOVE_MS = 2000;  // Gentle move.
constexpr uint32_t REPORT_MS = 50;

const uint8_t POSITION_MODE[] = {0x00, 0x00, 0x03, 0xFF};  // Angle limits 0/1023.

HardwareSerial ServoBus(1);

// Sends one instruction packet and copies it into packet.
// Returns the packet length.
uint8_t sendPacket(uint8_t id, uint8_t instruction,
                   const uint8_t *params, uint8_t count, uint8_t *packet) {
  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = id;
  packet[3] = count + 2;  // Packet length
  packet[4] = instruction;
  memcpy(packet + 5, params, count);

  uint8_t checksum = 0;
  for (uint8_t i = 2; i < 5 + count; ++i) {
    checksum += packet[i];
  }
  packet[5 + count] = static_cast<uint8_t>(~checksum);

  // Drop stale echo from earlier writes so replies line up.
  while (ServoBus.available()) {
    ServoBus.read();
  }

  ServoBus.write(packet, count + 6);
  ServoBus.flush(true);  // Wait for transmission to finish.
  return count + 6;
}

// Broadcast WRITE of 1..6 consecutive register bytes; no reply is requested.
void servoWrite(uint8_t address, const uint8_t *data, uint8_t count) {
  if (count == 0 || count > 6) return;

  uint8_t params[7] = {address};
  memcpy(params + 1, data, count);

  uint8_t packet[13];
  sendPacket(0xFE, 0x03, params, count + 1, packet);
}

// READ of 1..64 bytes from one servo, skipping the local echo.
// Returns the reply's ERROR status byte, or -1 if no valid reply arrived.
int servoRead(uint8_t id, uint8_t address, uint8_t count, uint8_t *out) {
  if (count == 0 || count > 64) return -1;

  const uint8_t params[] = {address, count};
  uint8_t packet[8];
  const uint8_t sent = sendPacket(id, 0x02, params, sizeof(params), packet);

  uint8_t buf[160];
  size_t n = 0;
  const size_t expected = sent + count + 6;
  const uint32_t startUs = micros();
  while (n < expected && micros() - startUs < 3000) {
    if (ServoBus.available()) {
      buf[n++] = ServoBus.read();
    }
  }

  size_t i = 0;
  if (n >= sent && memcmp(buf, packet, sent) == 0) {
    i = sent;
  }

  for (; i + count + 6 <= n; ++i) {
    if (buf[i] != 0xFF || buf[i + 1] != 0xFF ||
        buf[i + 2] != id || buf[i + 3] != count + 2) {
      continue;
    }

    uint8_t checksum = 0;
    for (size_t j = i + 2; j < i + 5 + count; ++j) {
      checksum += buf[j];
    }
    if (static_cast<uint8_t>(~checksum) != buf[i + 5 + count]) continue;

    memcpy(out, buf + i + 5, count);
    return buf[i + 4];
  }
  return -1;
}

void setTorque(bool on) {
  const uint8_t value = on ? 1 : 0;
  servoWrite(0x28, &value, 1);
}

// Entering position mode resets the goal to the present position, so the
// mode switch alone holds in place.
void holdHere() {
  servoWrite(0x09, POSITION_MODE, sizeof(POSITION_MODE));
  setTorque(true);
}

void moveTo(uint16_t goal) {
  holdHere();  // Switch first, so the goal written next sticks.

  const uint8_t move[] = {
    static_cast<uint8_t>(goal >> 8),
    static_cast<uint8_t>(goal),
    static_cast<uint8_t>(MOVE_MS >> 8),
    static_cast<uint8_t>(MOVE_MS),
    0x00, 0x00  // Speed 0: use the time.
  };
  servoWrite(0x2A, move, sizeof(move));
}

void handleCommand(const String &line) {
  if (line == "r") {
    setTorque(false);
    Serial.println("# released");
  } else if (line == "h") {
    holdHere();
    Serial.println("# holding here");
  } else if (line.length() && isDigit(line[0])) {
    const uint16_t goal = constrain(line.toInt(), 0, 1023);
    moveTo(goal);
    Serial.printf("# moving to %u\n", static_cast<unsigned>(goal));
  } else if (line.length()) {
    Serial.printf("# unknown command '%s' (r, h or a count)\n", line.c_str());
  }
}

void setup() {
  Serial.begin(115200);

  // Same RX and TX pin selects open-drain one-wire UART.
  // Keep the external pull-up to 3.3V.
  // Do not call pinMode() on DATA_PIN after this.
  ServoBus.begin(SERVO_BAUD, SERIAL_8N1, DATA_PIN, DATA_PIN);

  delay(1000);  // Allow the servo to finish powering up.

  moveTo(GOAL);
  Serial.printf("# holding at %u\n", static_cast<unsigned>(GOAL));
}

String pending;
uint32_t nextReportMs = 0;

void loop() {
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') {
      pending.trim();
      handleCommand(pending);
      pending = "";
    } else {
      pending += c;
    }
  }

  if (static_cast<int32_t>(millis() - nextReportMs) < 0) return;
  nextReportMs = millis() + REPORT_MS;

  // 0x38..0x3F: position, speed, load, voltage, temperature. 0x28: torque.
  uint8_t state[8], torque;
  if (servoRead(SERVO_ID, 0x38, sizeof(state), state) >= 0 &&
      servoRead(SERVO_ID, 0x28, 1, &torque) >= 0) {
    const unsigned load = state[4] << 8 | state[5];
    Serial.printf("t %lu  pos %4u  load %s%u  volt %u.%u  temp %u  torque %u\n",
                  static_cast<unsigned long>(millis()),
                  static_cast<unsigned>(state[0] << 8 | state[1]),
                  load & 0x400 ? "-" : "+", load & 0x3FF,
                  state[6] / 10, state[6] % 10,
                  static_cast<unsigned>(state[7]),
                  static_cast<unsigned>(torque));
  } else {
    Serial.println("read failed");
  }
}
