#include <Arduino.h>
#include <esp_arduino_version.h>

#if ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 3, 11)
#error "Install esp32 by Espressif Systems 3.3.11 or newer."
#endif

constexpr int DATA_PIN = 33;  // G33; use 26 for Grove yellow.
constexpr uint32_t SERVO_BAUD = 1000000;

constexpr int16_t MAX_DUTY = 1000;
constexpr uint32_t SWAP_MS = 1300;
constexpr uint32_t SAMPLE_MS = 100;  // Slow logging between swaps.

// Each swap tries a different way to get past the servo's internal PWM ramp.
// Findings so far:
// - Dropping torque or setting goal speed (0x2E) changes nothing.
// - Bouncing through position mode resets the motor-mode ramp to 0.
// - Position mode ramps ~4x faster (~5 units/ms), but re-entering motor
//   mode restarts at 0, so a position kick gains nothing.
// - Entering position mode resets the goal to the present position.
enum Swap { MODE_BOUNCE, POSITION_KICK, BOUNCE_REG29_HIGH, BOUNCE_REG29_LOW };
const char *const SWAP_NAMES[] = {
  "mode bounce", "position kick", "bounce, 0x29=254", "bounce, 0x29=1"
};

// 0x29 is undefined on SCS but is acceleration on the STS series.
const Swap ROTATION[] = {MODE_BOUNCE, BOUNCE_REG29_HIGH, BOUNCE_REG29_LOW};

// Position kick: time spent in position mode with a far goal
// before switching back to motor mode.
constexpr uint32_t KICK_MS = 40;
constexpr uint32_t CAPTURE_MS = 240;  // Fast capture window after each swap.

const uint8_t MOTOR_MODE[] = {0x00, 0x00, 0x00, 0x00};     // Angle limits 0/0.
const uint8_t POSITION_MODE[] = {0x00, 0x00, 0x03, 0xFF};  // Angle limits 0/1023.
uint32_t swapCount = 0;

struct Sample {
  uint16_t ms;
  int16_t pwm;
  uint16_t pos;
  bool positionMode;
};
Sample samples[160];
size_t sampleCount = 0;

HardwareSerial ServoBus(1);

int servoId = -1;  // Found at startup; -1 if no servo answered.
int16_t duty = MAX_DUTY;
uint32_t swapStartMs = 0;
uint32_t nextSampleMs = 0;

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

// READ of 1..64 bytes from one servo. The shared data line also hands back
// our own transmission, so the local echo is skipped before parsing.
// Returns the reply's ERROR status byte, or -1 if no valid reply arrived.
int servoRead(uint8_t id, uint8_t address, uint8_t count, uint8_t *out) {
  if (count == 0 || count > 64) return -1;

  const uint8_t params[] = {address, count};
  uint8_t packet[8];
  const uint8_t sent = sendPacket(id, 0x02, params, sizeof(params), packet);

  // Reply: FF FF id len error data[count] checksum, with len = count + 2.
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

int findServo() {
  uint8_t id;
  if (servoRead(1, 0x05, 1, &id) >= 0) return 1;  // Factory default ID.

  for (int candidate = 0; candidate <= 253; ++candidate) {
    if (servoRead(candidate, 0x05, 1, &id) >= 0) return candidate;
  }
  return -1;
}

void dumpRegisters() {
  for (uint8_t base = 0x00; base < 0x60; base += 16) {
    uint8_t data[16];
    Serial.printf("  %02X:", base);

    if (servoRead(servoId, base, sizeof(data), data) < 0) {
      Serial.println(" (no reply)");
      continue;
    }
    for (uint8_t b : data) {
      Serial.printf(" %02X", b);
    }
    Serial.println();
  }
}

// In motor mode 0x2C holds PWM duty; bit 10 selects reverse.
void setDuty(int16_t value) {
  const uint16_t raw = value < 0
    ? static_cast<uint16_t>(-value) | 0x0400
    : static_cast<uint16_t>(value);

  // SC-series values are big-endian: high byte first.
  const uint8_t data[] = {
    static_cast<uint8_t>(raw >> 8),
    static_cast<uint8_t>(raw)
  };
  servoWrite(0x2C, data, sizeof(data));
}

int signMagnitude(uint16_t value, uint8_t signBit) {
  const int magnitude = value & ((1u << signBit) - 1);
  return value & (1u << signBit) ? -magnitude : magnitude;
}

// Records PWM output and position every 2 ms until endMs after startMs.
void captureUntil(uint32_t startMs, uint32_t endMs, bool positionMode) {
  uint32_t nextUs = micros();

  while (millis() - startMs < endMs) {
    if (static_cast<int32_t>(micros() - nextUs) < 0) continue;
    nextUs += 2000;

    uint8_t d[8];
    if (servoId < 0 || sampleCount == sizeof(samples) / sizeof(samples[0]) ||
        servoRead(servoId, 0x38, sizeof(d), d) < 0) {
      continue;
    }

    samples[sampleCount++] = {
      static_cast<uint16_t>(millis() - startMs),
      static_cast<int16_t>(signMagnitude(d[4] << 8 | d[5], 10)),
      static_cast<uint16_t>(d[0] << 8 | d[1]),
      positionMode
    };
  }
}

void swapDirection(Swap how) {
  const uint32_t startMs = millis();
  sampleCount = 0;

  const uint8_t reg29 = how == BOUNCE_REG29_HIGH ? 254
                      : how == BOUNCE_REG29_LOW ? 1
                      : 0;
  servoWrite(0x29, &reg29, 1);

  if (how == POSITION_KICK) {
    // Entering position mode resets the goal to the present position,
    // so switch first and set the goal after.
    servoWrite(0x09, POSITION_MODE, sizeof(POSITION_MODE));

    // Positive duty drives the position reading down, negative drives it up.
    // Aim at the far end so the position loop pushes hard that way.
    const uint16_t goal = duty > 0 ? 0 : 1023;
    const uint8_t move[] = {
      static_cast<uint8_t>(goal >> 8),
      static_cast<uint8_t>(goal),
      0x00, 0x01,  // Time 1 ms: as fast as it can.
      0x00, 0x00   // Speed 0: use the time.
    };
    servoWrite(0x2A, move, sizeof(move));
    captureUntil(startMs, KICK_MS, true);
  } else {
    servoWrite(0x09, POSITION_MODE, sizeof(POSITION_MODE));
  }

  servoWrite(0x09, MOTOR_MODE, sizeof(MOTOR_MODE));
  setDuty(duty);
  captureUntil(startMs, CAPTURE_MS, false);

  Serial.printf("--- duty %+d (%s)\n", duty, SWAP_NAMES[how]);
  for (size_t i = 0; i < sampleCount; ++i) {
    Serial.printf("  %3u ms %c pwm-out %+5d pos %4u\n",
                  static_cast<unsigned>(samples[i].ms),
                  samples[i].positionMode ? 'P' : 'M',
                  samples[i].pwm,
                  static_cast<unsigned>(samples[i].pos));
  }
}

void setup() {
  Serial.begin(115200);

  // Same RX and TX pin selects open-drain one-wire UART.
  // Keep the external pull-up to 3.3V.
  // Do not call pinMode() on DATA_PIN after this.
  ServoBus.begin(SERVO_BAUD, SERIAL_8N1, DATA_PIN, DATA_PIN);

  delay(1000);  // Allow the servo to finish powering up.

  servoId = findServo();
  if (servoId < 0) {
    Serial.println("No reply from any servo ID; no telemetry.");
  } else {
    Serial.printf("Servo ID %d, registers before any change:\n", servoId);
    dumpRegisters();
  }

  // Motor mode once: min and max angle limits both 0.
  servoWrite(0x09, MOTOR_MODE, sizeof(MOTOR_MODE));

  const uint8_t torqueOn = 1;
  servoWrite(0x28, &torqueOn, 1);

  setDuty(duty);
  swapStartMs = nextSampleMs = millis();
  Serial.printf("--- duty %+d\n", duty);
}

void loop() {
  const uint32_t now = millis();

  if (now - swapStartMs >= SWAP_MS) {
    swapStartMs += SWAP_MS;
    duty = -duty;
    swapDirection(ROTATION[++swapCount % (sizeof(ROTATION) / sizeof(ROTATION[0]))]);
    return;
  }

  if (servoId < 0 || static_cast<int32_t>(now - nextSampleMs) < 0) return;
  nextSampleMs = now + SAMPLE_MS;

  // 0x38..0x3F: position, speed, load, voltage, temperature.
  uint8_t d[8];
  const int error = servoRead(servoId, 0x38, sizeof(d), d);
  const unsigned sinceSwap = now - swapStartMs;

  if (error < 0) {
    Serial.printf("%5u ms  (no reply)\n", sinceSwap);
    return;
  }

  // In motor mode, load reports the PWM duty actually being output.
  Serial.printf("%5u ms  pos %4u  speed %+5d  pwm-out %+5d  %4.1f V  %2u C  err %02X\n",
                sinceSwap,
                static_cast<unsigned>(d[0] << 8 | d[1]),
                signMagnitude(d[2] << 8 | d[3], 15),
                signMagnitude(d[4] << 8 | d[5], 10),
                d[6] / 10.0f,
                static_cast<unsigned>(d[7]),
                static_cast<unsigned>(error));
}
