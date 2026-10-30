#include <Arduino.h>
#include <esp_arduino_version.h>

#if ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 3, 11)
#error "Install esp32 by Espressif Systems 3.3.11 or newer."
#endif

constexpr int DATA_PIN = 33;  // G33; use 26 for Grove yellow.
constexpr uint32_t SERVO_BAUD = 1000000;

// Motor mode is open-loop PWM: 1000 = 100% duty.
// SC15 spec at full duty: 62.5 RPM no-load at 6 V.
constexpr int16_t MAX_PWM = 1000;
constexpr int DIRECTION = 1;  // -1 to spin the other way.

constexpr uint32_t RAMP_MS = 10000;  // Duty rises linearly 0 -> max.
constexpr uint32_t PAUSE_MS = 1000;
constexpr uint16_t UPDATE_MS = 20;
constexpr uint32_t REPORT_MS = 500;

HardwareSerial ServoBus(1);

enum class Phase { Ramp, Pause };

Phase phase = Phase::Ramp;
uint32_t phaseStartMs = 0;
uint32_t nextUpdateMs = 0;
uint32_t nextReportMs = 0;
uint32_t cycle = 1;

// Broadcast WRITE of 1..6 consecutive register bytes.
// Broadcast ID 0xFE means every connected servo; no reply is requested.
void servoWrite(uint8_t address, const uint8_t *data, uint8_t count) {
  if (count == 0 || count > 6) return;

  uint8_t packet[13] = {
    0xFF, 0xFF,
    0xFE,                            // Broadcast ID
    static_cast<uint8_t>(count + 3), // Packet length
    0x03,                            // WRITE instruction
    address
  };

  for (uint8_t i = 0; i < count; ++i) {
    packet[6 + i] = data[i];
  }

  uint8_t checksum = 0;
  for (uint8_t i = 2; i < 6 + count; ++i) {
    checksum += packet[i];
  }
  packet[6 + count] = static_cast<uint8_t>(~checksum);

  ServoBus.write(packet, count + 7);
  ServoBus.flush(true);  // Wait for transmission to finish.

  // Discard local echo. Broadcast WRITE does not get a servo reply.
  while (ServoBus.available()) {
    ServoBus.read();
  }
}

// In motor mode the goal-time register (0x2C) holds PWM duty:
// magnitude 0..1000, bit 10 set for reverse.
void setPwm(int16_t pwm) {
  const uint16_t raw = pwm < 0
    ? static_cast<uint16_t>(-pwm) | 0x0400
    : static_cast<uint16_t>(pwm);

  // SC-series values are big-endian: high byte first.
  const uint8_t data[] = {
    static_cast<uint8_t>(raw >> 8),
    static_cast<uint8_t>(raw)
  };

  servoWrite(0x2C, data, sizeof(data));
}

int16_t rampPwm(uint32_t t) {
  return static_cast<int16_t>(DIRECTION * (MAX_PWM * static_cast<int32_t>(t) / static_cast<int32_t>(RAMP_MS)));
}

void startRamp(uint32_t now) {
  phase = Phase::Ramp;
  phaseStartMs = nextReportMs = now;
  Serial.printf("Cycle %u: ramping duty 0 -> %d%% over %u ms\n",
                static_cast<unsigned>(cycle),
                MAX_PWM / 10,
                static_cast<unsigned>(RAMP_MS));
}

void setup() {
  Serial.begin(115200);

  // Same RX and TX pin selects open-drain one-wire UART.
  // Keep the external pull-up to 3.3V.
  // Do not call pinMode() on DATA_PIN after this.
  ServoBus.begin(SERVO_BAUD, SERIAL_8N1, DATA_PIN, DATA_PIN);

  delay(1000);  // Allow the servo to finish powering up.

  // Motor (continuous rotation) mode: min and max angle limits both 0.
  // The EEPROM lock stays on, so this is not saved: the servo returns
  // to position mode after a power cycle.
  const uint8_t motorMode[] = {0x00, 0x00, 0x00, 0x00};
  servoWrite(0x09, motorMode, sizeof(motorMode));

  setPwm(0);

  const uint8_t torqueOn = 1;
  servoWrite(0x28, &torqueOn, 1);

  nextUpdateMs = millis();
  startRamp(nextUpdateMs);
}

void loop() {
  const uint32_t now = millis();

  if (static_cast<int32_t>(now - nextUpdateMs) < 0) {
    delay(1);
    return;
  }
  nextUpdateMs += UPDATE_MS;

  const uint32_t elapsed = now - phaseStartMs;

  if (phase == Phase::Ramp) {
    const uint32_t t = min(elapsed, RAMP_MS);
    const int16_t pwm = rampPwm(t);

    setPwm(pwm);

    if (static_cast<int32_t>(now - nextReportMs) >= 0) {
      nextReportMs = now + REPORT_MS;
      Serial.printf("  t=%5.2f s  pwm %5d  duty %5.1f%%\n",
                    t / 1000.0f, pwm, pwm / 10.0f);
    }

    if (t == RAMP_MS) {
      setPwm(0);
      phase = Phase::Pause;
      phaseStartMs = now;
      Serial.printf("Cycle %u: full duty reached, stopped for %u ms\n",
                    static_cast<unsigned>(cycle),
                    static_cast<unsigned>(PAUSE_MS));
    }
  } else if (elapsed >= PAUSE_MS) {
    ++cycle;
    startRamp(now);
  }
}
