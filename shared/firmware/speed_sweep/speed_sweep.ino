#include <Arduino.h>
#include <esp_arduino_version.h>

#if ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 3, 11)
#error "Install esp32 by Espressif Systems 3.3.11 or newer."
#endif

constexpr int DATA_PIN = 33;  // G33; use 26 for Grove yellow.
constexpr uint32_t SERVO_BAUD = 1000000;

constexpr float CENTER_DEG = 105.0f;
constexpr float AMPLITUDE_DEG = 30.0f;  // Sweeps 75..135 degrees.

// SC15 spec: 0.16 s/60 deg at 6 V = 375 deg/s (0.18 s/60 deg at 4.8 V).
constexpr float MAX_SPEED_DPS = 375.0f;

constexpr uint32_t RAMP_MS = 10000;  // Speed rises linearly 0 -> max.
constexpr uint32_t PAUSE_MS = 1000;
constexpr uint16_t UPDATE_MS = 20;   // Setpoint stream period (50 Hz).
constexpr uint32_t REPORT_MS = 500;

HardwareSerial ServoBus(1);

enum class Phase { Ramp, Pause };

Phase phase = Phase::Ramp;
uint32_t phaseStartMs = 0;
uint32_t nextUpdateMs = 0;
uint32_t nextReportMs = 0;
uint32_t cycle = 1;

// Triangle-wave phase carried across cycles so each ramp resumes
// from where the previous one stopped.
float sweepOffsetDeg = 0.0f;

// SC15: 0..1023 position counts cover approximately 0..210 degrees.
uint16_t degreesToPosition(float degrees) {
  return static_cast<uint16_t>(lroundf(degrees * 1023.0f / 210.0f));
}

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

void setGoal(float degrees, uint16_t durationMs) {
  const uint16_t position = degreesToPosition(degrees);

  // SC-series values are big-endian: high byte first.
  const uint8_t data[] = {
    static_cast<uint8_t>(position >> 8),
    static_cast<uint8_t>(position),

    static_cast<uint8_t>(durationMs >> 8),
    static_cast<uint8_t>(durationMs),

    0x00, 0x00  // Speed = 0: use the specified movement time.
  };

  servoWrite(0x2A, data, sizeof(data));
}

// Angle on a triangle wave around CENTER_DEG after travelling distanceDeg,
// starting at the center and heading up.
float triangleDeg(float distanceDeg) {
  const float u = fmodf(distanceDeg, 4.0f * AMPLITUDE_DEG);

  if (u < AMPLITUDE_DEG) return CENTER_DEG + u;
  if (u < 3.0f * AMPLITUDE_DEG) return CENTER_DEG + 2.0f * AMPLITUDE_DEG - u;
  return CENTER_DEG + u - 4.0f * AMPLITUDE_DEG;
}

float rampSpeedDps(uint32_t t) {
  return MAX_SPEED_DPS * t / RAMP_MS;
}

// Distance covered t ms into the ramp: integral of the linear speed ramp.
float rampDistanceDeg(uint32_t t) {
  return 0.5f * rampSpeedDps(t) * (t / 1000.0f);
}

void setup() {
  Serial.begin(115200);

  // Same RX and TX pin selects open-drain one-wire UART.
  // Keep the external pull-up to 3.3V.
  // Do not call pinMode() on DATA_PIN after this.
  ServoBus.begin(SERVO_BAUD, SERIAL_8N1, DATA_PIN, DATA_PIN);

  delay(1000);  // Allow the servo to finish powering up.

  // Load the initial target before enabling torque.
  setGoal(CENTER_DEG, 2000);

  const uint8_t torqueOn = 1;
  servoWrite(0x28, &torqueOn, 1);

  delay(2100);  // Allow the initial centering move to finish.

  phaseStartMs = nextUpdateMs = nextReportMs = millis();
  Serial.printf("Cycle %u: ramping 0 -> %.0f deg/s over %u ms\n",
                static_cast<unsigned>(cycle),
                MAX_SPEED_DPS,
                static_cast<unsigned>(RAMP_MS));
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
    const float degrees = triangleDeg(sweepOffsetDeg + rampDistanceDeg(t));

    setGoal(degrees, UPDATE_MS);

    if (static_cast<int32_t>(now - nextReportMs) >= 0) {
      nextReportMs = now + REPORT_MS;
      Serial.printf("  t=%5.2f s  speed %5.1f deg/s  pos %5.1f deg\n",
                    t / 1000.0f, rampSpeedDps(t), degrees);
    }

    if (t == RAMP_MS) {
      sweepOffsetDeg = fmodf(sweepOffsetDeg + rampDistanceDeg(RAMP_MS),
                             4.0f * AMPLITUDE_DEG);
      phase = Phase::Pause;
      phaseStartMs = now;
      Serial.printf("Cycle %u: peak %.0f deg/s reached, holding at %.1f deg for %u ms\n",
                    static_cast<unsigned>(cycle),
                    MAX_SPEED_DPS,
                    degrees,
                    static_cast<unsigned>(PAUSE_MS));
    }
  } else if (elapsed >= PAUSE_MS) {
    phase = Phase::Ramp;
    phaseStartMs = nextReportMs = now;
    ++cycle;
    Serial.printf("Cycle %u: ramping 0 -> %.0f deg/s over %u ms\n",
                  static_cast<unsigned>(cycle),
                  MAX_SPEED_DPS,
                  static_cast<unsigned>(RAMP_MS));
  }
}
