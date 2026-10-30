#include <Arduino.h>
#include <esp_arduino_version.h>

#if ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 3, 11)
#error "Install esp32 by Espressif Systems 3.3.11 or newer."
#endif

constexpr int DATA_PIN = 33;  // G33; use 26 for Grove yellow.
constexpr uint32_t SERVO_BAUD = 1000000;

constexpr int MIN_DEG = 75;
constexpr int MAX_DEG = 135;
constexpr int START_DEG = 105;
constexpr int STEP_DEG = 10;

constexpr uint16_t MOVE_MS = 500;
constexpr uint32_t STEP_PERIOD_MS = 500;

HardwareSerial ServoBus(1);

int targetDeg = START_DEG;
int direction = 1;
uint32_t nextStepMs = 0;

// SC15: 0..1023 position counts cover approximately 0..210 degrees.
uint16_t degreesToPosition(int degrees) {
  return static_cast<uint16_t>((degrees * 1023L + 105) / 210);
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

void moveTo(int degrees, uint16_t durationMs) {
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

  Serial.printf("Command: %d deg, position %u, time %u ms\n",
                degrees,
                static_cast<unsigned>(position),
                static_cast<unsigned>(durationMs));
}

void setup() {
  Serial.begin(115200);

  // Same RX and TX pin selects open-drain one-wire UART.
  // Keep the external pull-up to 3.3V.
  // Do not call pinMode() on DATA_PIN after this.
  ServoBus.begin(SERVO_BAUD, SERIAL_8N1, DATA_PIN, DATA_PIN);

  delay(1000);  // Allow the servo to finish powering up.

  // Load the initial target before enabling torque.
  moveTo(START_DEG, 2000);

  const uint8_t torqueOn = 1;
  servoWrite(0x28, &torqueOn, 1);

  delay(2100);  // Allow the initial centering move to finish.
  nextStepMs = millis();
}

void loop() {
  const uint32_t now = millis();

  if (static_cast<int32_t>(now - nextStepMs) >= 0) {
    nextStepMs = now + STEP_PERIOD_MS;

    int next = targetDeg + direction * STEP_DEG;

    if (next > MAX_DEG || next < MIN_DEG) {
      direction = -direction;
      next = targetDeg + direction * STEP_DEG;
    }

    targetDeg = next;
    moveTo(targetDeg, MOVE_MS);
  }

  delay(1);
}
