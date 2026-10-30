#include <Arduino.h>
#include <esp_arduino_version.h>

#if ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 3, 11)
#error "Install esp32 by Espressif Systems 3.3.11 or newer."
#endif

constexpr int DATA_PIN = 33;  // G33; use 26 for Grove yellow.
constexpr uint32_t SERVO_BAUD = 1000000;

constexpr int16_t STEP_DUTY = 100;   // 10% per level
constexpr int16_t MAX_DUTY = 1000;   // 100%
constexpr uint32_t SEGMENT_MS = 1300;
constexpr uint32_t PAUSE_MS = 1000;

HardwareSerial ServoBus(1);

int16_t level = STEP_DUTY;

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

// duty: -1000..1000 in 0.1% steps; negative spins in reverse.
void spin(int16_t duty) {
  // The servo slews its PWM output at ~1.25 units/ms, so a full reversal
  // would take ~1.6 s. Bouncing through position mode resets that ramp
  // to 0, leaving only the climb from 0 (~0.8 s to full).
  const uint8_t positionMode[] = {0x00, 0x00, 0x03, 0xFF};
  servoWrite(0x09, positionMode, sizeof(positionMode));

  // Motor (continuous rotation) mode: min and max angle limits both 0.
  // Not saved with the EEPROM lock on, so it is re-sent every time
  // in case the servo reset since the last command.
  const uint8_t motorMode[] = {0x00, 0x00, 0x00, 0x00};
  servoWrite(0x09, motorMode, sizeof(motorMode));

  // In motor mode 0x2C holds PWM duty; bit 10 selects reverse.
  const uint16_t raw = duty < 0
    ? static_cast<uint16_t>(-duty) | 0x0400
    : static_cast<uint16_t>(duty);

  // SC-series values are big-endian: high byte first.
  const uint8_t data[] = {
    static_cast<uint8_t>(raw >> 8),
    static_cast<uint8_t>(raw)
  };
  servoWrite(0x2C, data, sizeof(data));

  const uint8_t torqueOn = 1;
  servoWrite(0x28, &torqueOn, 1);

  Serial.printf("%8u ms  duty %6.1f%%\n",
                static_cast<unsigned>(millis()),
                duty / 10.0f);
}

void setup() {
  Serial.begin(115200);

  // Same RX and TX pin selects open-drain one-wire UART.
  // Keep the external pull-up to 3.3V.
  // Do not call pinMode() on DATA_PIN after this.
  ServoBus.begin(SERVO_BAUD, SERIAL_8N1, DATA_PIN, DATA_PIN);

  delay(1000);  // Allow the servo to finish powering up.
}

void loop() {
  Serial.printf("Level %d%%\n", level / 10);

  spin(level);
  delay(SEGMENT_MS);

  spin(-level);  // Bounce 1
  delay(SEGMENT_MS);

  spin(level);   // Bounce 2
  delay(SEGMENT_MS);

  spin(0);       // Bounce to a stop: output drops to 0 at once.
  delay(PAUSE_MS);

  level = level >= MAX_DUTY ? STEP_DUTY : level + STEP_DUTY;
}
