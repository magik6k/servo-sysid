#include <Arduino.h>
#include <esp_arduino_version.h>

#if ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 3, 11)
#error "Install esp32 by Espressif Systems 3.3.11 or newer."
#endif

// One-shot bus diagnosis: do broadcast and addressed writes land?
// Each write is followed by a read-back of the registers it touched, and
// addressed writes also wait for the servo's status reply. The local echo
// of every packet is compared with what was sent, to catch a bad line.
// The only motion is 15% PWM for 0.2 s, twice.

constexpr int DATA_PIN = 33;  // G33; use 26 for Grove yellow.
constexpr uint32_t SERVO_BAUD = 1000000;
constexpr uint8_t SERVO_ID = 1;

HardwareSerial ServoBus(1);

void printHex(const uint8_t *data, size_t n) {
  for (size_t i = 0; i < n; ++i) {
    Serial.printf(" %02X", data[i]);
  }
}

// Sends one instruction packet, then collects whatever comes back within
// 3 ms (local echo first, then any reply). Returns the bytes received.
size_t transact(uint8_t id, uint8_t instruction, const uint8_t *params, uint8_t count,
                uint8_t *packet, uint8_t &sent, uint8_t *rx, size_t rxSize) {
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
  sent = count + 6;

  while (ServoBus.available()) {
    ServoBus.read();
  }
  ServoBus.write(packet, sent);
  ServoBus.flush(true);

  size_t n = 0;
  const uint32_t startUs = micros();
  while (n < rxSize && micros() - startUs < 3000) {
    if (ServoBus.available()) {
      rx[n++] = ServoBus.read();
    }
  }
  return n;
}

// Finds a valid status packet from id with `count` data bytes after the
// local echo. Returns its ERROR byte, or -1.
int parseReply(uint8_t id, uint8_t count, const uint8_t *packet, uint8_t sent,
               const uint8_t *rx, size_t n, uint8_t *out) {
  size_t i = n >= sent && memcmp(rx, packet, sent) == 0 ? sent : 0;
  for (; i + count + 6 <= n; ++i) {
    if (rx[i] != 0xFF || rx[i + 1] != 0xFF || rx[i + 2] != id || rx[i + 3] != count + 2) continue;
    uint8_t checksum = 0;
    for (size_t j = i + 2; j < i + 5 + count; ++j) {
      checksum += rx[j];
    }
    if (static_cast<uint8_t>(~checksum) != rx[i + 5 + count]) continue;
    if (out) memcpy(out, rx + i + 5, count);
    return rx[i + 4];
  }
  return -1;
}

const char *echoState(const uint8_t *packet, uint8_t sent, const uint8_t *rx, size_t n) {
  if (n < sent) return "echo SHORT";
  return memcmp(rx, packet, sent) == 0 ? "echo ok" : "echo BAD";
}

int readRegs(uint8_t address, uint8_t count, uint8_t *out) {
  const uint8_t params[] = {address, count};
  uint8_t packet[8], rx[96], sent;
  const size_t n = transact(SERVO_ID, 0x02, params, sizeof(params), packet, sent, rx, sizeof(rx));
  return parseReply(SERVO_ID, count, packet, sent, rx, n, out);
}

// WRITE to `id` (0xFE = broadcast). Prints the echo check and, for an
// addressed write, the status reply.
void write(uint8_t id, uint8_t address, const uint8_t *data, uint8_t count, const char *what) {
  uint8_t params[8] = {address};
  memcpy(params + 1, data, count);
  uint8_t packet[14], rx[64], sent;
  const size_t n = transact(id, 0x03, params, count + 1, packet, sent, rx, sizeof(rx));

  Serial.printf("%-34s %s", what, echoState(packet, sent, rx, n));
  if (id != 0xFE) {
    const int error = parseReply(id, 0, packet, sent, rx, n, nullptr);
    if (error < 0) {
      Serial.print(", NO status reply, rx:");
      printHex(rx, n);
    } else {
      Serial.printf(", status reply error=0x%02X", error);
    }
  }
  Serial.println();
}

void report(const char *label) {
  uint8_t limits[4], ctrl[9], state[10];
  const bool ok = readRegs(0x09, 4, limits) >= 0 &&
                  readRegs(0x28, 9, ctrl) >= 0 &&
                  readRegs(0x38, 10, state) >= 0;
  if (!ok) {
    Serial.printf("  [%s] READ FAILED\n", label);
    return;
  }

  const unsigned pwmReg = ctrl[4] << 8 | ctrl[5];
  const unsigned loadRaw = state[4] << 8 | state[5];
  Serial.printf("  [%s] limits %u/%u torque %u goal %u reg2C %s%u lock %u | pos %u load %s%u V %.1f status 0x%02X\n",
                label,
                limits[0] << 8 | limits[1], limits[2] << 8 | limits[3],
                ctrl[0],
                ctrl[2] << 8 | ctrl[3],
                pwmReg & 0x400 ? "-" : "", pwmReg & 0x3FF,
                ctrl[8],
                state[0] << 8 | state[1],
                loadRaw & 0x400 ? "-" : "", loadRaw & 0x3FF,
                state[6] / 10.0f,
                state[9]);
}

void setup() {
  Serial.begin(115200);
  ServoBus.begin(SERVO_BAUD, SERIAL_8N1, DATA_PIN, DATA_PIN);
  delay(1000);

  Serial.println("# bus_check");
  report("boot");

  const uint8_t on = 1, off = 0;
  const uint8_t motorMode[] = {0, 0, 0, 0};
  const uint8_t pwm150[] = {0x00, 150};
  const uint8_t pwm0[] = {0x00, 0x00};

  write(0xFE, 0x28, &off, 1, "broadcast torque=0");
  report("after broadcast torque=0");
  write(0xFE, 0x28, &on, 1, "broadcast torque=1");
  report("after broadcast torque=1");

  write(0xFE, 0x09, motorMode, sizeof(motorMode), "broadcast limits 0/0");
  write(0xFE, 0x2C, pwm150, sizeof(pwm150), "broadcast pwm=150");
  delay(200);
  report("broadcast pwm=150, +200 ms");
  write(0xFE, 0x2C, pwm0, sizeof(pwm0), "broadcast pwm=0");
  delay(300);

  write(SERVO_ID, 0x28, &off, 1, "id1 torque=0");
  report("after id1 torque=0");
  write(SERVO_ID, 0x28, &on, 1, "id1 torque=1");
  report("after id1 torque=1");

  write(SERVO_ID, 0x09, motorMode, sizeof(motorMode), "id1 limits 0/0");
  write(SERVO_ID, 0x2C, pwm150, sizeof(pwm150), "id1 pwm=150");
  delay(200);
  report("id1 pwm=150, +200 ms");
  write(SERVO_ID, 0x2C, pwm0, sizeof(pwm0), "id1 pwm=0");
  write(SERVO_ID, 0x28, &off, 1, "id1 torque=0 (leave safe)");
  report("end");
  Serial.println("# done");
}

void loop() {}
