#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

/*
  ESP32-S3 bridge for two YB-MRA02 I2C IMUs and one FEETECH HD-1910 servo.

  IMU1: SDA GPIO8,  SCL GPIO9
  IMU2: SDA GPIO10, SCL GPIO11
  Both IMUs: address 0x23, 100 kHz, quaternion at 0x16 (little-endian
  Float32 values in w,x,y,z order).

  USB CDC to Julia: 115200 baud, newline-delimited ASCII.
  UART2 to Waveshare Bus Servo Adapter (A), 1 Mbps:
    ESP32 GPIO21 (TX) -> adapter TX
    ESP32 GPIO18 (RX) -> adapter RX
    ESP32 GND         -> adapter GND
  Waveshare's adapter labels use same-name TX-TX / RX-RX wiring; do not use
  the usual crossed UART wiring here. Set the adapter jumper to position A.

  UART1 to LSC-16 axial servo controller, 9600 baud:
    ESP32 GPIO17 (TX) -> LSC-16 RX
    ESP32 GPIO16 (RX) <- LSC-16 TX
    ESP32 GND         -> LSC-16 GND
  LSC-16 channel 1 drives section 1 axial servo; channel 2 drives section 2.

  USB commands:
    PING                 -> PONG
    SCAN                 -> scans both I2C buses
    STATS                -> IMU read counters
    Q,<id>               -> read signed multi-turn position (ticks)
    M,<id>               -> read current operating mode (0 or 4 for position)
    L,<id>               -> read angle limit registers (0,0 means multi-turn)
    U,<id>               -> enable multi-turn limits and hold current position
    P,<id>,<ticks>,<rpm> -> set speed cap and position target
    T,<id>,0|1            -> disable/enable servo torque
    A,<section>,<pulse>,<ms> -> LSC-16 axial position, section 1/2,
                                pulse 500..2500, move time 20..5000 ms

  IMU output: IMU,id,sequence,timestamp_us,qw,qx,qy,qz,norm
  Servo output: POS,id,ticks; command response: ACK,... or ERR,...

  FEETECH SCS/STS instruction packet: FF FF ID LENGTH INSTRUCTION PARAMS CHECKSUM.
  This deliberately does not send the old LSC-16 0x55 0x55 packet format.
*/

constexpr uint8_t IMU_ADDRESS = 0x23;
constexpr uint8_t QUATERNION_REGISTER = 0x16;
constexpr uint32_t I2C_FREQUENCY = 100000;
constexpr uint32_t SAMPLE_PERIOD_US = 20000;
constexpr int IMU1_SDA_PIN = 8, IMU1_SCL_PIN = 9;
constexpr int IMU2_SDA_PIN = 10, IMU2_SCL_PIN = 11;
constexpr int SERVO_RX_PIN = 18, SERVO_TX_PIN = 21;
constexpr uint32_t SERVO_BAUD = 1000000;
constexpr int AXIAL_RX_PIN = 16, AXIAL_TX_PIN = 17;
constexpr uint32_t AXIAL_BAUD = 9600;
constexpr uint8_t FEETECH_PING = 0x01, FEETECH_READ = 0x02, FEETECH_WRITE = 0x03;
constexpr uint32_t SERVO_REPLY_TIMEOUT_MS = 80;
constexpr float MAX_SERVO_RPM = 113.0f;

TwoWire IMU1Bus(0), IMU2Bus(1);
HardwareSerial ServoSerial(2);
HardwareSerial AxialSerial(1);

struct ImuState {
  uint32_t sequence = 0, goodReads = 0, readErrors = 0;
  uint8_t lastError = 0;
};
ImuState imu1State, imu2State;
uint32_t lastSampleUs = 0;
char commandLine[64];
size_t commandLength = 0;

float littleEndianFloat(const uint8_t *bytes) {
  static_assert(sizeof(float) == 4, "32-bit float required");
  float value;
  memcpy(&value, bytes, sizeof(value));
  return value;
}

uint8_t readQuaternion(TwoWire &bus, float q[4]) {
  bus.beginTransmission(IMU_ADDRESS);
  bus.write(QUATERNION_REGISTER);
  uint8_t error = bus.endTransmission(false);
  if (error) return error;
  constexpr size_t needed = 16;
  size_t received = bus.requestFrom(static_cast<uint16_t>(IMU_ADDRESS), needed, true);
  if (received != needed) {
    while (bus.available()) bus.read();
    return 0xE0;
  }
  uint8_t bytes[needed];
  for (size_t i = 0; i < needed; ++i) {
    if (!bus.available()) return 0xE0;
    bytes[i] = static_cast<uint8_t>(bus.read());
  }
  float normSquared = 0.0f;
  for (int i = 0; i < 4; ++i) {
    q[i] = littleEndianFloat(bytes + 4 * i);
    if (!isfinite(q[i]) || fabsf(q[i]) > 1.2f) return 0xE1;
    normSquared += q[i] * q[i];
  }
  float qnorm = sqrtf(normSquared);
  if (!isfinite(qnorm) || qnorm < 0.5f || qnorm > 1.5f) return 0xE1;
  for (int i = 0; i < 4; ++i) q[i] /= qnorm;
  return 0;
}

void sampleAndPrint(TwoWire &bus, uint8_t id, ImuState &state, uint32_t timestampUs) {
  float q[4];
  uint8_t error = readQuaternion(bus, q);
  if (error) {
    ++state.readErrors;
    state.lastError = error;
    return;
  }
  ++state.sequence;
  ++state.goodReads;
  state.lastError = 0;
  float qnorm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  Serial.printf("IMU,%u,%lu,%lu,%.7f,%.7f,%.7f,%.7f,%.7f\n", id,
      static_cast<unsigned long>(state.sequence), static_cast<unsigned long>(timestampUs),
      q[0], q[1], q[2], q[3], qnorm);
}

bool devicePresent(TwoWire &bus) {
  bus.beginTransmission(IMU_ADDRESS);
  return bus.endTransmission(true) == 0;
}

void printScan() {
  Serial.printf("SCAN,1,address=0x23,present=%d\n", devicePresent(IMU1Bus));
  Serial.printf("SCAN,2,address=0x23,present=%d\n", devicePresent(IMU2Bus));
}

uint8_t checksum(const uint8_t *data, size_t length) {
  uint16_t total = 0;
  for (size_t i = 0; i < length; ++i) total += data[i];
  return static_cast<uint8_t>(~total);
}

bool servoExchange(uint8_t id, uint8_t instruction, const uint8_t *params,
                   uint8_t paramCount, uint8_t *replyParams, uint8_t &replyCount) {
  while (ServoSerial.available()) ServoSerial.read();
  uint8_t frame[16];
  frame[0] = 0xFF; frame[1] = 0xFF; frame[2] = id;
  frame[3] = static_cast<uint8_t>(paramCount + 2);
  frame[4] = instruction;
  for (uint8_t i = 0; i < paramCount; ++i) frame[5 + i] = params[i];
  frame[5 + paramCount] = checksum(frame + 2, paramCount + 3);
  ServoSerial.write(frame, paramCount + 6);
  ServoSerial.flush();

  uint8_t input[32];
  size_t used = 0;
  uint32_t started = millis();
  while (static_cast<uint32_t>(millis() - started) < SERVO_REPLY_TIMEOUT_MS) {
    while (ServoSerial.available() && used < sizeof(input)) input[used++] = ServoSerial.read();
    for (size_t i = 0; i + 5 <= used; ++i) {
      if (input[i] != 0xFF || input[i+1] != 0xFF || input[i+2] != id) continue;
      uint8_t length = input[i+3];
      size_t total = static_cast<size_t>(length) + 4;
      if (length < 2 || total > sizeof(input)) continue;
      if (i + total > used) break;
      uint8_t sum = 0;
      for (size_t j = i + 2; j < i + total; ++j) sum += input[j];
      if (sum != 0xFF || input[i+4] != 0) return false;
      replyCount = static_cast<uint8_t>(length - 2);
      if (replyCount > 0 && replyParams) {
        for (uint8_t j = 0; j < replyCount; ++j) replyParams[j] = input[i+5+j];
      }
      return true;
    }
    delay(1);
  }
  return false;
}

bool writeRegister(uint8_t id, uint8_t address, const uint8_t *values, uint8_t count) {
  uint8_t params[8];
  if (count > sizeof(params) - 1) return false;
  params[0] = address;
  for (uint8_t i = 0; i < count; ++i) params[i+1] = values[i];
  uint8_t response[8], responseCount = 0;
  return servoExchange(id, FEETECH_WRITE, params, count + 1, response, responseCount);
}

bool readRegister(uint8_t id, uint8_t address, uint8_t count, uint8_t *values) {
  uint8_t params[2] = {address, count};
  uint8_t responseCount = 0;
  return servoExchange(id, FEETECH_READ, params, sizeof(params), values, responseCount) &&
         responseCount == count;
}

bool readPosition(uint8_t id, int16_t &position) {
  uint8_t data[2];
  if (!readRegister(id, 0x38, 2, data)) return false;
  uint16_t raw = static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
  position = (raw & 0x8000) ? -static_cast<int16_t>(raw & 0x7FFF) : static_cast<int16_t>(raw);
  return true;
}

bool writePositionGoal(uint8_t id, int16_t position) {
  uint16_t magnitude = static_cast<uint16_t>(abs(static_cast<int>(position)));
  uint16_t raw = magnitude | (position < 0 ? 0x8000 : 0);
  uint8_t data[2] = {static_cast<uint8_t>(raw & 0xFF),
                     static_cast<uint8_t>((raw >> 8) & 0xFF)};
  return writeRegister(id, 0x2A, data, 2);
}

// Mode 4 is position PD. Multi-turn travel is selected separately by zeroing
// both angle-limit registers. Keep the actual shaft position as the goal while
// torque is briefly disabled, then restore the EEPROM lock and enable holding.
bool enableMultiTurn(uint8_t id, int16_t &heldPosition) {
  uint8_t mode, limits[4], oldLock, oldTorque;
  if (!readRegister(id, 0x21, 1, &mode) || (mode != 0 && mode != 4) ||
      !readRegister(id, 0x09, 4, limits) ||
      !readRegister(id, 0x37, 1, &oldLock) ||
      !readRegister(id, 0x28, 1, &oldTorque) ||
      !readPosition(id, heldPosition)) return false;
  const uint8_t off = 0, on = 1;
  bool torqueDisabled = false, unlocked = false, ok = true;
  if (!(limits[0] == 0 && limits[1] == 0 && limits[2] == 0 && limits[3] == 0)) {
    ok = writeRegister(id, 0x28, &off, 1);
    torqueDisabled = ok;
    if (ok) ok = writePositionGoal(id, heldPosition);
    if (ok) { ok = writeRegister(id, 0x37, &off, 1); unlocked = ok; }
    if (ok) { uint8_t zeros[4] = {0, 0, 0, 0}; ok = writeRegister(id, 0x09, zeros, 4); }
    if (ok) { delay(20); ok = readRegister(id, 0x09, 4, limits); }
    if (ok) ok = limits[0] == 0 && limits[1] == 0 && limits[2] == 0 && limits[3] == 0;
  }
  if (unlocked) ok = writeRegister(id, 0x37, &oldLock, 1) && ok;
  if (ok) ok = writePositionGoal(id, heldPosition);
  if (ok) ok = writeRegister(id, 0x28, &on, 1);
  if (!ok && torqueDisabled) {
    writePositionGoal(id, heldPosition);
    writeRegister(id, 0x28, &oldTorque, 1);
  }
  int16_t after;
  if (ok) ok = readPosition(id, after) && abs(static_cast<int>(after) - heldPosition) <= 30;
  return ok;
}

void sendAxialPosition(uint8_t channel, uint16_t position, uint16_t moveMs) {
  const uint8_t frame[] = {
      0x55, 0x55, 0x08, 0x03, 0x01,
      static_cast<uint8_t>(moveMs & 0xFF),
      static_cast<uint8_t>((moveMs >> 8) & 0xFF),
      channel,
      static_cast<uint8_t>(position & 0xFF),
      static_cast<uint8_t>((position >> 8) & 0xFF)
  };
  AxialSerial.write(frame, sizeof(frame));
  AxialSerial.flush();
}

void processCommand(char *line) {
  if (!strcmp(line, "PING")) { Serial.println("PONG"); return; }
  if (!strcmp(line, "SCAN")) { printScan(); return; }
  if (!strcmp(line, "STATS")) {
    Serial.printf("STATS,1,ok=%lu,errors=%lu,last=0x%02X\n", (unsigned long)imu1State.goodReads, (unsigned long)imu1State.readErrors, imu1State.lastError);
    Serial.printf("STATS,2,ok=%lu,errors=%lu,last=0x%02X\n", (unsigned long)imu2State.goodReads, (unsigned long)imu2State.readErrors, imu2State.lastError);
    return;
  }
  char *save = nullptr;
  char *op = strtok_r(line, ",", &save);
  char *idText = strtok_r(nullptr, ",", &save);
  if (!op || !idText) { Serial.println("ERR,FORMAT"); return; }
  if (!strcmp(op, "A")) {
    char *positionText = strtok_r(nullptr, ",", &save);
    char *timeText = strtok_r(nullptr, ",", &save);
    if (!positionText || !timeText || strtok_r(nullptr, ",", &save)) {
      Serial.println("ERR,FORMAT"); return;
    }
    char *endId = nullptr, *endPosition = nullptr, *endTime = nullptr;
    long channel = strtol(idText, &endId, 10);
    long position = strtol(positionText, &endPosition, 10);
    long moveMs = strtol(timeText, &endTime, 10);
    if (*endId || *endPosition || *endTime || channel < 1 || channel > 2 ||
        position < 500 || position > 2500 || moveMs < 20 || moveMs > 5000) {
      Serial.println("ERR,RANGE"); return;
    }
    sendAxialPosition(static_cast<uint8_t>(channel),
                      static_cast<uint16_t>(position),
                      static_cast<uint16_t>(moveMs));
    Serial.printf("ACK,A,%ld,%ld,%ld\n", channel, position, moveMs);
    return;
  }
  long idValue = strtol(idText, nullptr, 10);
  if (idValue < 0 || idValue > 253) { Serial.println("ERR,ID"); return; }
  uint8_t id = static_cast<uint8_t>(idValue);
  if (!strcmp(op, "Q")) {
    int16_t position;
    if (!readPosition(id, position)) { Serial.printf("ERR,SERVO_TIMEOUT,%u\n", id); return; }
    Serial.printf("POS,%u,%d\n", id, position);
    return;
  }
  if (!strcmp(op, "M")) {
    uint8_t mode;
    if (!readRegister(id, 0x21, 1, &mode)) { Serial.printf("ERR,SERVO_TIMEOUT,%u\n", id); return; }
    Serial.printf("MODE,%u,%u\n", id, mode);
    return;
  }
  if (!strcmp(op, "L")) {
    uint8_t limits[4];
    if (!readRegister(id, 0x09, 4, limits)) { Serial.printf("ERR,SERVO_TIMEOUT,%u\n", id); return; }
    uint16_t minimum = static_cast<uint16_t>(limits[0]) |
                       (static_cast<uint16_t>(limits[1]) << 8);
    uint16_t maximum = static_cast<uint16_t>(limits[2]) |
                       (static_cast<uint16_t>(limits[3]) << 8);
    Serial.printf("LIMIT,%u,%u,%u\n", id, minimum, maximum);
    return;
  }
  if (!strcmp(op, "U")) {
    int16_t position;
    if (!enableMultiTurn(id, position)) { Serial.printf("ERR,MULTITURN,%u\n", id); return; }
    Serial.printf("ACK,U,%u,%d\n", id, position);
    return;
  }
  if (!strcmp(op, "T")) {
    char *onText = strtok_r(nullptr, ",", &save);
    if (!onText) { Serial.println("ERR,FORMAT"); return; }
    long enabled = strtol(onText, nullptr, 10);
    if (enabled != 0 && enabled != 1) { Serial.println("ERR,RANGE"); return; }
    uint8_t value = static_cast<uint8_t>(enabled);
    if (!writeRegister(id, 0x28, &value, 1)) { Serial.printf("ERR,SERVO_TIMEOUT,%u\n", id); return; }
    Serial.printf("ACK,T,%u,%u\n", id, value);
    return;
  }
  if (!strcmp(op, "P")) {
    char *targetText = strtok_r(nullptr, ",", &save);
    char *rpmText = strtok_r(nullptr, ",", &save);
    if (!targetText || !rpmText) { Serial.println("ERR,FORMAT"); return; }
    long target = strtol(targetText, nullptr, 10);
    float rpm = strtof(rpmText, nullptr);
    if (target < -32767 || target > 32767 || !isfinite(rpm) || rpm <= 0 || rpm > MAX_SERVO_RPM) {
      Serial.println("ERR,RANGE"); return;
    }
    long speedUnits = lroundf(rpm / 0.732f);
    if (speedUnits < 1) speedUnits = 1;
    uint8_t speedData[2] = {static_cast<uint8_t>(speedUnits & 0xFF), static_cast<uint8_t>((speedUnits >> 8) & 0xFF)};
    uint16_t magnitude = static_cast<uint16_t>(labs(target));
    uint16_t rawTarget = magnitude | (target < 0 ? 0x8000 : 0);
    uint8_t targetData[2] = {static_cast<uint8_t>(rawTarget & 0xFF), static_cast<uint8_t>((rawTarget >> 8) & 0xFF)};
    if (!writeRegister(id, 0x2E, speedData, 2)) { Serial.printf("ERR,SPEED_WRITE,%u\n", id); return; }
    if (!writeRegister(id, 0x2A, targetData, 2)) { Serial.printf("ERR,TARGET_WRITE,%u\n", id); return; }
    Serial.printf("ACK,P,%u,%ld,%.2f\n", id, target, rpm);
    return;
  }
  Serial.println("ERR,UNKNOWN_COMMAND");
}

void pollUsbCommands() {
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      commandLine[commandLength] = '\0';
      if (commandLength) processCommand(commandLine);
      commandLength = 0;
    } else if (commandLength + 1 < sizeof(commandLine)) {
      commandLine[commandLength++] = c;
    } else {
      commandLength = 0;
      Serial.println("ERR,LINE_TOO_LONG");
    }
  }
}

void setup() {
  Serial.begin(115200);
  IMU1Bus.begin(IMU1_SDA_PIN, IMU1_SCL_PIN, I2C_FREQUENCY);
  IMU2Bus.begin(IMU2_SDA_PIN, IMU2_SCL_PIN, I2C_FREQUENCY);
  IMU1Bus.setTimeOut(20); IMU2Bus.setTimeOut(20);
  ServoSerial.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  AxialSerial.begin(AXIAL_BAUD, SERIAL_8N1, AXIAL_RX_PIN, AXIAL_TX_PIN);
  delay(500);
  Serial.println("FEETECH_IMU_BRIDGE_READY,uart=1000000");
  printScan();
}

void loop() {
  pollUsbCommands();
  uint32_t now = micros();
  if (static_cast<uint32_t>(now - lastSampleUs) >= SAMPLE_PERIOD_US) {
    lastSampleUs = now;
    sampleAndPrint(IMU1Bus, 1, imu1State, now);
    sampleAndPrint(IMU2Bus, 2, imu2State, now);
  }
}
