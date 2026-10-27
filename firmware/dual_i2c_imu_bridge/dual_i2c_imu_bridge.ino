#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

/*
  两只 YB-MRA02 九轴 IMU 的 I2C -> USB CDC 桥接程序。

  IMU 1: SDA -> GPIO8,  SCL -> GPIO9
  IMU 2: SDA -> GPIO10, SCL -> GPIO11
  两只 IMU 都由 ESP32-S3 的 3V3 供电，并与 ESP32-S3 共地。

  协议依据“通信协议.xlsx”：
    I2C 地址       0x23
    I2C 时钟       100 kHz
    四元数首寄存器 0x16
    四元数格式     4 个小端 Float32，顺序 w, x, y, z

  USB 串口输出：
    IMU,编号,序号,ESP32时间us,qw,qx,qy,qz,模长

  可从 USB 串口发送：PING、SCAN、STATS（每条命令以换行结束）。
*/

constexpr uint8_t IMU_ADDRESS = 0x23;
constexpr uint8_t QUATERNION_REGISTER = 0x16;
constexpr uint32_t I2C_FREQUENCY = 100000;
constexpr uint32_t SAMPLE_PERIOD_US = 20000;  // 50 Hz

constexpr int IMU1_SDA_PIN = 8;
constexpr int IMU1_SCL_PIN = 9;
constexpr int IMU2_SDA_PIN = 10;
constexpr int IMU2_SCL_PIN = 11;

// LSC-16 舵机控制板：控制板 TX -> GPIO18，控制板 RX <- GPIO21。
// 舵机板必须独立供电，并与 ESP32-S3 共地。
constexpr int SERVO_RX_PIN = 18;
constexpr int SERVO_TX_PIN = 21;
constexpr uint32_t SERVO_BAUD = 9600;
// CH1 and CH3 are continuous-rotation servos. Stop them if host heartbeat
// disappears; position servos CH2/CH4 are unaffected.
constexpr uint32_t BEND_WATCHDOG_MS = 500;
uint32_t lastBendCommandMs[2] = {0, 0};
bool bendMoving[2] = {false, false};

TwoWire IMU1Bus = TwoWire(0);
TwoWire IMU2Bus = TwoWire(1);
HardwareSerial ServoSerial(2);

struct ImuState {
  uint32_t sequence = 0;
  uint32_t goodReads = 0;
  uint32_t readErrors = 0;
  uint8_t lastError = 0;
};

ImuState imu1State;
ImuState imu2State;
uint32_t lastSampleUs = 0;
char commandLine[48];
size_t commandLength = 0;

float littleEndianFloat(const uint8_t *bytes) {
  static_assert(sizeof(float) == 4, "This program requires 32-bit float");
  float value;
  memcpy(&value, bytes, sizeof(value));
  return value;
}

/*
  先写入寄存器地址，再用 repeated-start 连续读取。
  返回值：0 成功；1..4 为 Wire.endTransmission() 错误；
          0xE0 表示返回字节数不足；0xE1 表示四元数无效。
*/
uint8_t readQuaternion(TwoWire &bus, float q[4]) {
  bus.beginTransmission(IMU_ADDRESS);
  bus.write(QUATERNION_REGISTER);
  uint8_t error = bus.endTransmission(false);
  if (error != 0) return error;

  constexpr size_t bytesNeeded = 16;
  size_t received = bus.requestFrom(
      static_cast<uint16_t>(IMU_ADDRESS),
      bytesNeeded,
      true);
  if (received != bytesNeeded) {
    while (bus.available()) bus.read();
    return 0xE0;
  }

  uint8_t bytes[bytesNeeded];
  for (size_t i = 0; i < bytesNeeded; ++i) {
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

void sampleAndPrint(TwoWire &bus, uint8_t imuId, ImuState &state,
                    uint32_t timestampUs) {
  float q[4];
  uint8_t error = readQuaternion(bus, q);
  if (error != 0) {
    ++state.readErrors;
    state.lastError = error;
    return;
  }

  ++state.sequence;
  ++state.goodReads;
  state.lastError = 0;
  float qnorm = sqrtf(q[0] * q[0] + q[1] * q[1] +
                      q[2] * q[2] + q[3] * q[3]);

  Serial.printf("IMU,%u,%lu,%lu,%.7f,%.7f,%.7f,%.7f,%.7f\n",
                imuId,
                static_cast<unsigned long>(state.sequence),
                static_cast<unsigned long>(timestampUs),
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

void sendServoPosition(uint8_t id, uint16_t position, uint16_t moveMs) {
  const uint8_t frame[] = {
      0x55, 0x55, 0x08, 0x03, 0x01,
      static_cast<uint8_t>(moveMs & 0xFF),
      static_cast<uint8_t>((moveMs >> 8) & 0xFF),
      id,
      static_cast<uint8_t>(position & 0xFF),
      static_cast<uint8_t>((position >> 8) & 0xFF)
  };
  ServoSerial.write(frame, sizeof(frame));
  ServoSerial.flush();
}

void processCommand(const char *line) {
  if (strcmp(line, "PING") == 0) {
    Serial.println("PONG");
  } else if (strcmp(line, "SCAN") == 0) {
    printScan();
  } else if (strcmp(line, "STATS") == 0) {
    Serial.printf("STATS,1,ok=%lu,errors=%lu,last=0x%02X\n",
                  static_cast<unsigned long>(imu1State.goodReads),
                  static_cast<unsigned long>(imu1State.readErrors),
                  imu1State.lastError);
    Serial.printf("STATS,2,ok=%lu,errors=%lu,last=0x%02X\n",
                  static_cast<unsigned long>(imu2State.goodReads),
                  static_cast<unsigned long>(imu2State.readErrors),
                  imu2State.lastError);
  } else if (line[0] == 'S' && line[1] == ',') {
    char copy[48];
    strncpy(copy, line, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';
    char *save = nullptr;
    strtok_r(copy, ",", &save);
    char *idText = strtok_r(nullptr, ",", &save);
    char *positionText = strtok_r(nullptr, ",", &save);
    char *timeText = strtok_r(nullptr, ",", &save);
    if (!idText || !positionText || !timeText) {
      Serial.println("ERR,FORMAT");
      return;
    }
    long id = strtol(idText, nullptr, 10);
    long position = strtol(positionText, nullptr, 10);
    long moveMs = strtol(timeText, nullptr, 10);
    if (id < 0 || id > 15 || position < 500 || position > 2500 ||
        moveMs < 20 || moveMs > 5000) {
      Serial.println("ERR,RANGE");
      return;
    }
    sendServoPosition(static_cast<uint8_t>(id),
                      static_cast<uint16_t>(position),
                      static_cast<uint16_t>(moveMs));
    if (id == 1 || id == 3) {
      int index = id == 1 ? 0 : 1;
      bendMoving[index] = position != 1500;
      lastBendCommandMs[index] = millis();
    }
    Serial.printf("ACK,S,%ld,%ld\n", id, position);
  } else {
    Serial.println("ERR,UNKNOWN_COMMAND");
  }
}

void pollUsbCommands() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      commandLine[commandLength] = '\0';
      if (commandLength > 0) processCommand(commandLine);
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

  bool bus0Started = IMU1Bus.begin(
      IMU1_SDA_PIN, IMU1_SCL_PIN, I2C_FREQUENCY);
  bool bus1Started = IMU2Bus.begin(
      IMU2_SDA_PIN, IMU2_SCL_PIN, I2C_FREQUENCY);
  ServoSerial.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  sendServoPosition(1, 1500, 120);
  sendServoPosition(3, 1500, 120);

  IMU1Bus.setTimeOut(20);
  IMU2Bus.setTimeOut(20);

  delay(500);
  Serial.printf("I2C_BRIDGE_READY,bus0=%d,bus1=%d\n",
                bus0Started, bus1Started);
  printScan();
}

void loop() {
  pollUsbCommands();

  uint32_t nowMs = millis();
  for (int index = 0; index < 2; ++index) {
    if (bendMoving[index] &&
        static_cast<uint32_t>(nowMs - lastBendCommandMs[index]) > BEND_WATCHDOG_MS) {
      uint8_t channel = index == 0 ? 1 : 3;
      sendServoPosition(channel, 1500, 120);
      bendMoving[index] = false;
      Serial.printf("WATCHDOG,CH%u,1500\n", channel);
    }
  }

  uint32_t now = micros();
  if (static_cast<uint32_t>(now - lastSampleUs) >= SAMPLE_PERIOD_US) {
    lastSampleUs = now;
    sampleAndPrint(IMU1Bus, 1, imu1State, now);
    sampleAndPrint(IMU2Bus, 2, imu2State, now);
  }
}
