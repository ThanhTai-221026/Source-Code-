#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ========================================================
// WIFI / MQTT
// ========================================================
const char* ssid_car     = "Tuiineee";
const char* password_car = "9876543210";
const char* mqtt_server  = "broker.hivemq.com";
const uint16_t mqtt_port = 1883;

const char* TOPIC_JOYSTICK = "robot/joystick_cmd";
const char* TOPIC_SERVO    = "robot/servo_cmd";
const char* TOPIC_TELEMETRY = "robot/telemetry";
const char* TOPIC_STATUS    = "robot/status";

WiFiClient espClient;
PubSubClient client(espClient);

// ========================================================
// ESP32-S3 PINOUT - MOTOR
// ========================================================
// L298N #1 - 2 BÁNH TRÁI
const int ENA_FL = 4;
const int IN1_FL = 5;
const int IN2_FL = 6;

const int ENB_RL = 7;
const int IN3_RL = 15;
const int IN4_RL = 16;

// L298N #2 - 2 BÁNH PHẢI
const int ENA_FR = 17;
const int IN5_FR = 18;
const int IN6_FR = 8;

const int ENB_RR = 9;
const int IN7_RR = 10;
const int IN8_RR = 11;

// ========================================================
// ENCODER
// ========================================================
const int ENC_A_FL = 12;
const int ENC_B_FL = 13;
const int ENC_A_FR = 14;
const int ENC_B_FR = 21;
const int ENC_A_RL = 38;
const int ENC_B_RL = 39;
const int ENC_A_RR = 40;
const int ENC_B_RR = 41;

// ========================================================
// PCA9685 - SERVO
// ========================================================
const int I2C_SDA = 42;
const int I2C_SCL = 47;
const uint8_t PCA9685_ADDR = 0x40;

Adafruit_PWMServoDriver pwm(PCA9685_ADDR);
bool pcaReady = false;

// Kênh servo trên PCA9685
const uint8_t CH_BASE     = 0;
const uint8_t CH_SHOULDER = 1;
const uint8_t CH_ELBOW    = 2;
const uint8_t CH_GRIPPER  = 3;

// PCA9685 12-bit, 50 Hz = chu kỳ 20 ms.
// Giữ trong vùng an toàn ban đầu để tránh đẩy servo vào cơ khí ở biên.
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2400;
const int SERVO_CENTER = 90;

int servoBase = SERVO_CENTER;
int servoShoulder = SERVO_CENTER;
int servoElbow = SERVO_CENTER;
int servoGripper = SERVO_CENTER;

// ========================================================
// MOTOR CONTROL
// ========================================================
const int freq = 5000;
const int resolution = 8;
const float PULSES_PER_REV = 330.0;
const float WHEEL_DIAMETER_METERS = 0.065;

volatile unsigned long ticks_FL = 0;
volatile unsigned long ticks_FR = 0;
volatile unsigned long ticks_RL = 0;
volatile unsigned long ticks_RR = 0;

double currentRPM_FL = 0;
double currentRPM_FR = 0;
double currentRPM_RL = 0;
double currentRPM_RR = 0;

double filteredRPM_FL = 0;
double filteredRPM_FR = 0;
double filteredRPM_RL = 0;
double filteredRPM_RR = 0;

double targetRPM_FL = 0;
double targetRPM_FR = 0;
double targetRPM_RL = 0;
double targetRPM_RR = 0;

double rampRPM_FL = 0;
double rampRPM_FR = 0;
double rampRPM_RL = 0;
double rampRPM_RR = 0;

float currentSpeedKmh = 0.0f;

// PID đang giữ giống hệ thống cũ, chỉ bổ sung reset rõ ràng khi STOP.
const double Kp = 0.80;
const double Ki = 0.20;
const double Kd = 0.01;

double integral_FL = 0, lastError_FL = 0;
double integral_FR = 0, lastError_FR = 0;
double integral_RL = 0, lastError_RL = 0;
double integral_RR = 0, lastError_RR = 0;

unsigned long lastTime = 0;
unsigned long lastCommandTime = 0;
unsigned long lastMQTTReconnect = 0;
unsigned long lastTelemetryTime = 0;
unsigned long lastStatusTime = 0;
const int controlInterval = 30;

// ========================================================
// ENCODER ISR
// ========================================================
void IRAM_ATTR isr_FL() { ticks_FL++; }
void IRAM_ATTR isr_FR() { ticks_FR++; }
void IRAM_ATTR isr_RL() { ticks_RL++; }
void IRAM_ATTR isr_RR() { ticks_RR++; }

// ========================================================
// SERVO HELPERS
// ========================================================
int angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);

  // 50 Hz => 20,000 us / 4096 ticks.
  // pulse = us * 4096 / 20000
  long pulse = map(angle, 0, 180,
                   (long)(SERVO_MIN_US * 4096L / 20000L),
                   (long)(SERVO_MAX_US * 4096L / 20000L));

  return constrain((int)pulse, 0, 4095);
}

void setServoAngle(uint8_t channel, int angle) {
  if (!pcaReady) return;

  angle = constrain(angle, 0, 180);
  int pulse = angleToPulse(angle);
  pwm.setPWM(channel, 0, pulse);
}

void setAllServosCenter() {
  servoBase = SERVO_CENTER;
  servoShoulder = SERVO_CENTER;
  servoElbow = SERVO_CENTER;
  servoGripper = SERVO_CENTER;

  setServoAngle(CH_BASE, servoBase);
  setServoAngle(CH_SHOULDER, servoShoulder);
  setServoAngle(CH_ELBOW, servoElbow);
  setServoAngle(CH_GRIPPER, servoGripper);
}

void scanI2C() {
  Serial.println("\n--- I2C SCAN ---");
  uint8_t found = 0;

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.printf("Tim thay I2C: 0x%02X\n", address);
      found++;
    }
  }

  if (found == 0) {
    Serial.println("KHONG TIM THAY THIET BI I2C!");
  }
  Serial.println("--- END I2C SCAN ---\n");
}

void initPCA9685() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000); // 100 kHz ổn định cho PCA9685
  delay(50);

  scanI2C();

  Wire.beginTransmission(PCA9685_ADDR);
  uint8_t error = Wire.endTransmission();

  if (error != 0) {
    pcaReady = false;
    Serial.printf("LOI: PCA9685 0x%02X khong phan hoi, error=%u\n",
                  PCA9685_ADDR, error);
    return;
  }

  // Không ép oscillator 27 MHz; module PCA9685 thông dụng dùng 25 MHz mặc định.
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(100);
  pcaReady = true;

  Serial.println("OK: PCA9685 da san sang @ 50 Hz");

  // Đưa servo về trung tâm sau khi PCA9685 sẵn sàng.
  setAllServosCenter();
}

void printServoState() {
  Serial.printf("SERVO | Base:%d  Shoulder:%d  Elbow:%d  Gripper:%d | PCA:%s\n",
                servoBase,
                servoShoulder,
                servoElbow,
                servoGripper,
                pcaReady ? "OK" : "LOI");
}

// ========================================================
// MOTOR HELPERS
// ========================================================
void resetPID() {
  integral_FL = integral_FR = integral_RL = integral_RR = 0;
  lastError_FL = lastError_FR = lastError_RL = lastError_RR = 0;
}

void updateMotorHardwareDirection(int leftDir, int rightDir) {
  // LEFT
  if (leftDir == 1) {
    digitalWrite(IN1_FL, HIGH);
    digitalWrite(IN2_FL, LOW);
    digitalWrite(IN3_RL, HIGH);
    digitalWrite(IN4_RL, LOW);
  }
  else if (leftDir == -1) {
    digitalWrite(IN1_FL, LOW);
    digitalWrite(IN2_FL, HIGH);
    digitalWrite(IN3_RL, LOW);
    digitalWrite(IN4_RL, HIGH);
  }
  else {
    digitalWrite(IN1_FL, LOW);
    digitalWrite(IN2_FL, LOW);
    digitalWrite(IN3_RL, LOW);
    digitalWrite(IN4_RL, LOW);
  }

  // RIGHT
  if (rightDir == 1) {
    digitalWrite(IN5_FR, HIGH);
    digitalWrite(IN6_FR, LOW);
    digitalWrite(IN7_RR, HIGH);
    digitalWrite(IN8_RR, LOW);
  }
  else if (rightDir == -1) {
    digitalWrite(IN5_FR, LOW);
    digitalWrite(IN6_FR, HIGH);
    digitalWrite(IN7_RR, LOW);
    digitalWrite(IN8_RR, HIGH);
  }
  else {
    digitalWrite(IN5_FR, LOW);
    digitalWrite(IN6_FR, LOW);
    digitalWrite(IN7_RR, LOW);
    digitalWrite(IN8_RR, LOW);
  }
}

void stopMotors() {
  targetRPM_FL = 0;
  targetRPM_FR = 0;
  targetRPM_RL = 0;
  targetRPM_RR = 0;

  rampRPM_FL = 0;
  rampRPM_FR = 0;
  rampRPM_RL = 0;
  rampRPM_RR = 0;

  updateMotorHardwareDirection(0, 0);
  ledcWrite(ENA_FL, 0);
  ledcWrite(ENB_RL, 0);
  ledcWrite(ENA_FR, 0);
  ledcWrite(ENB_RR, 0);
  resetPID();
}

void processMotion(int x, int y) {
  lastCommandTime = millis();

  if (abs(x) < 1200) x = 0;
  if (abs(y) < 1200) y = 0;

  if (x == 0 && y == 0) {
    stopMotors();
    return;
  }

  const double straightRPM = 180.0;
  const double turnRPM = straightRPM * 0.90;

  if (x > 0 && abs(x) >= abs(y)) {
    updateMotorHardwareDirection(1, 1);
    targetRPM_FL = straightRPM;
    targetRPM_RL = straightRPM;
    targetRPM_FR = straightRPM;
    targetRPM_RR = straightRPM;
  }
  else if (x < 0 && abs(x) >= abs(y)) {
    updateMotorHardwareDirection(-1, -1);
    targetRPM_FL = straightRPM;
    targetRPM_RL = straightRPM;
    targetRPM_FR = straightRPM;
    targetRPM_RR = straightRPM;
  }
  else if (y > 0 && abs(y) > abs(x)) {
    updateMotorHardwareDirection(1, -1);
    targetRPM_FL = turnRPM;
    targetRPM_RL = turnRPM;
    targetRPM_FR = turnRPM;
    targetRPM_RR = turnRPM;
  }
  else if (y < 0 && abs(y) > abs(x)) {
    updateMotorHardwareDirection(-1, 1);
    targetRPM_FL = turnRPM;
    targetRPM_RL = turnRPM;
    targetRPM_FR = turnRPM;
    targetRPM_RR = turnRPM;
  }
}

// ========================================================
// SERVO MQTT COMMAND
// ========================================================
void processServoCommand(String msg) {
  msg.trim();
  int colonIndex = msg.indexOf(':');

  if (colonIndex <= 0) return;

  String joint = msg.substring(0, colonIndex);
  int val = msg.substring(colonIndex + 1).toInt();
  val = constrain(val, 0, 180);

  if (!pcaReady) {
    Serial.printf("Bo qua lenh servo (%s:%d): PCA9685 chua san sang\n",
                  joint.c_str(), val);
    return;
  }

  if (joint == "base") {
    servoBase = val;
    setServoAngle(CH_BASE, servoBase);
  }
  else if (joint == "shoulder") {
    servoShoulder = val;
    setServoAngle(CH_SHOULDER, servoShoulder);
  }
  else if (joint == "elbow") {
    servoElbow = val;
    setServoAngle(CH_ELBOW, servoElbow);
  }
  else if (joint == "gripper") {
    servoGripper = val;
    setServoAngle(CH_GRIPPER, servoGripper);
  }
  else {
    return;
  }

  Serial.printf("Servo cmd: %s:%d\n", joint.c_str(), val);
}

// ========================================================
// MQTT CALLBACK
// ========================================================
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  message.reserve(length + 1);

  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  String topicStr = String(topic);

  if (topicStr == TOPIC_JOYSTICK) {
    int commaIndex = message.indexOf(',');

    if (commaIndex > 0) {
      int xVal = message.substring(0, commaIndex).toInt();
      int yVal = message.substring(commaIndex + 1).toInt();
      processMotion(xVal, yVal);
    }
  }
  else if (topicStr == TOPIC_SERVO) {
    processServoCommand(message);
  }
}

// ========================================================
// MQTT / WIFI
// ========================================================
void publishStatus() {
  if (!client.connected()) return;

  String status = String("{\"pca9685\":") +
                  (pcaReady ? "true" : "false") +
                  ",\"wifi\":true}";

  client.publish(TOPIC_STATUS, status.c_str(), true);
}

void handleMQTTReconnect() {
  if (client.connected()) {
    client.loop();
    return;
  }

  unsigned long now = millis();

  if (now - lastMQTTReconnect < 5000) return;
  lastMQTTReconnect = now;

  String clientId = "RescueCar-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  clientId += "-";
  clientId += String(random(0xFFFF), HEX);

  Serial.print("Dang ket noi MQTT... ");

  if (client.connect(clientId.c_str())) {
    Serial.println("OK");
    client.subscribe(TOPIC_JOYSTICK);
    client.subscribe(TOPIC_SERVO);
    publishStatus();
  }
  else {
    Serial.printf("FAIL, state=%d\n", client.state());
  }
}

void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.println("WiFi mat ket noi -> dang ket noi lai...");
  WiFi.disconnect();
  WiFi.begin(ssid_car, password_car);
}

// ========================================================
// PID / RAMP
// ========================================================
int computePID(double target, double current, double &integral, double &lastError) {
  if (target <= 2.0) {
    integral = 0;
    lastError = 0;
    return 0;
  }

  double minPWM = 65.0;
  double feedforward = minPWM + (target / 180.0) * (255.0 - minPWM);
  double error = target - current;

  integral = constrain(integral + error, -50.0, 50.0);
  double derivative = error - lastError;

  double output = feedforward
                + (Kp * error)
                + (Ki * integral)
                + (Kd * derivative);

  lastError = error;
  return constrain((int)output, 0, 255);
}

double approachSmooth(double target, double current, double step) {
  if (target == 0) return 0;
  if (current < target) return min(current + step, target);
  if (current > target) return max(current - step, target);
  return target;
}

// ========================================================
// SETUP
// ========================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("========================================");
  Serial.println("       RESCUE VEHICLE ESP32-S3");
  Serial.println("========================================");

  // ---------------- MOTOR PIN ----------------
  pinMode(IN1_FL, OUTPUT);
  pinMode(IN2_FL, OUTPUT);
  pinMode(IN3_RL, OUTPUT);
  pinMode(IN4_RL, OUTPUT);
  pinMode(IN5_FR, OUTPUT);
  pinMode(IN6_FR, OUTPUT);
  pinMode(IN7_RR, OUTPUT);
  pinMode(IN8_RR, OUTPUT);

  // Bắt đầu ở trạng thái STOP
  updateMotorHardwareDirection(0, 0);

  // Arduino-ESP32 core 3.x: ledcAttach/ledcWrite dùng PIN.
  bool ok1 = ledcAttach(ENA_FL, freq, resolution);
  bool ok2 = ledcAttach(ENB_RL, freq, resolution);
  bool ok3 = ledcAttach(ENA_FR, freq, resolution);
  bool ok4 = ledcAttach(ENB_RR, freq, resolution);

  Serial.printf("LEDC: FL=%s RL=%s FR=%s RR=%s\n",
                ok1 ? "OK" : "FAIL",
                ok2 ? "OK" : "FAIL",
                ok3 ? "OK" : "FAIL",
                ok4 ? "OK" : "FAIL");

  ledcWrite(ENA_FL, 0);
  ledcWrite(ENB_RL, 0);
  ledcWrite(ENA_FR, 0);
  ledcWrite(ENB_RR, 0);

  // ---------------- PCA9685 ----------------
  initPCA9685();
  printServoState();

  // ---------------- ENCODER ----------------
  pinMode(ENC_A_FL, INPUT_PULLUP);
  pinMode(ENC_A_FR, INPUT_PULLUP);
  pinMode(ENC_A_RL, INPUT_PULLUP);
  pinMode(ENC_A_RR, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A_FL), isr_FL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_FR), isr_FR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_RL), isr_RL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_RR), isr_RR, RISING);

  // ---------------- WIFI ----------------
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid_car, password_car);

  Serial.print("Dang ket noi WiFi");

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 20000) {
    delay(300);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi DA KET NOI");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  }
  else {
    Serial.println();
    Serial.println("WiFi CHUA KET NOI - se tu thu lai trong loop");
  }

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setKeepAlive(30);
  client.setBufferSize(256);

  lastTime = millis();
  lastCommandTime = millis();

  Serial.println("Khoi dong hoan tat.");
  Serial.println("Neu servo khong gong: xem I2C scan va nguon V+ PCA9685.");
}

// ========================================================
// LOOP
// ========================================================
void loop() {
  ensureWiFi();
  handleMQTTReconnect();

  // Không nhận lệnh xe > 2 giây => STOP vì an toàn.
  if (millis() - lastCommandTime > 2000) {
    if (targetRPM_FL != 0 || targetRPM_FR != 0 ||
        targetRPM_RL != 0 || targetRPM_RR != 0) {
      stopMotors();
    }
  }

  // ---------------- MOTOR + ENCODER ----------------
  unsigned long currentTime = millis();

  if (currentTime - lastTime >= controlInterval) {
    float dt = (currentTime - lastTime) / 1000.0f;
    lastTime = currentTime;

    noInterrupts();
    unsigned long tFL = ticks_FL;
    unsigned long tFR = ticks_FR;
    unsigned long tRL = ticks_RL;
    unsigned long tRR = ticks_RR;
    ticks_FL = 0;
    ticks_FR = 0;
    ticks_RL = 0;
    ticks_RR = 0;
    interrupts();

    currentRPM_FL = ((float)tFL / PULSES_PER_REV) / dt * 60.0;
    currentRPM_FR = ((float)tFR / PULSES_PER_REV) / dt * 60.0;
    currentRPM_RL = ((float)tRL / PULSES_PER_REV) / dt * 60.0;
    currentRPM_RR = ((float)tRR / PULSES_PER_REV) / dt * 60.0;

    // Lọc nhiễu encoder RL, giữ lại logic cũ của bạn.
    if (currentRPM_RL > 140.0) {
      currentRPM_RL = filteredRPM_RL;
    }

    filteredRPM_FL = filteredRPM_FL * 0.5 + currentRPM_FL * 0.5;
    filteredRPM_FR = filteredRPM_FR * 0.5 + currentRPM_FR * 0.5;
    filteredRPM_RL = filteredRPM_RL * 0.5 + currentRPM_RL * 0.5;
    filteredRPM_RR = filteredRPM_RR * 0.5 + currentRPM_RR * 0.5;

    currentSpeedKmh = (filteredRPM_FL * M_PI * WHEEL_DIAMETER_METERS * 60.0) / 1000.0;

    rampRPM_FL = approachSmooth(targetRPM_FL, rampRPM_FL, 100.0);
    rampRPM_FR = approachSmooth(targetRPM_FR, rampRPM_FR, 100.0);
    rampRPM_RL = approachSmooth(targetRPM_RL, rampRPM_RL, 100.0);
    rampRPM_RR = approachSmooth(targetRPM_RR, rampRPM_RR, 100.0);

    int pwmFL = computePID(rampRPM_FL, filteredRPM_FL, integral_FL, lastError_FL);
    int pwmFR = computePID(rampRPM_FR, filteredRPM_FR, integral_FR, lastError_FR);
    int pwmRL = computePID(rampRPM_RL, filteredRPM_RL, integral_RL, lastError_RL);
    int pwmRR = computePID(rampRPM_RR, filteredRPM_RR, integral_RR, lastError_RR);

    ledcWrite(ENA_FL, pwmFL);
    ledcWrite(ENB_RL, pwmRL);
    ledcWrite(ENA_FR, pwmFR);
    ledcWrite(ENB_RR, pwmRR);
  }

  // ---------------- TELEMETRY ----------------
  if (currentTime - lastTelemetryTime > 200) {
    lastTelemetryTime = currentTime;

    if (client.connected()) {
      String json = "{\"speed\":" + String(currentSpeedKmh, 2)
                  + ",\"pca9685\":" + String(pcaReady ? "true" : "false")
                  + "}";
      client.publish(TOPIC_TELEMETRY, json.c_str());
    }
  }

  // ---------------- STATUS ----------------
  if (currentTime - lastStatusTime > 3000) {
    lastStatusTime = currentTime;
    if (client.connected()) {
      publishStatus();
    }
  }

  delay(1);
}
