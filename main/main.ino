#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// ─── WiFi ───────────────────────────────────────────────
const char* ssid     = "pro";
const char* password = "slamxuwb";

// ─── MPU-9250 ───────────────────────────────────────────
#define MPU_ADDR 0x68

// Paste your offsets from calibration
const float AX_OFF = 722.37f, AY_OFF = 478.04f, AZ_OFF = 520.42f;
const float GX_OFF = -152.63f, GY_OFF = -721.61f, GZ_OFF = -233.23f;

// ─── TB6612FNG pins ─────────────────────────────────────
// Motor A (Left)
#define AIN1  5
#define AIN2  16
#define PWMA  4

// Motor B (Right)
#define BIN1  18
#define BIN2  19
#define PWMB  23

//#define STBY  12   // Standby pin — pull HIGH to enable driver

// LEDC channels
#define CH_A  0
#define CH_B  1

// ─── Kalman filter ──────────────────────────────────────
struct Kalman {
  float Q_angle   = 0.001f;
  float Q_bias    = 0.003f;
  float R_measure = 0.03f;
  float angle = 0.0f, bias = 0.0f;
  float P[2][2]   = {{0,0},{0,0}};

  float update(float newAngle, float newRate, float dt) {
    float rate = newRate - bias;
    angle += dt * rate;
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;
    float S = P[0][0] + R_measure;
    float K[2] = { P[0][0] / S, P[1][0] / S };
    float y = newAngle - angle;
    angle += K[0] * y;
    bias  += K[1] * y;
    P[0][0] -= K[0] * P[0][0];
    P[0][1] -= K[0] * P[0][1];
    P[1][0] -= K[1] * P[0][0];
    P[1][1] -= K[1] * P[0][1];
    return angle;
  }
} kalman;

// ─── PID ────────────────────────────────────────────────
float Kp = 25.0f, Ki = 0.8f, Kd = 0.6f;
float setpoint   = 0.0f;   // target angle (degrees) — trim if robot leans
float integral   = 0.0f;
float lastError  = 0.0f;
const float INTEGRAL_LIMIT = 200.0f;

bool running = false;

// ─── Web ─────────────────────────────────────────────────
WebServer        httpServer(80);
WebSocketsServer wsServer(81);

// ─── Timing ─────────────────────────────────────────────
unsigned long lastTime = 0;
const int     LOOP_HZ  = 200;
const float   DT       = 1.0f / LOOP_HZ;

// ────────────────────────────────────────────────────────
void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
  delay(100);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(); // ±250 dps
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(); // ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission(); // DLPF 44Hz
}

void readMPU(float &ax, float &ay, float &az,
             float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);

  int16_t rax = (Wire.read()<<8)|Wire.read();
  int16_t ray = (Wire.read()<<8)|Wire.read();
  int16_t raz = (Wire.read()<<8)|Wire.read();
  Wire.read(); Wire.read(); // temp
  int16_t rgx = (Wire.read()<<8)|Wire.read();
  int16_t rgy = (Wire.read()<<8)|Wire.read();
  int16_t rgz = (Wire.read()<<8)|Wire.read();

  ax = (rax - AX_OFF) / 16384.0f;
  ay = (ray - AY_OFF) / 16384.0f;
  az = (raz - AZ_OFF) / 16384.0f;
  gx = (rgx - GX_OFF) / 131.0f;
  gy = (rgy - GY_OFF) / 131.0f;
  gz = (rgz - GZ_OFF) / 131.0f;
}

// ─── Motor control ──────────────────────────────────────
void setMotors(int speed) {
  speed = constrain(speed, -255, 255);
  if (!running) speed = 0;

  if (speed >= 0) {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); //swap acc to the bot motion with regrading to the tilt
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(AIN1, HIGH);  digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);  digitalWrite(BIN2, LOW);
    speed = -speed;
  }
  ledcWrite(CH_A, speed);
  ledcWrite(CH_B, speed);
}

void stopMotors() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
  ledcWrite(CH_A, 0);
  ledcWrite(CH_B, 0);
}

// ─── WebSocket ──────────────────────────────────────────
void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type != WStype_TEXT) return;
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload)) return;

  if (doc.containsKey("kp"))      Kp      = doc["kp"];
  if (doc.containsKey("ki"))      Ki      = doc["ki"];
  if (doc.containsKey("kd"))      Kd      = doc["kd"];
  if (doc.containsKey("sp"))      setpoint = doc["sp"];
  if (doc.containsKey("running")) {
    running = doc["running"];
    if (!running) { stopMotors(); integral = 0; lastError = 0; }
  }
}

// ─── HTML page (served from ESP32) ──────────────────────
// We'll point the browser to the separate hosted page, OR
// you can hardcode the page string here.
// For now the ESP32 just returns its IP so you can open
// the hosted page and connect to ws://<ESP32_IP>:81

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><body>
<h3>Self-Balancing Bot</h3>
<p>Open the tuning page and enter WebSocket: <b id="ip"></b>:81</p>
<script>document.getElementById('ip').textContent = location.hostname;</script>
</body></html>
)rawliteral";

// ────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin();
  initMPU();

  // Motor driver pins
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  //pinMode(STBY, OUTPUT);
 // digitalWrite(STBY, HIGH);

  ledcSetup(CH_A, 20000, 8);  // 20kHz PWM, 8-bit
  ledcSetup(CH_B, 20000, 8);
  ledcAttachPin(PWMA, CH_A);
  ledcAttachPin(PWMB, CH_B);
  stopMotors();

  // WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting");
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
     delay(500);
     Serial.print(".");
     if (++tries > 40) {
        Serial.println("\nFailed. Check SSID/password or switch to hotspot.");
        break;
      }
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nIP: " + WiFi.localIP().toString());
  }
  httpServer.on("/", []() {
     httpServer.send(200, "text/html", INDEX_HTML);
   });
  httpServer.begin();
  wsServer.begin();
  wsServer.onEvent(onWsEvent);

  // Init Kalman with first accel reading
  float ax,ay,az,gx,gy,gz;
  readMPU(ax,ay,az,gx,gy,gz);
  //float initAngle = atan2(ay, az) * 180.0f / PI;
  float initAngle = atan2(ax, az) * 180.0f / PI;
  kalman.angle = initAngle;
  lastTime = micros();
}

void loop() {
  httpServer.handleClient();
  wsServer.loop();

  unsigned long now = micros();
  if ((now - lastTime) < (1000000UL / LOOP_HZ)) return;
  float dt = (now - lastTime) / 1e6f;
  lastTime = now;

  // ── Read IMU ──
  float ax,ay,az,gx,gy,gz;
  readMPU(ax,ay,az,gx,gy,gz);

  //float accelAngle = atan2(ay, az) * 180.0f / PI;
  //float angle = kalman.update(accelAngle, gx, dt);
  float accelAngle = atan2(ax, az) * 180.0f / PI;
  float angle = kalman.update(accelAngle, gy, dt);

  // ── Tilt cutoff — kill motors if too far ──
  if (abs(angle) > 60.0f) {
    stopMotors();
    running = false;
    return;
  }

  if (!running) return;

  // ── PID ──
  float error = angle - setpoint;
  integral   += error * dt;
  integral    = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float derivative = (error - lastError) / dt;
  lastError   = error;

  float output = Kp*error + Ki*integral + Kd*derivative;
  setMotors((int)output);

  // ── Telemetry over WebSocket (10Hz) ──
  static int teleTick = 0;
  if (++teleTick >= LOOP_HZ/10) {
    teleTick = 0;
    char buf[96];
    snprintf(buf, sizeof(buf),
      "{\"angle\":%.2f,\"output\":%.1f,\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f}",
      angle, output, Kp, Ki, Kd);
    wsServer.broadcastTXT(buf);
  }
  // float ax,ay,az,gx,gy,gz;
  // readMPU(ax,ay,az,gx,gy,gz);
  // Serial.printf("ax:%.3f ay:%.3f az:%.3f | gx:%.3f gy:%.3f gz:%.3f\n",
  //                ax, ay, az, gx, gy, gz);
  // delay(100);
}