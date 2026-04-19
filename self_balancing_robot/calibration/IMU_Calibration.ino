#include <Wire.h>

// MPU-9250 I2C address
#define MPU_ADDR 0x68

// Number of samples for averaging
#define SAMPLES 2000

float accel_offset[3] = {0, 0, 0};  // ax, ay, az
float gyro_offset[3]  = {0, 0, 0};  // gx, gy, gz

void writeRegister(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1);
  return Wire.read();
}

void initMPU() {
  writeRegister(0x6B, 0x00);  // PWR_MGMT_1 — wake up
  delay(100);
  writeRegister(0x1B, 0x00);  // GYRO_CONFIG  — ±250 dps
  writeRegister(0x1C, 0x00);  // ACCEL_CONFIG — ±2g
  writeRegister(0x1A, 0x03);  // CONFIG       — DLPF 44Hz
}

void readRaw(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  /* temp — skip */ Wire.read(); Wire.read();
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

void calibrate() {
  Serial.println("Keep MPU flat and still...");
  delay(3000);  // time to settle
  Serial.println("Sampling...");

  long sum[6] = {0, 0, 0, 0, 0, 0};
  int16_t ax, ay, az, gx, gy, gz;

  for (int i = 0; i < SAMPLES; i++) {
    readRaw(ax, ay, az, gx, gy, gz);
    sum[0] += ax;
    sum[1] += ay;
    sum[2] += az;
    sum[3] += gx;
    sum[4] += gy;
    sum[5] += gz;
    delay(2);
  }

  // Accel offsets: X and Y should be 0, Z should be +16384 (1g at ±2g scale)
  accel_offset[0] = sum[0] / (float)SAMPLES;
  accel_offset[1] = sum[1] / (float)SAMPLES;
  accel_offset[2] = sum[2] / (float)SAMPLES - 16384.0f;

  // Gyro offsets: all three should be 0
  gyro_offset[0] = sum[3] / (float)SAMPLES;
  gyro_offset[1] = sum[4] / (float)SAMPLES;
  gyro_offset[2] = sum[5] / (float)SAMPLES;

  Serial.println("\n=== Calibration Offsets ===");
  Serial.print("Accel  X: "); Serial.print(accel_offset[0]);
  Serial.print("  Y: ");      Serial.print(accel_offset[1]);
  Serial.print("  Z: ");      Serial.println(accel_offset[2]);
  Serial.print("Gyro   X: "); Serial.print(gyro_offset[0]);
  Serial.print("  Y: ");      Serial.print(gyro_offset[1]);
  Serial.print("  Z: ");      Serial.println(gyro_offset[2]);
  Serial.println("\nCopy these into your main code.");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);
  initMPU();
  calibrate();
}

void loop() {
  // Nothing — offsets printed once to Serial
}