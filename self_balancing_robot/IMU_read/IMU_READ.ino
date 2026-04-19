#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;

// --- FINAL PERFECTED OFFSETS ---
const float AX_OFF = 722.37;
const float AY_OFF = 478.04;
const float AZ_OFF = 520.42;
const float GX_OFF = -152.63;
const float GY_OFF = -721.61;
const float GZ_OFF = -233.23;

//Accel  X: 722.37  Y: 478.04  Z: 520.42
//Gyro   X: -152.63  Y: -721.61  Z: -233.23

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14, (bool)true);

  int16_t rax = Wire.read() << 8 | Wire.read();
  int16_t ray = Wire.read() << 8 | Wire.read();
  int16_t raz = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); 
  int16_t rgx = Wire.read() << 8 | Wire.read();
  int16_t rgy = Wire.read() << 8 | Wire.read();
  int16_t rgz = Wire.read() << 8 | Wire.read();

  // Convert to real units
  float ax = (rax - AX_OFF) / 16384.0; 
  float ay = (ray - AY_OFF) / 16384.0;
  float az = (raz - AZ_OFF) / 16384.0;

  float gx = (rgx - GX_OFF) / 131.0;
  float gy = (rgy - GY_OFF) / 131.0;
  float gz = (rgz - GZ_OFF) / 131.0;

  // --- APPLY DEADZONE (Filtering noise) ---
  if (abs(gx) < 0.05) gx = 0;
  if (abs(gy) < 0.05) gy = 0;
  if (abs(gz) < 0.05) gz = 0;

  // Displaying X, Y, Z for both sets
  Serial.print("ACCEL > X: "); Serial.print(ax, 3);
  Serial.print(" | Y: ");      Serial.print(ay, 3);
  Serial.print(" | Z: ");      Serial.print(az, 3);
  
  Serial.print("  ||  GYRO > X: ");  Serial.print(gx, 2);
  Serial.print(" | Y: ");           Serial.print(gy, 2);
  Serial.print(" | Z: ");           Serial.println(gz, 2);

  delay(100);
}