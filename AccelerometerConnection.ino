/*
 MPU6050 test sketch for XIAO ESP32S3
Library: "MPU6050 by Electronic Cats" (installs I2Cdev dependency)
Wiring:  VCC->3V3  GND->GND  SCL->D5(GPIO6)  SDA->D4(GPIO5)

 Open Serial Monitor at 115200 baud after uploading.
 */

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 accelgyro;   // default I2C address 0x68

int16_t ax, ay, az;  // raw accelerometer
int16_t gx, gy, gz;  // raw gyro

// Scale factor for +/-16g range: 2048 LSB per g. (Default 2g range = 16384.)
const float ACCEL_SCALE = 2048.0;
const float G_TO_MS2     = 9.80665;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin();                 // D4 = SDA, D5 = SCL (XIAO defaults)

  Serial.println("Initializing MPU6050...");
  accelgyro.initialize();
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);  // wide range for a rocket

  Serial.println(accelgyro.testConnection()
                 ? "MPU6050 connected!"
                 : "MPU6050 NOT found - check wiring and 3V3 power.");
}

void loop() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert raw accel counts to g, then to m/s^2
  float axg = ax / ACCEL_SCALE;
  float ayg = ay / ACCEL_SCALE;
  float azg = az / ACCEL_SCALE;

  Serial.printf("Accel (g)  X:%6.2f  Y:%6.2f  Z:%6.2f   |   ", axg, ayg, azg);
  Serial.printf("Accel (m/s^2)  X:%6.2f  Y:%6.2f  Z:%6.2f   |   ",
                axg * G_TO_MS2, ayg * G_TO_MS2, azg * G_TO_MS2);
  Serial.printf("Gyro (raw)  X:%6d  Y:%6d  Z:%6d\n", gx, gy, gz);

  delay(200);
}