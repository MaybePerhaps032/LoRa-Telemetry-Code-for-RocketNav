#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

// Define I2C pins for XIAO ESP32S3
#define SDA_PIN D4
#define SCL_PIN D5

Adafruit_BMP3XX bmp;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Initialize custom I2C pins for XIAO ESP32S3
  Wire.begin(SDA_PIN, SCL_PIN);

  // Default I2C address for CJMCU-388 is usually 0x77 or 0x76
  if (!bmp.begin_I2C(0x77, &Wire)) {
    Serial.println("Could not find a valid BMP388 sensor, checking 0x76...");
    if (!bmp.begin_I2C(0x76, &Wire)) {
      Serial.println("Error: BMP388 not detected. Check wiring!");
      while (1);
    }
  }

  // Set up oversampling and filter settings for performance
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void loop() {
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx Altitude = ");
  Serial.print(bmp.readAltitude(1013.25)); // Adjusted for local sea-level pressure
  Serial.println(" m");

  Serial.println("---");
  delay(2000);
}