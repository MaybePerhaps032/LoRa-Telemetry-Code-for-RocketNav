#include <TinyGPSPlus.h> //

// Define the Serial pins 
#define GPS_RX_PIN 44  // Connected to GPS TXD (XIAO D7)
#define GPS_TX_PIN 43  // Connected to GPS RXD (XIAO D6)
#define GPS_BAUD 9600  // GY-GPS6MV2 default baud rate

// Create a TinyGPS++ object to parse data
TinyGPSPlus gps;

// Use HardwareSerial 1 for the GPS connection
HardwareSerial gpsSerial(1);

void setup() {
  // Start the USB Serial Monitor for debugging
  Serial.begin(115200);
  delay(2000); 
  Serial.println("Initializing GPS connection...");

  // Initialize the hardware serial port for the GPS module
  // config: SERIAL_8N1, RX pin, TX pin
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

void loop() {
  // Feed the incoming GPS serial data into the TinyGPS++ parser
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // Every 2 seconds, print the parsed GPS info if updated
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 2000) {
    lastPrint = millis();

    Serial.print("Satellites: ");
    if (gps.satellites.isValid()) {
      Serial.println(gps.satellites.value());
    } else {
      Serial.println("Searching...");
    }

    Serial.print("Location: ");
    if (gps.location.isValid()) {
      // Print Latitude and Longitude with high precision
      Serial.print(gps.location.lat(), 6);
      Serial.print(", ");
      Serial.println(gps.location.lng(), 6);
    } else {
      Serial.println("No Fix / Waiting for Satellite...");
    }
    Serial.println("-------------------------------------");
  }
}