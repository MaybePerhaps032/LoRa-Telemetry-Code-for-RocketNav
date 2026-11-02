/*
  TRANSMITTER  (Seeed XIAO ESP32S3 + Wio-SX1262)
 
   Sensors:
     MPU6050  (IMU)         I2C  addr 0x68   SDA->D4/GPIO5   SCL->D5/GPIO6
     BMP388   (barometer)   I2C  addr 0x77 or 0x76  (same bus, no conflict)
     NEO-6M   (GPS)         UART1 9600 baud  GPS TXD->D7/GPIO44  RXD->D6/GPIO43
     SX1262   (LoRa)        SPI via B2B connector, 915 MHz
 
   Libraries required:
     MPU6050 by Electronic Cats   (pulls in I2Cdev)
     Adafruit BMP3XX + Adafruit Unified Sensor
     TinyGPSPlus
     RadioLib
 */

#include <Wire.h>
#include <SPI.h>
#include <RadioLib.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <TinyGPSPlus.h>

 
// PIN MAP
#define LORA_CS     41
#define LORA_DIO1   39
#define LORA_RST    42
#define LORA_BUSY   40
#define LORA_SCK     7
#define LORA_MISO    8
#define LORA_MOSI    9

#define GPS_RX_PIN  44   // XIAO D7  <- GPS TXD
#define GPS_TX_PIN  43   // XIAO D6  -> GPS RXD
#define GPS_BAUD  9600


// RADIO CONFIG  

#define LORA_FREQ       915.0
#define LORA_BW         125.0
#define LORA_SF         7
#define LORA_CR         5
#define LORA_SYNCWORD   0x12
#define LORA_POWER      10


// TIMING
#define IMU_PERIOD_MS       20   //  50 Hz
#define BARO_PERIOD_MS      50   //  20 Hz
#define TX_PERIOD_IDLE_MS  500   //   2 Hz on the pad
#define TX_PERIOD_FLY_MS   200   //   5 Hz once launch is detected

// Launch detection: sustained acceleration above this many g
#define LAUNCH_G_THRESHOLD  3.0
#define LAUNCH_HOLD_MS      100


// SENSOR SCALING

const float ACCEL_SCALE = 2048.0;   // LSB per g  at +/-16 g
const float GYRO_SCALE  = 16.4;     // LSB per deg/s at +/-2000 deg/s


//  TELEMETRY PACKET  --  KEEP THIS BLOCK IDENTICAL IN BOTH SKETCHES

#define PKT_MAGIC    0xA5
#define PKT_VERSION  0x01

#define FLAG_IMU_OK   0x01
#define FLAG_BARO_OK  0x02
#define FLAG_GPS_FIX  0x04
#define FLAG_LAUNCHED 0x08

struct __attribute__((packed)) TelemetryPacket {
  uint8_t  magic;        // 0xA5 sanity marker
  uint8_t  version;      // packet format version
  uint8_t  flags;        // status bits, see FLAG_* above
  uint8_t  sats;         // GPS satellites in view

  uint16_t seq;          // packet counter (wraps at 65535)
  int16_t  temp_c10;     // temperature, 0.1 degC units

  uint32_t t_ms;         // millis() at time of sampling

  float    pressure_pa;  // raw barometric pressure
  float    alt_agl_m;    // altitude above the launch pad
  int32_t  lat_e7;       // latitude  * 1e7
  int32_t  lon_e7;       // longitude * 1e7

  int16_t  ax, ay, az;   // raw accel counts (divide by 2048 -> g)
  int16_t  gx, gy, gz;   // raw gyro counts  (divide by 16.4 -> deg/s)

  int16_t  gps_alt_m;    // GPS altitude, whole metres MSL
  uint16_t crc;          // CRC-16/CCITT over the preceding 42 bytes
};

static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}


// OBJECTS

SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);
MPU6050 imu;
Adafruit_BMP3XX bmp;
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);


// STATE
volatile bool txDone = true;      // set by the DIO1 interrupt
bool txPending = false;           // true while a packet is in the air
bool imuOK  = false;
bool baroOK = false;

float groundPressurePa = 101325.0;   // captured at boot, used for AGL
uint16_t seqCounter = 0;
bool launched = false;
uint32_t aboveThresholdSince = 0;

// latest sensor values
int16_t ax, ay, az, gx, gy, gz;
float pressurePa = 0, altAglM = 0, tempC = 0;

uint32_t lastImu = 0, lastBaro = 0, lastTx = 0;

void IRAM_ATTR onTxDone() { txDone = true; }


void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println(F("\n=== Rocket Telemetry TX booting ==="));
  Serial.printf("Packet size: %u bytes\n", (unsigned)sizeof(TelemetryPacket));

  // I2C bus (MPU6050 + BMP388 )
  Wire.begin();                       // D4/D5 are the XIAO hardware defaults
  Wire.setClock(400000);              // 400 kHz, both sensors handle this

  //  MPU6050 
  Serial.print(F("MPU6050... "));
  imu.initialize();
  imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);    // +/-16 g
  imu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);    // +/-2000 deg/s
  imuOK = imu.testConnection();
  Serial.println(imuOK ? F("OK") : F("NOT FOUND"));

  // BMP388 
  Serial.print(F("BMP388... "));
  baroOK = bmp.begin_I2C(0x77, &Wire) || bmp.begin_I2C(0x76, &Wire);
  if (baroOK) {
    // Bosch "drone" preset: fast, low-noise altitude
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    Serial.println(F("OK"));

    // 
    Serial.print(F("Capturing ground reference"));
    double sum = 0; int n = 0;
    for (int i = 0; i < 40; i++) {
      if (bmp.performReading()) { sum += bmp.pressure; n++; }
      if (i % 10 == 0) Serial.print('.');
      delay(25);
    }
    if (n > 0) groundPressurePa = sum / n;
    Serial.printf(" %.1f Pa\n", groundPressurePa);
  } else {
    Serial.println(F("NOT FOUND"));
  }

  // GPS 
  gpsSerial.setRxBufferSize(1024);    // must be called before begin()
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println(F("GPS UART started (fix may take several minutes)"));

  // LoRa 
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  Serial.print(F("LoRa... "));
  int state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR,
                          LORA_SYNCWORD, LORA_POWER);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("FAILED, code %d\n", state);
    while (true) delay(1000);
  }
  radio.setPacketSentAction(onTxDone);   // RadioLib < 6.x: setDio1Action()
  Serial.println(F("OK"));

  Serial.println(F("=== Running ===\n"));
}

void readImu() {
  if (!imuOK) return;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Launch detection on total acceleration magnitude
  float axg = ax / ACCEL_SCALE, ayg = ay / ACCEL_SCALE, azg = az / ACCEL_SCALE;
  float mag = sqrtf(axg * axg + ayg * ayg + azg * azg);

  if (!launched) {
    if (mag > LAUNCH_G_THRESHOLD) {
      if (aboveThresholdSince == 0) aboveThresholdSince = millis();
      else if (millis() - aboveThresholdSince > LAUNCH_HOLD_MS) {
        launched = true;
        Serial.println(F(">>> LAUNCH DETECTED <<<"));
      }
    } else {
      aboveThresholdSince = 0;
    }
  }
}

void readBaro() {
  if (!baroOK) return;
  if (!bmp.performReading()) return;
  pressurePa = bmp.pressure;
  tempC      = bmp.temperature;
  // Barometric formula, referenced to the pad rather than sea level
  altAglM = 44330.0 * (1.0 - powf(pressurePa / groundPressurePa, 0.1902949));
}

void feedGps() {
  while (gpsSerial.available() > 0) gps.encode(gpsSerial.read());
}


void sendPacket() {
  TelemetryPacket pkt = {};   // zero-fill

  pkt.magic   = PKT_MAGIC;
  pkt.version = PKT_VERSION;
  pkt.seq     = seqCounter++;
  pkt.t_ms    = millis();

  pkt.flags = 0;
  if (imuOK)    pkt.flags |= FLAG_IMU_OK;
  if (baroOK)   pkt.flags |= FLAG_BARO_OK;
  if (launched) pkt.flags |= FLAG_LAUNCHED;

  pkt.ax = ax; pkt.ay = ay; pkt.az = az;
  pkt.gx = gx; pkt.gy = gy; pkt.gz = gz;

  pkt.pressure_pa = pressurePa;
  pkt.alt_agl_m   = altAglM;
  pkt.temp_c10    = (int16_t)(tempC * 10.0);

  if (gps.location.isValid()) {
    pkt.flags  |= FLAG_GPS_FIX;
    pkt.lat_e7  = (int32_t)(gps.location.lat() * 1e7);
    pkt.lon_e7  = (int32_t)(gps.location.lng() * 1e7);
  }
  if (gps.altitude.isValid())   pkt.gps_alt_m = (int16_t)gps.altitude.meters();
  if (gps.satellites.isValid()) pkt.sats      = (uint8_t)gps.satellites.value();

  pkt.crc = crc16_ccitt((uint8_t *)&pkt, sizeof(pkt) - sizeof(pkt.crc));

  txDone = false;
  txPending = true;
  int state = radio.startTransmit((uint8_t *)&pkt, sizeof(pkt));
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("TX start error %d\n", state);
    txDone = true;
    txPending = false;
  }

  // Local echo so you can watch it on the Serial Monitor
  Serial.printf("#%u t=%lu | a %.2f %.2f %.2f g | alt %.1f m | %.1f Pa | "
                "sats %u | %s\n",
                pkt.seq, (unsigned long)pkt.t_ms,
                ax / ACCEL_SCALE, ay / ACCEL_SCALE, az / ACCEL_SCALE,
                altAglM, pressurePa, pkt.sats,
                (pkt.flags & FLAG_GPS_FIX) ? "FIX" : "no fix");
}


void loop() {
  uint32_t now = millis();

  feedGps();   // called every pass so the UART buffer never backs up

  if (now - lastImu >= IMU_PERIOD_MS) {
    lastImu = now;
    readImu();
  }

  if (now - lastBaro >= BARO_PERIOD_MS) {
    lastBaro = now;
    readBaro();
  }

  // Release the radio once, right after the previous packet finished
  if (txPending && txDone) {
    radio.finishTransmit();
    txPending = false;
  }

  uint32_t txPeriod = launched ? TX_PERIOD_FLY_MS : TX_PERIOD_IDLE_MS;
  if (!txPending && (now - lastTx >= txPeriod)) {
    lastTx = now;
    sendPacket();
  }
}
