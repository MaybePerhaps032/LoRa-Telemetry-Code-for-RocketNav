/*

   ROCKET TELEMETRY - GROUND STATION RECEIVER  (Heltec WiFi LoRa 32 V3)

 
   Listens for 44-byte binary telemetry packets from the XIAO transmitter,
   validates them, and prints one CSV row per packet at 115200 baud.
 
   Capture the Serial Monitor output to a .csv file and will have a flight
   log ready for training
 
   Library required: RadioLib
 
   

 */

#include <RadioLib.h>

// Heltec V3 internal SX1262 pin mapping
#define LORA_CS    8
#define LORA_DIO1 14
#define LORA_RST  12
#define LORA_BUSY 13
#define VEXT_PIN  35


// RADIO CONFIG  --  MUST MATCH THE TRANSMITTER SKETCH EXACTLY

#define LORA_FREQ       915.0
#define LORA_BW         125.0
#define LORA_SF         7
#define LORA_CR         5
#define LORA_SYNCWORD   0x12
#define LORA_POWER      10


// SENSOR SCALING (must match the ranges set on the transmitter)

const float ACCEL_SCALE = 2048.0;   // LSB per g      at +/-16 g
const float GYRO_SCALE  = 16.4;     // LSB per deg/s  at +/-2000 deg/s


//  TELEMETRY PACKET 

#define PKT_MAGIC    0xA5
#define PKT_VERSION  0x01

#define FLAG_IMU_OK   0x01
#define FLAG_BARO_OK  0x02
#define FLAG_GPS_FIX  0x04
#define FLAG_LAUNCHED 0x08

struct __attribute__((packed)) TelemetryPacket {
  uint8_t  magic;
  uint8_t  version;
  uint8_t  flags;
  uint8_t  sats;

  uint16_t seq;
  int16_t  temp_c10;

  uint32_t t_ms;

  float    pressure_pa;
  float    alt_agl_m;
  int32_t  lat_e7;
  int32_t  lon_e7;

  int16_t  ax, ay, az;
  int16_t  gx, gy, gz;

  int16_t  gps_alt_m;
  uint16_t crc;
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

SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);

volatile bool packetWaiting = false;
uint32_t rxCount = 0, badCount = 0;
uint16_t lastSeq = 0;
bool haveLastSeq = false;
uint32_t lostCount = 0;

void IRAM_ATTR onPacketReceived() { packetWaiting = true; }


void setup() {
  Serial.begin(115200);
  delay(2000);

  // Enable the Heltec V3 Vext rail
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);

  Serial.println(F("\n=== Rocket Telemetry Ground Station ==="));
  Serial.printf("Expecting %u byte packets\n",
                (unsigned)sizeof(TelemetryPacket));

  Serial.print(F("LoRa... "));
  int state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR,
                          LORA_SYNCWORD, LORA_POWER);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("FAILED, code %d\n", state);
    while (true) delay(1000);
  }

  radio.setPacketReceivedAction(onPacketReceived);  // RadioLib < 6.x: setDio1Action()
  radio.startReceive();
  Serial.println(F("OK - listening\n"));

  // CSV header. Everything after this line is data.
  Serial.println(F("seq,t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,"
                   "pressure_pa,alt_agl_m,temp_c,lat,lon,gps_alt_m,"
                   "sats,gps_fix,launched,rssi_dbm,snr_db"));
}


void loop() {
  if (!packetWaiting) return;
  packetWaiting = false;

  TelemetryPacket pkt;
  size_t len = radio.getPacketLength();
  int state = radio.readData((uint8_t *)&pkt, sizeof(pkt));

  // Re-arm the receiver immediately so we miss as little as possible
  radio.startReceive();

  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("# RX error %d\n", state);
    badCount++;
    return;
  }

  if (len != sizeof(pkt)) {
    Serial.printf("# wrong length: got %u expected %u\n",
                  (unsigned)len, (unsigned)sizeof(pkt));
    badCount++;
    return;
  }

  if (pkt.magic != PKT_MAGIC || pkt.version != PKT_VERSION) {
    Serial.printf("# bad magic/version: 0x%02X v%u\n", pkt.magic, pkt.version);
    badCount++;
    return;
  }

  uint16_t expect = crc16_ccitt((uint8_t *)&pkt, sizeof(pkt) - sizeof(pkt.crc));
  if (expect != pkt.crc) {
    Serial.println(F("# CRC mismatch - packet discarded"));
    badCount++;
    return;
  }

  //valid packet
  rxCount++;
  if (haveLastSeq) {
    uint16_t gap = pkt.seq - lastSeq;   // unsigned math handles the wrap
    if (gap > 1) lostCount += (gap - 1);
  }
  lastSeq = pkt.seq;
  haveLastSeq = true;

  Serial.printf("%u,%lu,"
                "%.4f,%.4f,%.4f,"
                "%.2f,%.2f,%.2f,"
                "%.2f,%.2f,%.1f,"
                "%.7f,%.7f,%d,"
                "%u,%u,%u,"
                "%.1f,%.2f\n",
                pkt.seq, (unsigned long)pkt.t_ms,
                pkt.ax / ACCEL_SCALE, pkt.ay / ACCEL_SCALE, pkt.az / ACCEL_SCALE,
                pkt.gx / GYRO_SCALE,  pkt.gy / GYRO_SCALE,  pkt.gz / GYRO_SCALE,
                pkt.pressure_pa, pkt.alt_agl_m, pkt.temp_c10 / 10.0,
                pkt.lat_e7 / 1e7, pkt.lon_e7 / 1e7, pkt.gps_alt_m,
                pkt.sats,
                (pkt.flags & FLAG_GPS_FIX) ? 1 : 0,
                (pkt.flags & FLAG_LAUNCHED) ? 1 : 0,
                radio.getRSSI(), radio.getSNR());

  // Link quality summary every 50 good packets, prefixed with # so it is
  // easy to strip out of the CSV later
  if (rxCount % 50 == 0) {
    Serial.printf("# link: %lu good, %lu bad, %lu lost, RSSI %.1f dBm\n",
                  (unsigned long)rxCount, (unsigned long)badCount,
                  (unsigned long)lostCount, radio.getRSSI());
  }
}
