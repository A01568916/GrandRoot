/*
 * esp32_sensores_DEBUG_FULL.ino
 *
 * Versión con debug intensivo para encontrar por qué no llegan datos del GPS.
 * Imprime cada paso del loop con timestamps.
 */

#include <Wire.h>
#include <TinyGPS++.h>

#define GPS_RX_PIN    16
#define GPS_TX_PIN    17
#define GPS_BAUDRATE  9600

#define LINK_TX_PIN   4
#define LINK_RX_PIN   5
#define LINK_BAUD     115200

#define IMU_ADDR      0x68
#define ACCEL_SCALE   16384.0f
#define GYRO_SCALE    131.0f

#define INTERVALO_MS  500   // más lento para ver bien

const double LAT_REF = 28.7385775000;
const double LON_REF = -106.1217361670;

#define REG_PWR_MGMT_1  0x6B
#define REG_WHO_AM_I    0x75
#define REG_ACCEL_XOUT  0x3B

TinyGPSPlus gps;

unsigned long t_ultimo_envio = 0;
unsigned long total_bytes_gps = 0;
unsigned long total_chars_processed = 0;
unsigned long ciclos_loop = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("  DEBUG FULL - ESP32 Sensores");
  Serial.println("========================================");

  // I2C + IMU
  Wire.begin(21, 22);
  delay(200);
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);
  Serial.println("[IMU] Despertado");

  // Serial2 para GPS
  Serial2.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.printf("[GPS] Serial2 RX=%d TX=%d @ %d baud\n",
                GPS_RX_PIN, GPS_TX_PIN, GPS_BAUDRATE);

  // Serial1 para link al PID
  Serial1.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  Serial.printf("[LINK] Serial1 TX=%d → ESP32 PID @ %d baud\n",
                LINK_TX_PIN, LINK_BAUD);

  Serial.println("\nEmpezando loop. Cada 500ms imprimo status.\n");
}

void loop() {
  ciclos_loop++;

  // ── Procesar GPS ───────────────────────────────────────────
  int bytes_este_ciclo = 0;
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    total_bytes_gps++;
    bytes_este_ciclo++;
    gps.encode(c);
  }

  unsigned long ahora = millis();

  // ── Status cada 500 ms ──────────────────────────────────────
  if (ahora - t_ultimo_envio >= INTERVALO_MS) {
    t_ultimo_envio = ahora;

    Serial.printf(
      "[%lus] loop=%lu  | GPS: bytes=%lu  chars_proc=%lu  sats=%u  valid=%s  lat=%.6f lon=%.6f\n",
      ahora / 1000,
      ciclos_loop,
      total_bytes_gps,
      (unsigned long)gps.charsProcessed(),
      gps.satellites.value(),
      gps.location.isValid() ? "SI" : "NO",
      gps.location.lat(),
      gps.location.lng()
    );

    // También mandar un mensaje corto al PID para ver si Serial1 funciona
    Serial1.printf("{\"test\":%lu}\n", ahora);
  }
}
