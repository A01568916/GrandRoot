/*
 ╔══════════════════════════════════════════════════════════════════════════════╗
 ║  GrandRoot — ESP32 SENSORES                                                  ║
 ║  GPS NEO-6M/8M (TinyGPS++)  +  IMU MPU6050 (I2C)                             ║
 ║                                                                              ║
 ║  ─── ARQUITECTURA ────────────────────────────────────────────────────────── ║
 ║                                                                              ║
 ║   ESP32 SENSORES (este)                                                      ║
 ║          │ Serial1 TX (GPIO 4)                                               ║
 ║          ▼  JSON cada INTERVALO_MS                                           ║
 ║   ESP32 PID (motores + WiFi)                                                 ║
 ║          ▼ WebSocket                                                         ║
 ║   Dashboard en navegador                                                     ║
 ║                                                                              ║
 ║  NO se envía nada por USB-Serial. El USB sólo se usa para LOGS de depuración ║
 ║  (puedes desconectar el cable USB en operación y todo sigue funcionando).    ║
 ║                                                                              ║
 ║  ─── PINOUT ──────────────────────────────────────────────────────────────── ║
 ║                                                                              ║
 ║    GPS TXD    →  ESP32 RXD2 (GPIO 16)                                        ║
 ║    GPS RXD    →  ESP32 TXD2 (GPIO 17)   (no se usa pero hay que declararlo)  ║
 ║    GPS VIN    →  5V o 3.3V según módulo                                      ║
 ║    GPS GND    →  GND                                                         ║
 ║                                                                              ║
 ║    MPU6050 SCL →  ESP32 GPIO 22                                              ║
 ║    MPU6050 SDA →  ESP32 GPIO 21                                              ║
 ║    MPU6050 VCC →  3V3                                                        ║
 ║    MPU6050 GND →  GND                                                        ║
 ║    MPU6050 AD0 →  GND  (dirección I2C = 0x68)                                ║
 ║                                                                              ║
 ║    >>> LINK INTER-ESP32 <<<                                                  ║
 ║    ESP32 SENSORES GPIO 4 (TX1)  ──►  ESP32 PID GPIO 16 (RX2)                 ║
 ║    GND COMÚN entre ambos ESP32 (¡obligatorio!)                               ║
 ║                                                                              ║
 ║  Librería requerida:                                                         ║
 ║    TinyGPS++  (Library Manager del Arduino IDE)                              ║
 ╚══════════════════════════════════════════════════════════════════════════════╝
*/

#include <Wire.h>
#include <TinyGPS++.h>

// ═══════════════════════════════════════════════════════════════════════════
// CONFIGURACIÓN
// ═══════════════════════════════════════════════════════════════════════════

#define GPS_RX_PIN    16          // ESP32 RXD2 ← GPS TXD
#define GPS_TX_PIN    17          // ESP32 TXD2 → GPS RXD (no se usa)
#define GPS_BAUDRATE  9600        // default NEO-6M / NEO-8M

// ── Link al ESP32 PID por UART1 ─────────────────────────────────────────────
// Serial1 con pines remapeados: GPIO 4 = TX1, GPIO 5 = RX1 (no usado)
#define LINK_TX_PIN   4
#define LINK_RX_PIN   5
#define LINK_BAUD     115200

#define IMU_ADDR      0x68
#define ACCEL_SCALE   16384.0f    // LSB/g para ±2g
#define GYRO_SCALE    131.0f      // LSB/(°/s) para ±250°/s

// Cada cuántos ms enviamos un paquete al ESP32 PID
#define INTERVALO_MS  200

// Punto de referencia GPS para X/Y locales en metros
const double LAT_REF = 28.7385775000;
const double LON_REF = -106.1217361670;

// ═══════════════════════════════════════════════════════════════════════════
// REGISTROS MPU6050
// ═══════════════════════════════════════════════════════════════════════════

#define REG_SMPLRT_DIV  0x19
#define REG_CONFIG      0x1A
#define REG_GYRO_CFG    0x1B
#define REG_ACCEL_CFG   0x1C
#define REG_ACCEL_XOUT  0x3B
#define REG_GYRO_XOUT   0x43
#define REG_PWR_MGMT_1  0x6B
#define REG_WHO_AM_I    0x75

// ═══════════════════════════════════════════════════════════════════════════
// VARIABLES GLOBALES
// ═══════════════════════════════════════════════════════════════════════════

TinyGPSPlus gps;

struct DatosGPS {
  double lat   = 0.0;
  double lon   = 0.0;
  double x     = 0.0;
  double y     = 0.0;
  uint8_t sats = 0;
  bool ok      = false;
};

struct DatosIMU {
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;
  bool ok = false;
};

DatosGPS gpsData;
DatosIMU imuData;

unsigned long t_ultimo_envio = 0;

// ═══════════════════════════════════════════════════════════════════════════
// FUNCIONES I2C (MPU6050)
// ═══════════════════════════════════════════════════════════════════════════

void imu_escribir(uint8_t reg, uint8_t valor) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(valor);
  Wire.endTransmission();
}

uint8_t imu_leer(uint8_t reg) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

void imu_leerBytes(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_ADDR, n);
  for (uint8_t i = 0; i < n; i++)
    buf[i] = Wire.available() ? Wire.read() : 0;
}

inline int16_t combinar(uint8_t hi, uint8_t lo) {
  return (int16_t)((uint16_t)hi << 8 | lo);
}

// ═══════════════════════════════════════════════════════════════════════════
// CONVERSIÓN GPS → coordenadas planas (metros)
// ═══════════════════════════════════════════════════════════════════════════

double gpsToX(double lat, double lon) {
  return (lon - LON_REF) * cos(LAT_REF * PI / 180.0) * 111320.0;
}

double gpsToY(double lat, double lon) {
  return (lat - LAT_REF) * 111320.0;
}

// ═══════════════════════════════════════════════════════════════════════════
// INIT IMU
// ═══════════════════════════════════════════════════════════════════════════

bool initIMU() {
  uint8_t id = imu_leer(REG_WHO_AM_I);
  Serial.printf("[IMU] WHO_AM_I = 0x%02X (esperado 0x68, 0x71 o 0x73)\n", id);

  if (id != 0x68 && id != 0x71 && id != 0x73) {
    Serial.println("[IMU][ERROR] Sensor no detectado.");
    Serial.println("[IMU][ERROR] Revisa SDA/SCL, 3.3V y pin AD0.");
    return false;
  }

  const char* modelo = (id == 0x68) ? "MPU6050" : "MPU9250";

  imu_escribir(REG_PWR_MGMT_1, 0x00);
  delay(100);
  imu_escribir(REG_SMPLRT_DIV, 0x07);
  imu_escribir(REG_CONFIG,     0x06);
  imu_escribir(REG_GYRO_CFG,   0x00);
  imu_escribir(REG_ACCEL_CFG,  0x00);

  Serial.printf("[IMU] OK — %s configurado.\n", modelo);
  return true;
}

void leerIMU() {
  uint8_t raw[6];

  imu_leerBytes(REG_ACCEL_XOUT, raw, 6);
  imuData.ax = combinar(raw[0], raw[1]) / ACCEL_SCALE;
  imuData.ay = combinar(raw[2], raw[3]) / ACCEL_SCALE;
  imuData.az = combinar(raw[4], raw[5]) / ACCEL_SCALE;

  imu_leerBytes(REG_GYRO_XOUT, raw, 6);
  imuData.gx = combinar(raw[0], raw[1]) / GYRO_SCALE;
  imuData.gy = combinar(raw[2], raw[3]) / GYRO_SCALE;
  imuData.gz = combinar(raw[4], raw[5]) / GYRO_SCALE;
}

// ═══════════════════════════════════════════════════════════════════════════
// PUBLICAR JSON POR UART1 (al ESP32 PID)
// ═══════════════════════════════════════════════════════════════════════════
//
// Formato — una línea terminada en '\n' para que el receptor pueda
// hacer readStringUntil('\n'). Tipos compactos: floats con pocas decimales.
//
// Campos:
//   lat, lon : grados decimales (10 decimales para precisión)
//   x, y     : metros desde LAT_REF / LON_REF
//   sats     : satélites visibles
//   ax,ay,az : aceleración en g
//   gx,gy,gz : velocidad angular en °/s
//   gps_ok   : 1/0
//   imu_ok   : 1/0
//   ts       : millis() del ESP32 (para detectar pérdidas)
//
void publicarJSON() {
  Serial1.printf(
    "{\"lat\":%.10f,\"lon\":%.10f,\"x\":%.3f,\"y\":%.3f,"
    "\"sats\":%u,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
    "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
    "\"gps_ok\":%d,\"imu_ok\":%d,\"ts\":%lu}\n",
    gpsData.lat, gpsData.lon,
    gpsData.x,   gpsData.y,
    gpsData.sats,
    imuData.ax, imuData.ay, imuData.az,
    imuData.gx, imuData.gy, imuData.gz,
    gpsData.ok ? 1 : 0,
    imuData.ok ? 1 : 0,
    millis()
  );
}

// ═══════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  // USB-Serial: SOLO para logs de depuración
  Serial.begin(115200);
  delay(500);

  Serial.println("\n============================================");
  Serial.println("  GrandRoot — ESP32 Sensores");
  Serial.println("  GPS + IMU MPU6050");
  Serial.println("  Salida: Serial1 TX → ESP32 PID");
  Serial.println("============================================");

  // I2C + IMU
  Wire.begin(21, 22);
  delay(500);
  imuData.ok = initIMU();

  // Serial2 para el GPS
  Serial2.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.printf("[GPS] Serial2 RX=%d TX=%d @ %d baud\n",
                GPS_RX_PIN, GPS_TX_PIN, GPS_BAUDRATE);

  // Serial1 para el link al ESP32 PID
  Serial1.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  Serial.printf("[LINK] Serial1 TX=%d → ESP32 PID @ %d baud\n",
                LINK_TX_PIN, LINK_BAUD);

  Serial.printf("[SISTEMA] Publicando cada %d ms\n", INTERVALO_MS);
  Serial.println("--------------------------------------------");
}

// ═══════════════════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════════════════

void loop() {

  // ── Alimentar TinyGPS++ con todo lo disponible ───────────────────────────
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    if (gps.encode(c)) {
      if (gps.location.isValid()) {
        gpsData.lat  = gps.location.lat();
        gpsData.lon  = gps.location.lng();
        gpsData.x    = gpsToX(gpsData.lat, gpsData.lon);
        gpsData.y    = gpsToY(gpsData.lat, gpsData.lon);
        gpsData.sats = gps.satellites.value();
        gpsData.ok   = true;
      } else {
        gpsData.sats = gps.satellites.value();
        gpsData.ok   = false;
      }
    }
  }

  // ── Advertir si el GPS no manda nada (cableado suelto?) ──────────────────
  if (millis() > 10000 && gps.charsProcessed() < 10) {
    static unsigned long lastWarn = 0;
    if (millis() - lastWarn > 5000) {
      Serial.println("[GPS][WARN] Sin datos. Revisa cableado RXD2/TXD2.");
      lastWarn = millis();
    }
  }

  // ── Leer IMU ─────────────────────────────────────────────────────────────
  if (imuData.ok) {
    leerIMU();
  } else {
    static bool ultimo_estado_imu = false;
    bool nuevo = initIMU();
    if (nuevo && !ultimo_estado_imu) {
      Serial.println("[IMU] Reconectado correctamente.");
    }
    imuData.ok = nuevo;
    ultimo_estado_imu = nuevo;
  }

  // ── Publicar al ESP32 PID al ritmo de INTERVALO_MS ──────────────────────
  unsigned long ahora = millis();
  if (ahora - t_ultimo_envio >= INTERVALO_MS) {
    t_ultimo_envio = ahora;
    publicarJSON();
  }
}
