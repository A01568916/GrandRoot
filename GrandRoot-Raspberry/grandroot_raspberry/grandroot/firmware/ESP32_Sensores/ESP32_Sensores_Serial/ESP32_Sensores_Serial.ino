/*
 ╔══════════════════════════════════════════════════════════════════════════════╗
 ║  GrandRoot — ESP32 SENSORES  [FreeRTOS + Serial con Raspberry Pi]            ║
 ║  GPS NEO-6M/8M (TinyGPS++)  +  IMU MPU6050 (I2C)                             ║
 ║                                                                              ║
 ║  ─── CAMBIOS RESPECTO A LA VERSIÓN ESP-NOW ───────────────────────────────── ║
 ║                                                                              ║
 ║  • WiFi y ESP-NOW ELIMINADOS completamente.                                  ║
 ║  • Comunicación por Serial USB con la Raspberry Pi.                          ║
 ║  • La Raspberry Pi reenvía los datos al dashboard por Socket.IO.             ║
 ║                                                                              ║
 ║  ─── PROTOCOLO DE SALIDA ────────────────────────────────────────────────── ║
 ║                                                                              ║
 ║    Cada INTERVALO_MS milisegundos, imprime por Serial:                       ║
 ║      SENS,{"lat":..,"lon":..,"x":..,"y":..,"sats":..,"ax":..,"ay":..,"az":..║
 ║            "gx":..,"gy":..,"gz":..,"gps_ok":..,"imu_ok":..,"ts":..}         ║
 ║                                                                              ║
 ║    Formato idéntico al que mandaba el ESP32 PID por WebSocket.               ║
 ║    La Raspberry lo parsea con parsear_sensores() y emite socket.on('sens').  ║
 ║                                                                              ║
 ║  ─── PINOUT (sin cambios) ──────────────────────────────────────────────── ║
 ║                                                                              ║
 ║    GPS TXD    →  ESP32 RXD2 (GPIO 16)                                        ║
 ║    GPS RXD    →  ESP32 TXD2 (GPIO 17)  (no se usa)                           ║
 ║    GPS VIN    →  5V o 3.3V según módulo                                      ║
 ║    GPS GND    →  GND                                                         ║
 ║                                                                              ║
 ║    MPU6050 SCL →  ESP32 GPIO 22                                              ║
 ║    MPU6050 SDA →  ESP32 GPIO 21                                              ║
 ║    MPU6050 VCC →  3V3                                                        ║
 ║    MPU6050 GND →  GND                                                        ║
 ║    MPU6050 AD0 →  GND  (dirección I2C = 0x68)                                ║
 ║                                                                              ║
 ║  ─── CONEXIÓN A RASPBERRY PI ──────────────────────────────────────────── ║
 ║                                                                              ║
 ║    ESP32 USB → Raspberry Pi  (/dev/ttySensores)                              ║
 ║    Baud rate: 115200                                                         ║
 ║                                                                              ║
 ║    Para crear el alias de puerto en la Raspberry Pi, agrega en               ║
 ║    /etc/udev/rules.d/99-grandroot.rules (ver README):                        ║
 ║      SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",   ║
 ║      ATTRS{serial}=="<serial_del_esp32>", SYMLINK+="ttySensores"             ║
 ║                                                                              ║
 ║  Librerías requeridas (Library Manager):                                     ║
 ║    TinyGPS++                                                                 ║
 ╚══════════════════════════════════════════════════════════════════════════════╝
*/

// ═══════════════════════════════════════════════════════════════════════════
// LIBRERÍAS
// ═══════════════════════════════════════════════════════════════════════════

#include <Wire.h>
#include <TinyGPS++.h>

// ═══════════════════════════════════════════════════════════════════════════
// CONFIGURACIÓN
// ═══════════════════════════════════════════════════════════════════════════

#define GPS_RX_PIN    16
#define GPS_TX_PIN    17
#define GPS_BAUDRATE  9600
#define SERIAL_BAUD   115200

#define IMU_ADDR      0x68
#define ACCEL_SCALE   16384.0f
#define GYRO_SCALE    131.0f

// Intervalo de envío a la Raspberry Pi (ms)
#define INTERVALO_MS  200

// Timeout para operaciones I2C
#define I2C_TIMEOUT_MS  10

// Referencia GPS local (Chihuahua)
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
// VARIABLES GLOBALES COMPARTIDAS ENTRE TAREAS
// ═══════════════════════════════════════════════════════════════════════════

SemaphoreHandle_t xDatosMutex;

struct DatosGPS {
  double lat   = 0.0;
  double lon   = 0.0;
  float  x     = 0.0;
  float  y     = 0.0;
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

TinyGPSPlus gps;

// ═══════════════════════════════════════════════════════════════════════════
// CONVERSIÓN GPS → X/Y (metros respecto a referencia)
// ═══════════════════════════════════════════════════════════════════════════

inline double gpsToX(double lat, double lon) {
  return (lon - LON_REF) * cos(LAT_REF * DEG_TO_RAD) * 111320.0;
}
inline double gpsToY(double lat, double lon) {
  return (lat - LAT_REF) * 110540.0;
}

// ═══════════════════════════════════════════════════════════════════════════
// FUNCIONES I2C (MPU6050) — con timeout
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
  if (Wire.endTransmission(false) != 0) return 0xFF;
  Wire.requestFrom((uint8_t)IMU_ADDR, (uint8_t)1);
  unsigned long t = millis();
  while (!Wire.available()) {
    if (millis() - t > I2C_TIMEOUT_MS) return 0xFF;
    vTaskDelay(1);
  }
  return Wire.read();
}

bool imu_leerBytes(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    memset(buf, 0, n);
    return false;
  }
  Wire.requestFrom((uint8_t)IMU_ADDR, n);
  unsigned long t = millis();
  for (uint8_t i = 0; i < n; i++) {
    while (!Wire.available()) {
      if (millis() - t > I2C_TIMEOUT_MS) {
        while (i < n) buf[i++] = 0;
        return false;
      }
      vTaskDelay(1);
    }
    buf[i] = Wire.read();
  }
  return true;
}

inline int16_t combinar(uint8_t hi, uint8_t lo) {
  return (int16_t)((uint16_t)hi << 8 | lo);
}

// ═══════════════════════════════════════════════════════════════════════════
// INIT IMU
// ═══════════════════════════════════════════════════════════════════════════

bool imu_init() {
  Wire.begin();
  Wire.setClock(400000);
  imu_escribir(REG_PWR_MGMT_1, 0x00);  // Wake up
  delay(100);
  uint8_t whoami = imu_leer(REG_WHO_AM_I);
  if (whoami != 0x68 && whoami != 0x72) {
    Serial.printf("[IMU] WHO_AM_I inesperado: 0x%02X\n", whoami);
    return false;
  }
  imu_escribir(REG_SMPLRT_DIV, 0x04);  // 200 Hz
  imu_escribir(REG_CONFIG,     0x03);  // DLPF 44 Hz
  imu_escribir(REG_GYRO_CFG,   0x00);  // ±250 °/s
  imu_escribir(REG_ACCEL_CFG,  0x00);  // ±2g
  Serial.println("[IMU] MPU6050 inicializado OK");
  return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// TAREA FREERTOS — IMU (Core 0)
// ═══════════════════════════════════════════════════════════════════════════

void tareaIMU(void* pvParameters) {
  bool ok = imu_init();
  if (!ok) Serial.println("[IMU] ERROR al inicializar.");

  for (;;) {
    uint8_t raw[6];
    DatosIMU local;

    if (!imu_leerBytes(REG_ACCEL_XOUT, raw, 6)) { ok = false; vTaskDelay(pdMS_TO_TICKS(100)); continue; }
    local.ax = combinar(raw[0], raw[1]) / ACCEL_SCALE;
    local.ay = combinar(raw[2], raw[3]) / ACCEL_SCALE;
    local.az = combinar(raw[4], raw[5]) / ACCEL_SCALE;

    if (!imu_leerBytes(REG_GYRO_XOUT, raw, 6)) { ok = false; vTaskDelay(pdMS_TO_TICKS(100)); continue; }
    local.gx = combinar(raw[0], raw[1]) / GYRO_SCALE;
    local.gy = combinar(raw[2], raw[3]) / GYRO_SCALE;
    local.gz = combinar(raw[4], raw[5]) / GYRO_SCALE;
    local.ok = true;
    ok = true;

    if (xSemaphoreTake(xDatosMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      imuData = local;
      xSemaphoreGive(xDatosMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(50));  // 20 Hz
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// TAREA FREERTOS — GPS + ENVÍO SERIAL (Core 1)
// ═══════════════════════════════════════════════════════════════════════════

void tareaGPS_Envio(void* pvParameters) {
  Serial2.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.printf("[GPS] Serial2 RX=%d @ %d baud\n", GPS_RX_PIN, GPS_BAUDRATE);

  unsigned long t_ultimo_envio = 0;
  unsigned long t_warn_gps     = 0;

  for (;;) {
    // ── Alimentar TinyGPS++ ──────────────────────────────────────────────
    while (Serial2.available() > 0) {
      char c = Serial2.read();
      if (gps.encode(c)) {
        if (xSemaphoreTake(xDatosMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          if (gps.location.isValid()) {
            gpsData.lat  = gps.location.lat();
            gpsData.lon  = gps.location.lng();
            gpsData.x    = (float)gpsToX(gpsData.lat, gpsData.lon);
            gpsData.y    = (float)gpsToY(gpsData.lat, gpsData.lon);
            gpsData.sats = gps.satellites.value();
            gpsData.ok   = true;
          } else {
            gpsData.sats = gps.satellites.value();
            gpsData.ok   = false;
          }
          xSemaphoreGive(xDatosMutex);
        }
      }
    }

    if (millis() > 10000 && gps.charsProcessed() < 10) {
      if (millis() - t_warn_gps > 5000) {
        Serial.println("[GPS][WARN] Sin datos — revisa cableado.");
        t_warn_gps = millis();
      }
    }

    // ── Publicar cada INTERVALO_MS por Serial ────────────────────────────
    unsigned long ahora = millis();
    if (ahora - t_ultimo_envio >= INTERVALO_MS) {
      t_ultimo_envio = ahora;

      DatosGPS gLocal;
      DatosIMU iLocal;
      if (xSemaphoreTake(xDatosMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        gLocal = gpsData;
        iLocal = imuData;
        xSemaphoreGive(xDatosMutex);
      }

      // Formato SENS,{json} — compatible con parsear_sensores() en la Raspberry
      Serial.printf(
        "SENS,{\"lat\":%.10f,\"lon\":%.10f,\"x\":%.3f,\"y\":%.3f,"
        "\"sats\":%u,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
        "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
        "\"gps_ok\":%d,\"imu_ok\":%d,\"ts\":%lu}\n",
        gLocal.lat, gLocal.lon, gLocal.x, gLocal.y,
        gLocal.sats,
        iLocal.ax, iLocal.ay, iLocal.az,
        iLocal.gx, iLocal.gy, iLocal.gz,
        gLocal.ok ? 1 : 0,
        iLocal.ok ? 1 : 0,
        (unsigned long)ahora
      );
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  Serial.println("\n============================================");
  Serial.println("  GrandRoot — ESP32 Sensores [Serial]");
  Serial.println("============================================");
  Serial.println("  Enviando SENS,{json} cada 200 ms por Serial.");
  Serial.println("============================================");

  xDatosMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(tareaIMU,       "tareaIMU",  4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(tareaGPS_Envio, "tareaGPS",  6144, NULL, 2, NULL, 1);

  Serial.println("[SISTEMA] Tareas iniciadas.");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
