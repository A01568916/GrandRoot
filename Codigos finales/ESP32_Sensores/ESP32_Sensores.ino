/*
 ╔══════════════════════════════════════════════════════════════════════════════╗
 ║  GrandRoot — ESP32 SENSORES  [FreeRTOS + ESP-NOW]                            ║
 ║  GPS NEO-6M/8M (TinyGPS++)  +  IMU MPU6050 (I2C)                             ║
 ║                                                                              ║
 ║  ─── CAMBIOS RESPECTO A LA VERSIÓN UART ──────────────────────────────────── ║
 ║                                                                              ║
 ║  • UART eliminado completamente. Ya NO hay cable entre los ESP32.            ║
 ║  • Comunicación vía ESP-NOW (Wi-Fi 2.4 GHz, latencia ~1 ms).                ║
 ║  • I2C (MPU6050) corre en su propia tarea FreeRTOS en Core 0.               ║
 ║    Si el sensor se cuelga, solo esa tarea se bloquea; el GPS y              ║
 ║    el envío ESP-NOW siguen funcionando en Core 1.                           ║
 ║  • GPS + publicación ESP-NOW corren en Core 1 (donde ya estaba el loop).    ║
 ║  • Datos compartidos entre tareas protegidos con SemaphoreHandle_t.         ║
 ║                                                                              ║
 ║  ─── PINOUT (sin cambios respecto a versión anterior) ──────────────────── ║
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
 ║    >>> CABLE UART ELIMINADO — ya no se necesita <<<                          ║
 ║                                                                              ║
 ║  ─── PRIMERO QUE HACER ────────────────────────────────────────────────── ║
 ║                                                                              ║
 ║  1. Sube este sketch al ESP32 SENSORES y abre el Monitor Serie.              ║
 ║     Verás la línea:  [ESP-NOW] Mi MAC: XX:XX:XX:XX:XX:XX                     ║
 ║  2. Copia esa MAC.                                                           ║
 ║  3. Pégala en Control_PID_WIFI_espnow.ino como PEER_MAC.                    ║
 ║  4. Sube ese sketch al ESP32 PID.                                            ║
 ║                                                                              ║
 ║  Librerías requeridas (Library Manager):                                     ║
 ║    TinyGPS++                                                                 ║
 ║    (ESP-NOW y FreeRTOS vienen incluidos en el core ESP32 de Arduino)        ║
 ╚══════════════════════════════════════════════════════════════════════════════╝
*/

// ═══════════════════════════════════════════════════════════════════════════
// LIBRERÍAS
// ═══════════════════════════════════════════════════════════════════════════

#include <Wire.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_mac.h>

// ═══════════════════════════════════════════════════════════════════════════
// CONFIGURACIÓN
// ═══════════════════════════════════════════════════════════════════════════

#define GPS_RX_PIN    16
#define GPS_TX_PIN    17
#define GPS_BAUDRATE  9600

#define IMU_ADDR      0x68
#define ACCEL_SCALE   16384.0f
#define GYRO_SCALE    131.0f

// Intervalo de envío al ESP32 PID (ms)
#define INTERVALO_MS  200

// Timeout para operaciones I2C — si Wire.requestFrom() no responde
// en este tiempo, la tarea IMU abandona esa lectura y reintenta al
// siguiente ciclo en lugar de colgarse para siempre.
#define I2C_TIMEOUT_MS  10

// Debug por USB-Serial
#define DEBUG_USB  1

// Referencia GPS local
const double LAT_REF = 28.7385775000;
const double LON_REF = -106.1217361670;

// ── MAC del ESP32 PID ────────────────────────────────────────────────────────
// IMPORTANTE: reemplaza con la MAC real de tu ESP32 PID.
// La encuentras en el Monitor Serie del ESP32 PID al inicio, o con:
//   Serial.println(WiFi.macAddress());
// Formato: { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF }
uint8_t PEER_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// ═══════════════════════════════════════════════════════════════════════════
// ESTRUCTURA DE DATOS ESP-NOW
// ═══════════════════════════════════════════════════════════════════════════
//
// Enviamos una struct binaria en lugar de JSON para aprovechar el límite
// de 250 bytes de ESP-NOW con margen de sobra (esta struct ocupa ~60 bytes).
// El receptor la deserializa y arma el JSON para el WebSocket.
//
struct __attribute__((packed)) PaqueteSensores {
  double  lat;
  double  lon;
  float   x;
  float   y;
  uint8_t sats;
  float   ax, ay, az;
  float   gx, gy, gz;
  uint8_t gps_ok;
  uint8_t imu_ok;
  uint32_t ts;
};

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

// El semáforo protege gpsData e imuData para que la tarea IMU (Core 0)
// y la tarea GPS/envío (Core 1) no lean/escriban simultáneamente.
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

// Flag para saber si ESP-NOW ya tiene peer registrado
bool espnow_listo = false;

// ═══════════════════════════════════════════════════════════════════════════
// FUNCIONES I2C (MPU6050) — con timeout para no bloquearse
// ═══════════════════════════════════════════════════════════════════════════

void imu_escribir(uint8_t reg, uint8_t valor) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(valor);
  Wire.endTransmission();
}

// Versión con timeout: retorna 0xFF si el sensor no responde a tiempo.
uint8_t imu_leer(uint8_t reg) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  Wire.requestFrom((uint8_t)IMU_ADDR, (uint8_t)1);
  unsigned long t = millis();
  while (!Wire.available()) {
    if (millis() - t > I2C_TIMEOUT_MS) return 0xFF;
    vTaskDelay(1);  // cede CPU en lugar de busy-wait
  }
  return Wire.read();
}

// Versión con timeout: rellena con 0 si el sensor no responde.
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

bool initIMU() {
  uint8_t id = imu_leer(REG_WHO_AM_I);
  Serial.printf("[IMU] WHO_AM_I = 0x%02X\n", id);
  if (id != 0x68 && id != 0x71 && id != 0x73) {
    Serial.println("[IMU][ERROR] Sensor no detectado.");
    return false;
  }
  imu_escribir(REG_PWR_MGMT_1, 0x00);
  vTaskDelay(pdMS_TO_TICKS(100));
  imu_escribir(REG_SMPLRT_DIV, 0x07);
  imu_escribir(REG_CONFIG,     0x06);
  imu_escribir(REG_GYRO_CFG,   0x00);
  imu_escribir(REG_ACCEL_CFG,  0x00);
  Serial.println("[IMU] OK — MPU6050 configurado.");
  return true;
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
// CALLBACK ESP-NOW (confirmación de envío — solo para debug)
// ═══════════════════════════════════════════════════════════════════════════
//
// En Arduino ESP32 core 3.x la firma cambió: el primer argumento pasó de
// (const uint8_t* mac) a (const wifi_tx_info_t* info).
// Usamos #if para compilar la firma correcta según la versión del core.
//
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onEnvioESPNOW(const wifi_tx_info_t* info, esp_now_send_status_t status) {
#else
void onEnvioESPNOW(const uint8_t* mac, esp_now_send_status_t status) {
#endif
#if DEBUG_USB
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("[ESP-NOW] ⚠ Envío fallido — ¿el PID está encendido?");
  }
#endif
}

// ═══════════════════════════════════════════════════════════════════════════
// TAREA FREERTOS — IMU (Core 0)
// ═══════════════════════════════════════════════════════════════════════════
//
// Corre en Core 0, separada del GPS y del envío.
// Si Wire.requestFrom() tarda más de I2C_TIMEOUT_MS, la función imu_leerBytes()
// retorna false y esta tarea simplemente espera al siguiente ciclo (50 ms).
// El resto del sistema (GPS, ESP-NOW) sigue funcionando sin interrupción.
//
void tareaIMU(void* pvParameters) {
  // Wire.begin() también debe llamarse desde esta tarea si se usa en Core 0.
  // Lo inicializamos aquí para asegurar que el driver I2C vive en el mismo core.
  Wire.begin(21, 22);
  Wire.setClock(400000);  // 400 kHz (fast mode)

  bool ok = initIMU();

  for (;;) {
    if (!ok) {
      // Reintento periódico si el sensor desaparece
      ok = initIMU();
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    uint8_t raw[6];
    DatosIMU local;

    // Acelerómetro
    if (!imu_leerBytes(REG_ACCEL_XOUT, raw, 6)) { ok = false; continue; }
    local.ax = combinar(raw[0], raw[1]) / ACCEL_SCALE;
    local.ay = combinar(raw[2], raw[3]) / ACCEL_SCALE;
    local.az = combinar(raw[4], raw[5]) / ACCEL_SCALE;

    // Giroscopio
    if (!imu_leerBytes(REG_GYRO_XOUT, raw, 6)) { ok = false; continue; }
    local.gx = combinar(raw[0], raw[1]) / GYRO_SCALE;
    local.gy = combinar(raw[2], raw[3]) / GYRO_SCALE;
    local.gz = combinar(raw[4], raw[5]) / GYRO_SCALE;
    local.ok = true;

    // Escribir en variable global protegida con mutex
    if (xSemaphoreTake(xDatosMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      imuData = local;
      xSemaphoreGive(xDatosMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(50));  // 20 Hz — el PID solo lee cada 200 ms
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// TAREA FREERTOS — GPS + ENVÍO ESP-NOW (Core 1)
// ═══════════════════════════════════════════════════════════════════════════
//
// Core 1 es el mismo donde corre el loop() de Arduino. Al usar xTaskCreatePinnedToCore
// con Core 1, esta tarea convive con el scheduler de Arduino sin problema.
// No hay un loop() en este sketch — todo el trabajo ocurre en las tareas.
//
void tareaGPS_Envio(void* pvParameters) {
  // GPS por Serial2
  Serial2.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.printf("[GPS] Serial2 RX=%d @ %d baud\n", GPS_RX_PIN, GPS_BAUDRATE);

  unsigned long t_ultimo_envio = 0;
  unsigned long t_warn_gps     = 0;

  for (;;) {
    // ── Alimentar TinyGPS++ ────────────────────────────────────────────────
    while (Serial2.available() > 0) {
      char c = Serial2.read();
      if (gps.encode(c)) {
        // Tomar mutex solo para escribir gpsData
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

    // ── Advertencia si el GPS no manda nada ───────────────────────────────
    if (millis() > 10000 && gps.charsProcessed() < 10) {
      if (millis() - t_warn_gps > 5000) {
        Serial.println("[GPS][WARN] Sin datos — revisa cableado.");
        t_warn_gps = millis();
      }
    }

    // ── Publicar cada INTERVALO_MS vía ESP-NOW ────────────────────────────
    unsigned long ahora = millis();
    if (ahora - t_ultimo_envio >= INTERVALO_MS) {
      t_ultimo_envio = ahora;

      // Leer datos protegidos
      PaqueteSensores pkt;
      if (xSemaphoreTake(xDatosMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        pkt.lat    = gpsData.lat;
        pkt.lon    = gpsData.lon;
        pkt.x      = gpsData.x;
        pkt.y      = gpsData.y;
        pkt.sats   = gpsData.sats;
        pkt.ax     = imuData.ax;
        pkt.ay     = imuData.ay;
        pkt.az     = imuData.az;
        pkt.gx     = imuData.gx;
        pkt.gy     = imuData.gy;
        pkt.gz     = imuData.gz;
        pkt.gps_ok = gpsData.ok ? 1 : 0;
        pkt.imu_ok = imuData.ok ? 1 : 0;
        pkt.ts     = (uint32_t)ahora;
        xSemaphoreGive(xDatosMutex);
      }

      if (espnow_listo) {
        esp_now_send(PEER_MAC, (uint8_t*)&pkt, sizeof(pkt));
      }

#if DEBUG_USB
      Serial.printf(
        "[%lums] GPS:%s sats=%u  lat=%.6f lon=%.6f  X=%.2f Y=%.2f  | "
        "IMU:%s  ax=%.2f ay=%.2f az=%.2f  gz=%.1f  heap=%u\n",
        ahora,
        pkt.gps_ok ? "FIX" : "---",
        pkt.sats,
        pkt.lat, pkt.lon, pkt.x, pkt.y,
        pkt.imu_ok ? "OK " : "---",
        pkt.ax, pkt.ay, pkt.az, pkt.gz,
        ESP.getFreeHeap()
      );
#endif
    }

    // Ceder CPU brevemente para que FreeRTOS atienda otras tareas
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n============================================");
  Serial.println("  GrandRoot — ESP32 SENSORES [FreeRTOS+ESP-NOW]");
  Serial.println("============================================");

  // ── Wi-Fi en modo Station (requerido para ESP-NOW) ────────────────────
  // Nota: el ESP32 PID corre en modo AP. Este ESP32 no se conecta al AP;
  // ESP-NOW funciona a nivel de trama 802.11, independiente de si hay
  // asociación a una red. No hay conflicto.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  Serial.printf("[ESP-NOW] Mi MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
              mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.println("         ^ Copia esta MAC al ESP32 PID como PEER_MAC");

  // ── Inicializar ESP-NOW ───────────────────────────────────────────────
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW][ERROR] Falló la inicialización. Reiniciando...");
    delay(3000);
    ESP.restart();
  }
  esp_now_register_send_cb(onEnvioESPNOW);

  // Registrar peer (el ESP32 PID)
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, PEER_MAC, 6);
  peer.channel = 0;   // canal actual
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[ESP-NOW][ERROR] No se pudo agregar el peer. Verifica PEER_MAC.");
    // No hacemos restart — el envío fallará silenciosamente, útil para debug
  } else {
    espnow_listo = true;
    Serial.println("[ESP-NOW] Peer registrado OK.");
  }

  // ── Mutex para proteger gpsData / imuData ────────────────────────────
  xDatosMutex = xSemaphoreCreateMutex();

  // ── Crear tarea IMU en Core 0 ─────────────────────────────────────────
  // Stack 4096 bytes, prioridad 2 (media).
  // Wire.begin() se llama DENTRO de la tarea, no aquí, para que el driver
  // I2C quede ligado a Core 0 y no haya conflictos de interrupción.
  xTaskCreatePinnedToCore(
    tareaIMU,       // función
    "tareaIMU",     // nombre (para debug con vTaskList)
    4096,           // stack en bytes
    NULL,           // parámetro
    2,              // prioridad (0=mínima, configMAX_PRIORITIES-1=máxima)
    NULL,           // handle (no lo necesitamos)
    0               // Core 0
  );

  // ── Crear tarea GPS+Envío en Core 1 ──────────────────────────────────
  // Stack 6144 bytes (TinyGPS++ y printf consumen más).
  xTaskCreatePinnedToCore(
    tareaGPS_Envio,
    "tareaGPS",
    6144,
    NULL,
    2,
    NULL,
    1               // Core 1
  );

  Serial.println("[SISTEMA] Tareas iniciadas.");
  Serial.println("--------------------------------------------");
}

// ── loop vacío ────────────────────────────────────────────────────────────
// Con FreeRTOS todo el trabajo ocurre en las tareas creadas arriba.
// Loop sigue existiendo pero solo mantiene el watchdog de Arduino.
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
