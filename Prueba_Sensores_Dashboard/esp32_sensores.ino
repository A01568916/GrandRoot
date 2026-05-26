/*
 ╔══════════════════════════════════════════════════════════════════════════════╗
 ║  GrandRoot — ESP32 SENSORES                                                  ║
 ║  Combina GPS (TinyGPS++ por Serial2) + IMU MPU9250 (I2C)                    ║
 ║  Publica un paquete JSON por USB-Serial cada INTERVALO_MS ms                ║
 ║                                                                              ║
 ║  PINOUT:                                                                     ║
 ║    GPS TXD  → ESP32 RXD2 (GPIO 16)                                           ║
 ║    GPS RXD  → ESP32 TXD2 (GPIO 17)                                           ║
 ║    GPS VIN  → ESP32 VIN  (5 V o 3.3 V segun modulo)                         ║
 ║    GPS GND  → ESP32 GND                                                      ║
 ║                                                                              ║
 ║    MPU9250 SCL → ESP32 GPIO 22                                               ║
 ║    MPU9250 SDA → ESP32 GPIO 21                                               ║
 ║    MPU9250 VCC → ESP32 3V3                                                   ║
 ║    MPU9250 GND → ESP32 GND                                                   ║
 ║    MPU9250 AD0 → GND  (direccion I2C = 0x68)                                 ║
 ║                                                                              ║
 ║  Libreria requerida:                                                         ║
 ║    TinyGPS++  (instalar en Library Manager de Arduino IDE)                   ║
 ║                                                                              ║
 ║  Salida por USB-Serial (115200 baud):                                        ║
 ║    {"lat":28.7385,"lon":-106.1217,"x":0.0,"y":0.0,"sats":7,               ║
 ║     "ax":0.01,"ay":-0.02,"az":0.98,"gx":0.1,"gy":-0.3,"gz":0.05,          ║
 ║     "gps_ok":true,"imu_ok":true,"ts":12345}                                 ║
 ║                                                                              ║
 ║  El campo "ts" es millis() del ESP32 — sirve para detectar paquetes         ║
 ║  perdidos en la Raspberry Pi.                                                ║
 ╚══════════════════════════════════════════════════════════════════════════════╝
*/

#include <Wire.h>
#include <TinyGPS++.h>

// ═══════════════════════════════════════════════════════════════════════════
// CONFIGURACION — modifica aqui si cambias hardware
// ═══════════════════════════════════════════════════════════════════════════

#define GPS_RX_PIN    16          // ESP32 RXD2 ← GPS TXD
#define GPS_TX_PIN    17          // ESP32 TXD2 → GPS RXD (no se usa, pero se declara)
#define GPS_BAUDRATE  9600        // baud del modulo GPS (default NEO-6M / NEO-8M)

#define IMU_ADDR      0x68        // AD0=GND → 0x68  |  AD0=3.3V → 0x69
#define ACCEL_SCALE   16384.0f    // LSB/g  para escala ±2g
#define GYRO_SCALE    131.0f      // LSB/(°/s) para escala ±250°/s

// Intervalo de publicacion de datos en milisegundos
// GPS actualiza a ~1 Hz (1000 ms), el IMU puede ir mas rapido.
// Con 200 ms publicas el ultimo valor GPS disponible + IMU fresco.
#define INTERVALO_MS  200

// Punto de referencia GPS para coordenadas locales X/Y (metros)
// Cambia estas coordenadas al punto de inicio de tu parcela
const double LAT_REF = 28.7385775000;
const double LON_REF = -106.1217361670;

// ═══════════════════════════════════════════════════════════════════════════
// REGISTROS MPU9250
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

// Estado GPS — se actualiza cada vez que llega una trama NMEA valida
struct DatosGPS {
  double lat   = 0.0;
  double lon   = 0.0;
  double x     = 0.0;   // metros desde LAT_REF / LON_REF
  double y     = 0.0;
  uint8_t sats = 0;
  bool ok      = false;
};

// Estado IMU — se actualiza en cada ciclo de loop
struct DatosIMU {
  float ax = 0, ay = 0, az = 0;   // aceleracion en g
  float gx = 0, gy = 0, gz = 0;   // velocidad angular en °/s
  bool ok = false;
};

DatosGPS gpsData;
DatosIMU imuData;

unsigned long t_ultimo_envio = 0;

// ═══════════════════════════════════════════════════════════════════════════
// FUNCIONES AUXILIARES I2C (MPU9250)
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
// CONVERSION GPS A COORDENADAS PLANAS (metros)
// ═══════════════════════════════════════════════════════════════════════════

double gpsToX(double lat, double lon) {
  return (lon - LON_REF) * cos(LAT_REF * PI / 180.0) * 111320.0;
}

double gpsToY(double lat, double lon) {
  return (lat - LAT_REF) * 111320.0;
}

// ═══════════════════════════════════════════════════════════════════════════
// INICIALIZACION IMU
// Devuelve true si el sensor responde correctamente
// ═══════════════════════════════════════════════════════════════════════════

bool initIMU() {
  uint8_t id = imu_leer(REG_WHO_AM_I);

  // LOG: siempre imprime el ID para diagnostico
  Serial.printf("[IMU] WHO_AM_I = 0x%02X (esperado 0x71 o 0x73)\n", id);

  if (id != 0x71 && id != 0x73) {
    Serial.println("[IMU][ERROR] Sensor no detectado.");
    Serial.println("[IMU][ERROR] Revisa: cableado SDA/SCL, alimentacion 3.3V, pin AD0.");
    return false;
  }

  imu_escribir(REG_PWR_MGMT_1, 0x00);   // despertar
  delay(100);
  imu_escribir(REG_SMPLRT_DIV, 0x07);   // 125 Hz de muestreo interno
  imu_escribir(REG_CONFIG,     0x06);   // filtro pasa-bajas activo
  imu_escribir(REG_GYRO_CFG,   0x00);   // ±250 °/s
  imu_escribir(REG_ACCEL_CFG,  0x00);   // ±2 g

  Serial.println("[IMU] OK — MPU9250 configurado correctamente.");
  return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// LEER IMU (llama en cada ciclo, es rapido)
// ═══════════════════════════════════════════════════════════════════════════

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
// PUBLICAR PAQUETE JSON POR SERIAL USB
// ═══════════════════════════════════════════════════════════════════════════

void publicarJSON() {
  // Formato compacto en una sola linea terminada en \n
  // La Raspberry Pi lee con readline() y hace json.loads()
  Serial.printf(
    "{\"lat\":%.10f,\"lon\":%.10f,\"x\":%.4f,\"y\":%.4f,"
    "\"sats\":%u,\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f,"
    "\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,"
    "\"gps_ok\":%s,\"imu_ok\":%s,\"ts\":%lu}\n",
    gpsData.lat, gpsData.lon,
    gpsData.x,   gpsData.y,
    gpsData.sats,
    imuData.ax, imuData.ay, imuData.az,
    imuData.gx, imuData.gy, imuData.gz,
    gpsData.ok  ? "true" : "false",
    imuData.ok  ? "true" : "false",
    millis()
  );
}

// ═══════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  // USB-Serial hacia la Raspberry Pi (115200 baud)
  Serial.begin(115200);
  delay(500);

  Serial.println("\n============================================");
  Serial.println("  GrandRoot — ESP32 Sensores");
  Serial.println("  GPS + IMU MPU9250");
  Serial.println("============================================");

  // Serial2 para el GPS
  Serial2.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.printf("[GPS] Serial2 abierto en RX=%d TX=%d a %d baud\n",
                GPS_RX_PIN, GPS_TX_PIN, GPS_BAUDRATE);

  // I2C + IMU
  Wire.begin();   // SDA=GPIO21, SCL=GPIO22
  delay(200);
  imuData.ok = initIMU();

  // Si el IMU fallo, seguimos funcionando pero marcamos imu_ok=false
  // El sistema de vision y GPS pueden seguir operando sin el IMU

  Serial.println("[SISTEMA] Iniciando publicacion de datos...");
  Serial.printf("[SISTEMA] Intervalo de envio: %d ms\n", INTERVALO_MS);
  Serial.println("--------------------------------------------");
}

// ═══════════════════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════════════════

void loop() {

  // ── Alimentar TinyGPS++ con todos los bytes disponibles del GPS ──────────
  // Esto es no-bloqueante: solo procesa lo que ya llego, no espera
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    if (gps.encode(c)) {
      // TinyGPS++ completo una trama NMEA nueva
      if (gps.location.isValid()) {
        gpsData.lat  = gps.location.lat();
        gpsData.lon  = gps.location.lng();
        gpsData.x    = gpsToX(gpsData.lat, gpsData.lon);
        gpsData.y    = gpsToY(gpsData.lat, gpsData.lon);
        gpsData.sats = gps.satellites.value();
        gpsData.ok   = true;
      } else {
        // Satelites procesados pero sin fix todavia
        gpsData.sats = gps.satellites.value();
        gpsData.ok   = false;
      }
    }
  }

  // ── LOG: Advertir si el GPS no manda datos despues de 10 segundos ────────
  // Esto ayuda a detectar cableado suelto o modulo defectuoso
  if (millis() > 10000 && gps.charsProcessed() < 10) {
    Serial.println("[GPS][WARN] No se reciben datos del GPS. Revisa el cableado RXD2/TXD2.");
  }

  // ── Leer IMU (rapido, ~1 ms) ─────────────────────────────────────────────
  if (imuData.ok) {
    leerIMU();
  } else {
    // Intentar reinicializar IMU si estaba desconectado
    // (se intenta cada ciclo, pero solo imprime si cambia de estado)
    static bool ultimo_estado_imu = false;
    bool nuevo = initIMU();
    if (nuevo && !ultimo_estado_imu) {
      Serial.println("[IMU] Reconectado correctamente.");
    }
    imuData.ok = nuevo;
    ultimo_estado_imu = nuevo;
  }

  // ── Publicar paquete JSON al ritmo definido por INTERVALO_MS ─────────────
  unsigned long ahora = millis();
  if (ahora - t_ultimo_envio >= INTERVALO_MS) {
    t_ultimo_envio = ahora;
    publicarJSON();
  }
}
