/*
  mpu9250_lectura.ino
  ───────────────────────────────────────────────────────────────────────────
  Lee acelerómetro, giroscopio y temperatura del MPU9250 via I2C y los
  imprime por Serial en formato legible.

  Plataforma : ESP32
  Librería   : Solo Wire.h (incluida en Arduino IDE — sin librerías externas)

  Conexión física:
  ┌─────────────┬──────────────────────────────────┐
  │ MPU9250 pin │ ESP32 pin                        │
  ├─────────────┼──────────────────────────────────┤
  │ VCC         │ 3.3 V                            │
  │ GND         │ GND                              │
  │ SCL         │ GPIO 22  (I2C SCL por defecto)   │
  │ SDA         │ GPIO 21  (I2C SDA por defecto)   │
  │ AD0         │ GND  →  dirección I2C = 0x68     │
  │             │ (conectar a 3.3V para usar 0x69) │
  └─────────────┴──────────────────────────────────┘

  Escalas por defecto (sin cambiar configuración del sensor):
    Acelerómetro : ±2 g      → 16 384 LSB/g
    Giroscopio   : ±250 °/s  →    131 LSB/(°/s)
  ───────────────────────────────────────────────────────────────────────────
*/

#include <Wire.h>

// ── Dirección I2C ────────────────────────────────────────────────────────────
#define MPU9250_ADDR    0x68   // AD0=GND → 0x68 | AD0=3.3V → 0x69

// ── Registros ────────────────────────────────────────────────────────────────
#define REG_SMPLRT_DIV  0x19   // divisor de frecuencia de muestreo
#define REG_CONFIG      0x1A   // configuración DLPF
#define REG_GYRO_CFG    0x1B   // escala del giroscopio
#define REG_ACCEL_CFG   0x1C   // escala del acelerómetro
#define REG_ACCEL_XOUT  0x3B   // primer byte de acelerómetro (6 bytes)
#define REG_GYRO_XOUT   0x43   // primer byte de giroscopio  (6 bytes)
#define REG_PWR_MGMT_1  0x6B   // control de energía
#define REG_WHO_AM_I    0x75   // identificador del chip (0x71 ó 0x73)

// ── Factores de escala (defecto: ±2g, ±250°/s) ───────────────────────────────
#define ACCEL_SCALE  16384.0f
#define GYRO_SCALE     131.0f

// ── Intervalo entre lecturas (ms) ────────────────────────────────────────────
#define INTERVALO_MS  100


// ─────────────────────────────────────────────────────────────────────────────
// Funciones auxiliares I2C
// ─────────────────────────────────────────────────────────────────────────────

void escribirReg(uint8_t reg, uint8_t valor) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(reg);
  Wire.write(valor);
  Wire.endTransmission();
}

uint8_t leerReg(uint8_t reg) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

// Lee 'n' bytes consecutivos a partir de 'reg' en el buffer 'buf'
void leerBytes(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDR, n);
  for (uint8_t i = 0; i < n; i++) {
    buf[i] = Wire.available() ? Wire.read() : 0;
  }
}

// Combina dos bytes (big-endian) en un entero con signo de 16 bits
inline int16_t combinar(uint8_t hi, uint8_t lo) {
  return (int16_t)((uint16_t)hi << 8 | lo);
}


// ─────────────────────────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(9600);
  Wire.begin();   // SDA = GPIO 21, SCL = GPIO 22  (por defecto en ESP32)
  delay(250);

  Serial.println();
  Serial.println("=================================================");
  Serial.println("  MPU9250 — Acelerometro + Giroscopio (ESP32)   ");
  Serial.println("=================================================");

  // Verificar que el sensor responde
  uint8_t id = leerReg(REG_WHO_AM_I);
  Serial.printf("WHO_AM_I = 0x%02X  (esperado: 0x71 o 0x73)\n", id);
  if (id != 0x71 && id != 0x73) {
    Serial.println("\nERROR: Sensor no detectado.");
    Serial.println("  - Verifica el cableado SDA/SCL.");
    Serial.println("  - Verifica la direccion I2C (AD0 → 0x68 o 0x69).");
    Serial.println("  - Verifica alimentacion (3.3 V).");
    while (true) delay(1000);   // detener ejecución
  }

  // Despertar el MPU9250 (sale del modo sleep al encender)
  escribirReg(REG_PWR_MGMT_1, 0x00);
  delay(100);

  // Configuración opcional (puedes cambiar estos valores)
  escribirReg(REG_SMPLRT_DIV, 0x07);    // frecuencia de muestreo: 1 kHz / (1+7) = 125 Hz
  escribirReg(REG_CONFIG,     0x06);    // DLPF activado (filtra vibraciones de alta frecuencia)
  escribirReg(REG_GYRO_CFG,   0x00);    // giroscopio  ±250  °/s  (0x08=±500, 0x10=±1000, 0x18=±2000)
  escribirReg(REG_ACCEL_CFG,  0x00);    // acelerómetro ±2 g       (0x08=±4g,  0x10=±8g,   0x18=±16g)

  Serial.println("Sensor configurado correctamente.\n");
  Serial.println("  Ax(g)    Ay(g)    Az(g)   |   Gx(°/s)  Gy(°/s)  Gz(°/s)");
  Serial.println("  -------------------------------------------------------");
}


// ─────────────────────────────────────────────────────────────────────────────
// Loop
// ─────────────────────────────────────────────────────────────────────────────

void loop() {
  uint8_t raw[14];

  // Leer acelerómetro (6 bytes) y giroscopio (6 bytes) por separado
  // (se omite la temperatura que ocupa los bytes 6-7 entre ambos bloques)
  leerBytes(REG_ACCEL_XOUT, raw, 6);

  // Acelerómetro (g)
  float ax = combinar(raw[0], raw[1]) / ACCEL_SCALE;
  float ay = combinar(raw[2], raw[3]) / ACCEL_SCALE;
  float az = combinar(raw[4], raw[5]) / ACCEL_SCALE;

  // Giroscopio (°/s)  — empieza en REG_GYRO_XOUT (0x43), 6 bytes
  leerBytes(REG_GYRO_XOUT, raw, 6);
  float gx = combinar(raw[0], raw[1]) / GYRO_SCALE;
  float gy = combinar(raw[2], raw[3]) / GYRO_SCALE;
  float gz = combinar(raw[4], raw[5]) / GYRO_SCALE;

  // Imprimir datos
  Serial.printf("  %7.4f  %7.4f  %7.4f  |  %8.3f  %8.3f  %8.3f\n",
                ax, ay, az,
                gx, gy, gz);

  delay(INTERVALO_MS);
}
