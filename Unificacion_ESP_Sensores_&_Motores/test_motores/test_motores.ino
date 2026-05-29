/*
 * test_motores.ino
 * Diagnóstico mínimo de los motores. Sin PID, sin WiFi, sin encoders.
 *
 * Recorre estos 4 tests y los va imprimiendo por USB-Serial:
 *
 *   TEST 1: EN=LOW (drivers habilitados si son activos en bajo)
 *           DAC sube de 0 → 255 en motor IZQ
 *
 *   TEST 2: Igual pero con EN=HIGH (drivers habilitados si son activos en alto)
 *
 *   TEST 3: Igual pero motor DER, EN=LOW
 *
 *   TEST 4: Igual pero motor DER, EN=HIGH
 *
 * MIRA EL MOTOR y reporta en qué test se movió:
 *   - Solo TEST 1 y 3 se mueven  →  EN es activo en LOW (código actual está bien)
 *   - Solo TEST 2 y 4 se mueven  →  EN es activo en HIGH (hay que invertir el código)
 *   - NINGUNO se mueve           →  problema de cableado, alimentación o driver
 *
 * Conecta una sola batería/fuente a la vez, asegúrate de tener la fuente
 * de potencia de los motores conectada (no solo el USB del ESP32).
 */

// === PINES (los mismos del Control_PID_WIFI.ino) ===
#define SV_SIGNAL_IZQ  25     // DAC izquierda
#define SV_SIGNAL_DER  26     // DAC derecha
#define FR_IZQ         14     // dirección izquierda
#define FR_DER         27     // dirección derecha
#define EN_IZQ         21     // enable izquierda
#define EN_DER         33     // enable derecha

void rampa(int dac_pin, int en_pin, bool en_activo_en_high, const char* nombre) {
  Serial.println("--------------------------------------------------");
  Serial.printf(">>> %s (EN_PIN=%d, activo_en=%s)\n",
                nombre, en_pin, en_activo_en_high ? "HIGH" : "LOW");
  Serial.println("    Mira el motor por 8 segundos.");
  Serial.println("    Verás el DAC subir de 0 a 255.");

  // Configurar pin EN
  pinMode(en_pin, OUTPUT);
  digitalWrite(en_pin, en_activo_en_high ? HIGH : LOW);

  // Rampa lenta y visible
  for (int v = 0; v <= 255; v += 5) {
    dacWrite(dac_pin, v);
    Serial.printf("    DAC=%3d\n", v);
    delay(150);
  }

  // Mantener un segundo a tope
  Serial.println("    DAC=255 (manteniendo 2 s)...");
  delay(2000);

  // Apagar
  dacWrite(dac_pin, 0);
  pinMode(en_pin, INPUT);   // desactivar
  Serial.println("    OFF.\n");

  delay(2000);   // pausa entre tests
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("\n==================================================");
  Serial.println("  TEST MOTORES  -  GrandRoot");
  Serial.println("==================================================");
  Serial.println("  Reporta en QUE TEST se mueve el motor.");
  Serial.println("==================================================\n");

  // Pines de dirección — todos en HIGH (una sola dirección, hacia adelante)
  pinMode(FR_IZQ, OUTPUT); digitalWrite(FR_IZQ, HIGH);
  pinMode(FR_DER, OUTPUT); digitalWrite(FR_DER, LOW);   // ojo, en el original DER se invierte

  // Asegurarse de que ambos DAC empiecen en 0
  dacWrite(SV_SIGNAL_IZQ, 0);
  dacWrite(SV_SIGNAL_DER, 0);

  delay(1000);

  Serial.println("Espera 3 segundos antes de empezar...");
  delay(3000);
}

void loop() {
  // TEST 1: motor IZQ con EN=LOW
  rampa(SV_SIGNAL_IZQ, EN_IZQ, false, "TEST 1 - Motor IZQ, EN=LOW");

  // TEST 2: motor IZQ con EN=HIGH
  rampa(SV_SIGNAL_IZQ, EN_IZQ, true,  "TEST 2 - Motor IZQ, EN=HIGH");

  // TEST 3: motor DER con EN=LOW
  rampa(SV_SIGNAL_DER, EN_DER, false, "TEST 3 - Motor DER, EN=LOW");

  // TEST 4: motor DER con EN=HIGH
  rampa(SV_SIGNAL_DER, EN_DER, true,  "TEST 4 - Motor DER, EN=HIGH");

  Serial.println("\n==================================================");
  Serial.println("  CICLO COMPLETO.  Reseteando en 5 s...");
  Serial.println("==================================================\n");
  delay(5000);
}
