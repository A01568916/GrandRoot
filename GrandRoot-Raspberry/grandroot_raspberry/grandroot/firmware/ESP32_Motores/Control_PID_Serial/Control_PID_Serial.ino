/*
 * Control_PID_Serial.ino
 * ESP32 Motores — Control PI Posicional + Serial con Raspberry Pi
 *
 * ─── CAMBIOS RESPECTO A LA VERSIÓN WIFI/ESP-NOW ─────────────────────────────
 *
 *  • WiFi, ESP-NOW y WebServer ELIMINADOS completamente.
 *    El ESP32 ya no sirve HTML ni crea un AP.
 *
 *  • Toda la comunicación es por Serial (USB) con la Raspberry Pi.
 *    La Raspberry Pi corre Flask y sirve el dashboard al navegador.
 *
 *  • Protocolo de línea simple (compatible con el dashboard existente):
 *      Salida (ESP32 → Raspberry):
 *        TEL,<medida_i>,<medida_d>,<ref_i>,<ref_d>,<error_i>,<error_d>,<dac_i>,<dac_d>
 *        ENABLE,true/false   (eco del estado de motores)
 *        PARAM_OK,nombre,valor
 *
 *      Entrada (Raspberry → ESP32, comandos del dashboard):
 *        MOVE,<vx>,<vy>      joystick  (-1.0 a 1.0)
 *        STOP
 *        EN,1 / EN,0
 *        KP,<valor>
 *        KI,<valor>
 *        PMAX,<valor>
 *        PARAM,<nombre>,<valor>
 *
 * ─── CONEXIÓN ────────────────────────────────────────────────────────────────
 *
 *   ESP32 USB → Raspberry Pi   (/dev/ttyESP32)
 *   Baud rate: 115200
 *
 *   Para crear el alias de puerto en la Raspberry Pi, agrega en
 *   /etc/udev/rules.d/99-grandroot.rules (ver README):
 *     SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",
 *     ATTRS{serial}=="<serial_del_esp32>", SYMLINK+="ttyESP32"
 */

// =====================================================
// FORWARD DECLARATIONS
// =====================================================
struct MotorState;
void stepPI(MotorState &m, float T);

// =====================================================
// LIBRERÍAS
// =====================================================
// (No se necesita WiFi, ESP-NOW ni AsyncWebServer)

// =====================================================
// PARÁMETROS
// =====================================================
#define SAMPLE_MS   200
#define SERIAL_BAUD 115200

int   PULSOS_MAX = 22;

// =====================================================
// CINEMÁTICA
// =====================================================
const float R_RUEDA = 0.1397f;
const float L_BASE  = 1.12f;

float VMAX = 4.0f;
float WMAX = 7.4f;
float OMEGA_MAX = VMAX / R_RUEDA;

// =====================================================
// PI POSICIONAL
// =====================================================
float kp = 6.0f;
float ki = 3.0f;

float INTEGRAL_MAX     = 255.0f / 3.0f;
int   REF_MIN_GIRO     = 3;
int   DAC_MIN_ARRANQUE = 70;

// =====================================================
// PINES MOTORES
// =====================================================
#define SV_SIGNAL_IZQ  25
#define SV_SIGNAL_DER  26
#define FR_IZQ         14
#define FR_DER         27
#define EN_IZQ         21
#define EN_DER         33
#define ENC_IZQ        18
#define ENC_DER        34

// =====================================================
// SWITCH DE DIAGNÓSTICO
// =====================================================
#define DESACTIVAR_IRQS_DER  0

// =====================================================
// ENCODERS
// =====================================================
volatile long cnt_izq = 0;
volatile long cnt_der = 0;
void IRAM_ATTR isr_izq() { cnt_izq++; }
void IRAM_ATTR isr_der() { cnt_der++; }

// =====================================================
// DIRECCIÓN LÓGICA
// =====================================================
bool dir_actual_izq = true;
bool dir_actual_der = true;

// =====================================================
// ESTADO MOTOR
// =====================================================
struct MotorState {
  int   ref;
  long  medida;
  float error;
  float integral;
  float u;
  int   dac;
};

MotorState motor_i = { 0, 0, 0.0f, 0.0f, 0.0f, 0 };
MotorState motor_d = { 0, 0, 0.0f, 0.0f, 0.0f, 0 };

bool motors_enabled = false;
unsigned long t_prev = 0;

// =====================================================
// RESET MOTOR
// =====================================================
void resetMotor(MotorState &m) {
  m.integral = 0.0f;
  m.u        = 0.0f;
  m.error    = 0.0f;
  m.dac      = 0;
}

// =====================================================
// DIRECCIÓN
// =====================================================
void setDirIzq(bool adelante) { digitalWrite(FR_IZQ, adelante ? HIGH : LOW); }
void setDirDer(bool adelante) { digitalWrite(FR_DER, adelante ? LOW  : HIGH); }

// =====================================================
// HABILITAR / DESHABILITAR MOTORES
// =====================================================
void enableMotores(bool on) {
  motors_enabled = on;
  digitalWrite(EN_IZQ, on ? HIGH : LOW);
  digitalWrite(EN_DER, on ? HIGH : LOW);
  if (!on) {
    dacWrite(SV_SIGNAL_IZQ, 0);
    dacWrite(SV_SIGNAL_DER, 0);
    resetMotor(motor_i);
    resetMotor(motor_d);
  }
  // Eco al dashboard
  Serial.printf("ENABLE,%s\n", on ? "true" : "false");
}

// =====================================================
// CINEMÁTICA DIFERENCIAL
// =====================================================
void cinematica(float vx, float vy, int &ref_izq, int &ref_der) {
  float V = vx * VMAX;
  float w = vy * WMAX;
  float omega_r = V / R_RUEDA + (L_BASE / (2.0f * R_RUEDA)) * w;
  float omega_l = V / R_RUEDA - (L_BASE / (2.0f * R_RUEDA)) * w;
  float scale = max(max(fabsf(omega_r), fabsf(omega_l)), OMEGA_MAX) / OMEGA_MAX;
  omega_r /= scale;
  omega_l /= scale;
  ref_izq = (int)roundf(constrain(omega_l / OMEGA_MAX, -1.0f, 1.0f) * PULSOS_MAX);
  ref_der = (int)roundf(constrain(omega_r / OMEGA_MAX, -1.0f, 1.0f) * PULSOS_MAX);
  if (ref_izq != 0)
    ref_izq = (ref_izq > 0) ? max(ref_izq,  REF_MIN_GIRO)
                             : min(ref_izq, -REF_MIN_GIRO);
  if (ref_der != 0)
    ref_der = (ref_der > 0) ? max(ref_der,  REF_MIN_GIRO)
                             : min(ref_der, -REF_MIN_GIRO);
}

// =====================================================
// APLICAR MOTORES
// =====================================================
void aplicarMotores(int ri, int rd) {
  bool dir_izq = (ri >= 0);
  bool dir_der = (rd >= 0);
  if (dir_izq != dir_actual_izq) { dacWrite(SV_SIGNAL_IZQ, 0); resetMotor(motor_i); }
  if (dir_der != dir_actual_der) { dacWrite(SV_SIGNAL_DER, 0); resetMotor(motor_d); }
  setDirIzq(dir_izq);
  setDirDer(dir_der);
  dir_actual_izq = dir_izq;
  dir_actual_der = dir_der;
  motor_i.ref = abs(ri);
  motor_d.ref = abs(rd);
}

// =====================================================
// PASO PI
// =====================================================
void stepPI(MotorState &m, float T) {
  m.error    = m.ref - m.medida;
  m.integral = constrain(m.integral + m.error * T, -INTEGRAL_MAX, INTEGRAL_MAX);
  m.u        = kp * m.error + ki * m.integral;

  int dac = (int)constrain(fabsf(m.u), 0, 255);
  if (m.ref != 0 && dac < DAC_MIN_ARRANQUE) dac = DAC_MIN_ARRANQUE;
  if (m.ref == 0) { dac = 0; resetMotor(m); }
  m.dac = dac;
}

// =====================================================
// PARSEAR COMANDO SERIAL
// =====================================================
void procesarComando(const String &msg) {
  String m = msg;
  m.trim();
  if (m.length() == 0) return;

  // MOVE,vx,vy
  if (m.startsWith("MOVE,")) {
    int c1 = m.indexOf(',', 5);
    if (c1 < 0) return;
    float vx = m.substring(5, c1).toFloat();
    float vy = m.substring(c1 + 1).toFloat();
    int ri, rd;
    cinematica(vx, vy, ri, rd);
    aplicarMotores(ri, rd);
    return;
  }

  // STOP
  if (m == "STOP") {
    aplicarMotores(0, 0);
    return;
  }

  // EN,1 o EN,0
  if (m.startsWith("EN,")) {
    enableMotores(m.charAt(3) == '1');
    return;
  }

  // KP,valor
  if (m.startsWith("KP,")) {
    kp = m.substring(3).toFloat();
    Serial.printf("PARAM_OK,kp,%.4f\n", kp);
    return;
  }

  // KI,valor
  if (m.startsWith("KI,")) {
    ki = m.substring(3).toFloat();
    Serial.printf("PARAM_OK,ki,%.4f\n", ki);
    return;
  }

  // PMAX,valor
  if (m.startsWith("PMAX,")) {
    PULSOS_MAX = (int)m.substring(5).toFloat();
    Serial.printf("PARAM_OK,pmax,%d\n", PULSOS_MAX);
    return;
  }

  // PARAM,nombre,valor (genérico del dashboard)
  if (m.startsWith("PARAM,")) {
    int c1 = m.indexOf(',', 6);
    if (c1 < 0) return;
    String nombre = m.substring(6, c1);
    float  valor  = m.substring(c1 + 1).toFloat();
    if      (nombre == "kp")   { kp = valor; }
    else if (nombre == "ki")   { ki = valor; }
    else if (nombre == "pmax") { PULSOS_MAX = (int)valor; }
    Serial.printf("PARAM_OK,%s,%.4f\n", nombre.c_str(), valor);
    return;
  }
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println("\n============================================");
  Serial.println("  GrandRoot — ESP32 Motores [Serial]");
  Serial.println("============================================");

  // Motores
  pinMode(FR_IZQ, OUTPUT);  pinMode(FR_DER, OUTPUT);
  pinMode(EN_IZQ, OUTPUT);  pinMode(EN_DER, OUTPUT);
  setDirIzq(true);          setDirDer(true);
  dacWrite(SV_SIGNAL_IZQ, 0);
  dacWrite(SV_SIGNAL_DER, 0);
  enableMotores(false);

  // Encoders
  pinMode(ENC_IZQ, INPUT_PULLUP);
  pinMode(ENC_DER, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_IZQ), isr_izq, RISING);
#if !DESACTIVAR_IRQS_DER
  attachInterrupt(digitalPinToInterrupt(ENC_DER), isr_der, RISING);
#endif

  t_prev = millis();
  Serial.println("[Sistema] Listo. Esperando comandos por Serial.");
}

// =====================================================
// LOOP — Control PI + telemetría + parseo serial
// =====================================================
void loop() {

  // ── Leer comandos seriales ──────────────────────────────────────────────
  static String serialBuf = "";
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      procesarComando(serialBuf);
      serialBuf = "";
    } else if (c != '\r') {
      serialBuf += c;
    }
  }

  // ── Control PI cada SAMPLE_MS ms ───────────────────────────────────────
  unsigned long ahora = millis();
  if (ahora - t_prev >= SAMPLE_MS) {
    t_prev = ahora;

    // Leer encoders
    noInterrupts();
    long pi = cnt_izq;
    long pd = cnt_der;
    cnt_izq = 0;
    cnt_der = 0;
    interrupts();

    motor_i.medida = dir_actual_izq ?  pi : -pi;
    motor_d.medida = dir_actual_der ?  pd : -pd;

    // Control PI
    if (motors_enabled) {
      float T = SAMPLE_MS / 1000.0f;
      stepPI(motor_i, T);
      stepPI(motor_d, T);
      dacWrite(SV_SIGNAL_IZQ, motor_i.dac);
      dacWrite(SV_SIGNAL_DER, motor_d.dac);
    } else {
      motor_i.error = 0.0f;
      motor_d.error = 0.0f;
      motor_i.dac   = 0;
      motor_d.dac   = 0;
    }

    // Telemetría → Raspberry Pi (formato compatible con dashboard)
    Serial.printf("TEL,%ld,%ld,%d,%d,%.1f,%.1f,%d,%d\n",
                  motor_i.medida, motor_d.medida,
                  motor_i.ref,    motor_d.ref,
                  motor_i.error,  motor_d.error,
                  motor_i.dac,    motor_d.dac);
  }
}
