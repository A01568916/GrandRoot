/*
 * Control_PID_Serial.ino
 * ESP32 — Control PI Posicional
 * Robot diferencial controlado por SERIAL (desde Raspberry Pi 5)
 *
 * MIGRACIÓN: Esta versión reemplaza el Access Point WiFi + WebSocket
 * por comunicación SERIAL pura a 115200 baudios. La Raspberry Pi corre
 * grandroot_bridge.py (Flask) que sirve el HTML y traduce las peticiones
 * web en comandos seriales hacia este ESP32.
 *
 * PROTOCOLO (idéntico al de WebSocket, ahora por Serial, una línea por comando):
 *   ← (recibe)  ENABLE,true / ENABLE,false
 *   ← (recibe)  MOVE,vx,vy
 *   ← (recibe)  PARAM,nombre,valor
 *   → (envía)   TEL,pi,pd,ri,rd,ei,ed,daci,dacd   (cada SAMPLE_MS ms)
 *   → (envía)   PARAM_OK,nombre,valor             (eco al aplicar parámetro)
 *
 * IMPORTANTE SOBRE EL SERIAL:
 *   El puerto Serial ahora es el CANAL DE DATOS, no un canal de debug.
 *   Por eso NO se imprimen mensajes de log ([WS], [PARAM], etc.): cualquier
 *   texto que no sea TEL, o PARAM_OK, ensuciaría el flujo que lee la
 *   Raspberry. Si necesitas depurar, usa un prefijo y fíltralo en Python,
 *   o usa Serial2 hacia otro pin.
 *
 * FIXES de control conservados del original:
 *  1. Referencia mínima en giros   2. Anti-windup
 *  3. fabsf solo al DAC final      4. Feedforward zona muerta
 *  5. Reset integral/DAC al soltar flecha (ref=0)
 */

// =====================================================
// FORWARD DECLARATIONS
// =====================================================

struct MotorState;
void stepPI(MotorState &m, float T);

// =====================================================
// PINES
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
// PARÁMETROS (modificables en tiempo real vía PARAM,)
// =====================================================

#define SAMPLE_MS   200

int   PULSOS_MAX      = 22;

// =====================================================
// CINEMÁTICA
// =====================================================

const float R_RUEDA = 0.1397f;
const float L_BASE  = 1.12f;

float VMAX = 4.0f;
float WMAX = 7.4f;

float OMEGA_MAX = VMAX / R_RUEDA;   // Se recalcula si cambia VMAX

// =====================================================
// PI POSICIONAL
// =====================================================

float kp = 6.0f;
float ki = 3.0f;

float INTEGRAL_MAX    = 255.0f / 3.0f;  // Anti-windup — se recalcula si cambia ki
int   REF_MIN_GIRO    = 3;              // Ref. mínima FIX 1
int   DAC_MIN_ARRANQUE = 70;            // Feedforward FIX 4

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
// BUFFER DE LÍNEA SERIAL
// =====================================================

String rxLine = "";

// =====================================================
// RESET DE ESTADO DE UN MOTOR
// =====================================================

void resetMotor(MotorState &m) {
  m.integral = 0.0f;
  m.u        = 0.0f;
  m.error    = 0.0f;
  m.dac      = 0;
}

// =====================================================
// DIRECCIÓN MOTORES
// =====================================================

void setDirIzq(bool adelante) {
  digitalWrite(FR_IZQ, adelante ? HIGH : LOW);
}

void setDirDer(bool adelante) {
  digitalWrite(FR_DER, adelante ? LOW : HIGH);
}

// =====================================================
// CINEMÁTICA DIFERENCIAL
// =====================================================

void cinematica(float vx, float vy, int &ref_izq, int &ref_der) {

  float V = vx * VMAX;
  float w = vy * WMAX;

  float omega_r = V / R_RUEDA + (L_BASE / (2.0f * R_RUEDA)) * w;
  float omega_l = V / R_RUEDA - (L_BASE / (2.0f * R_RUEDA)) * w;

  float scale = max(
    max(fabsf(omega_r), fabsf(omega_l)),
    OMEGA_MAX
  ) / OMEGA_MAX;

  omega_r /= scale;
  omega_l /= scale;

  ref_izq = (int)roundf(constrain(omega_l / OMEGA_MAX, -1.0f, 1.0f) * PULSOS_MAX);
  ref_der = (int)roundf(constrain(omega_r / OMEGA_MAX, -1.0f, 1.0f) * PULSOS_MAX);

  // FIX 1 — Referencia mínima de giro
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

  if (dir_izq != dir_actual_izq) {
    dacWrite(SV_SIGNAL_IZQ, 0);
    resetMotor(motor_i);
  }
  if (dir_der != dir_actual_der) {
    dacWrite(SV_SIGNAL_DER, 0);
    resetMotor(motor_d);
  }

  setDirIzq(dir_izq);
  setDirDer(dir_der);

  dir_actual_izq = dir_izq;
  dir_actual_der = dir_der;

  // FIX 5 — Al soltar la flecha (ref→0) limpiamos estado acumulado
  if (ri == 0) {
    dacWrite(SV_SIGNAL_IZQ, 0);
    resetMotor(motor_i);
  }
  if (rd == 0) {
    dacWrite(SV_SIGNAL_DER, 0);
    resetMotor(motor_d);
  }

  motor_i.ref = ri;
  motor_d.ref = rd;
}

// =====================================================
// ENABLE MOTORES
// =====================================================

void enableMotores(bool on) {

  if (on) {
    pinMode(EN_IZQ, OUTPUT);
    digitalWrite(EN_IZQ, LOW);
    pinMode(EN_DER, OUTPUT);
    digitalWrite(EN_DER, LOW);
  } else {
    dacWrite(SV_SIGNAL_IZQ, 0);
    dacWrite(SV_SIGNAL_DER, 0);
    pinMode(EN_IZQ, INPUT);
    pinMode(EN_DER, INPUT);
    resetMotor(motor_i);
    resetMotor(motor_d);
  }

  motors_enabled = on;
}

// =====================================================
// STEP PI
// =====================================================

void stepPI(MotorState &m, float T) {

  // FIX 5 — Si ref es 0 no hay nada que controlar; salimos limpio
  if (m.ref == 0) {
    resetMotor(m);
    return;
  }

  m.error    = (float)m.ref - (float)m.medida;
  m.integral += m.error * T;
  m.integral  = constrain(m.integral, -INTEGRAL_MAX, INTEGRAL_MAX); // FIX 2

  float u_raw = kp * m.error + ki * m.integral; // FIX 3
  m.u = fabsf(u_raw);
  m.u = constrain(m.u, 0.0f, 255.0f);

  // FIX 4 — Feedforward zona muerta
  m.dac = (int)constrain(m.u + DAC_MIN_ARRANQUE,
                         (float)DAC_MIN_ARRANQUE, 255.0f);
}

// =====================================================
// PROCESAR PARÁMETRO  PARAM,nombre,valor
// =====================================================

void procesarParam(String cmd) {
  // cmd llega sin el prefijo "PARAM,"
  int sep = cmd.indexOf(',');
  if (sep < 0) return;

  String nombre = cmd.substring(0, sep);
  float  valor  = cmd.substring(sep + 1).toFloat();

  nombre.toLowerCase();

  if      (nombre == "pulsos_max")       { PULSOS_MAX       = (int)valor; }
  else if (nombre == "vmax")             { VMAX             = valor;
                                           OMEGA_MAX        = VMAX / R_RUEDA; }
  else if (nombre == "wmax")             { WMAX             = valor; }
  else if (nombre == "kp")               { kp               = valor; }
  else if (nombre == "ki")               { ki               = valor;
                                           INTEGRAL_MAX     = 255.0f / ki; }
  else if (nombre == "ref_min_gir")      { REF_MIN_GIRO     = (int)valor; }
  else if (nombre == "dac_min_arranque") { DAC_MIN_ARRANQUE = (int)valor; }
  else {
    return;  // Nombre desconocido — sin eco (no ensuciar serial)
  }

  // Eco de confirmación al cliente (por Serial)
  Serial.print("PARAM_OK,");
  Serial.print(nombre);
  Serial.print(",");
  Serial.println(String(valor, 4));
}

// =====================================================
// PROCESAR COMANDO
// =====================================================

void procesarComando(String cmd) {

  cmd.trim();
  if (cmd.length() == 0) return;

  // ENABLE,true  /  ENABLE,false
  if (cmd.startsWith("ENABLE,")) {
    bool on = (cmd.substring(7) == "true");
    enableMotores(on);
    Serial.print("ENABLE,");
    Serial.println(on ? "true" : "false");
  }

  // MOVE,vx,vy
  else if (cmd.startsWith("MOVE,")) {
    if (!motors_enabled) return;
    int p1 = cmd.indexOf(',');
    int p2 = cmd.indexOf(',', p1 + 1);
    if (p2 < 0) return;
    float vx = cmd.substring(p1 + 1, p2).toFloat();
    float vy = cmd.substring(p2 + 1).toFloat();
    vx = constrain(vx, -1.0f, 1.0f);
    vy = constrain(vy, -1.0f, 1.0f);
    int ri, rd;
    cinematica(vx, vy, ri, rd);
    aplicarMotores(ri, rd);
  }

  // PARAM,nombre,valor
  else if (cmd.startsWith("PARAM,")) {
    procesarParam(cmd.substring(6));
  }
}

// =====================================================
// LECTURA SERIAL NO BLOQUEANTE (una línea por comando)
// =====================================================

void leerSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      procesarComando(rxLine);
      rxLine = "";
    } else if (c != '\r') {
      rxLine += c;
      if (rxLine.length() > 80) rxLine = "";  // Protección anti-desborde
    }
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  // — Motores —
  pinMode(FR_IZQ, OUTPUT);
  pinMode(FR_DER, OUTPUT);
  setDirIzq(true);
  setDirDer(true);
  dacWrite(SV_SIGNAL_IZQ, 0);
  dacWrite(SV_SIGNAL_DER, 0);
  enableMotores(false);

  // — Encoders —
  attachInterrupt(digitalPinToInterrupt(ENC_IZQ), isr_izq, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_DER), isr_der, RISING);

  rxLine.reserve(96);

  t_prev = millis();
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  // — Atender comandos entrantes por serial —
  leerSerial();

  unsigned long ahora = millis();

  if (ahora - t_prev >= SAMPLE_MS) {

    t_prev = ahora;

    // — Leer encoders —
    noInterrupts();
    long pi = cnt_izq;
    long pd = cnt_der;
    cnt_izq = 0;
    cnt_der = 0;
    interrupts();

    // — Signo lógico —
    motor_i.medida = dir_actual_izq ?  pi : -pi;
    motor_d.medida = dir_actual_der ?  pd : -pd;

    // — Control PI —
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

    // — Telemetría por Serial (canal de datos hacia la Raspberry) —
    char buf[80];
    snprintf(buf, sizeof(buf),
             "TEL,%ld,%ld,%d,%d,%.1f,%.1f,%d,%d",
             motor_i.medida, motor_d.medida,
             motor_i.ref,    motor_d.ref,
             motor_i.error,  motor_d.error,
             motor_i.dac,    motor_d.dac);
    Serial.println(buf);
  }
}
