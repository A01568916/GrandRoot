/*
 * Control_PID_Vision.ino
 * ESP32 — Control PI Posicional con Vision por Computadora
 *
 * Recibe comandos MOVE desde Python (robot_vision_serial.py) por Serial USB.
 * Protocolo identico al Control_PID_Serial original:
 *   PC  → ESP32 :  ENABLE,true / ENABLE,false
 *   PC  → ESP32 :  MOVE,vx,vy
 *   ESP32 → PC  :  TEL,pi,pd,ri,rd,ei,ed,daci,dacd
 *
 * La velocidad de referencia la controla Python a traves de REF_PULSOS.
 * No hay HTML ni WiFi — solo Serial USB.
 *
 * FIXES heredados del Control_PID_Serial:
 *  1. Referencia minima en giros
 *  2. Anti-windup
 *  3. fabsf solo al DAC final
 *  4. Feedforward zona muerta
 */

// =====================================================
// FORWARD DECLARATIONS
// =====================================================

struct MotorState;
void stepPI(MotorState &m, float T);

// =====================================================
// PINES  (identicos al original)
// =====================================================

#define SV_SIGNAL_IZQ  25
#define SV_SIGNAL_DER  26

#define FR_IZQ         14
#define FR_DER         27

#define EN_IZQ         13
#define EN_DER         33

#define ENC_IZQ        32
#define ENC_DER        34

// =====================================================
// PARÁMETROS
// =====================================================

#define SAMPLE_MS   200
#define PULSOS_MAX  22

// =====================================================
// CINEMÁTICA
// =====================================================

const float R_RUEDA  = 0.1397f;
const float L_BASE   = 1.12f;
const float VMAX     = 4.0f;
const float WMAX     = 2.0f;
const float OMEGA_MAX = VMAX / R_RUEDA;

// =====================================================
// PI POSICIONAL
// =====================================================

float kp = 6.0f;
float ki = 3.0f;

const float INTEGRAL_MAX     = 255.0f / 3.0f;  // Anti-windup: DAC_MAX / ki
const int   REF_MIN_GIRO     = 12;
const int   DAC_MIN_IZQ      = 60;  // Feedforward izquierdo — calibra si arranca desigual
const int   DAC_MIN_DER      = 60;  // Feedforward derecho  — sube el del motor mas lento

// =====================================================
// ENCODERS
// =====================================================

volatile long cnt_izq = 0;
volatile long cnt_der = 0;

void IRAM_ATTR isr_izq() { cnt_izq++; }
void IRAM_ATTR isr_der() { cnt_der++; }

// =====================================================
// ESTADO
// =====================================================

bool dir_actual_izq = true;
bool dir_actual_der = true;

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
String serialBuffer  = "";

// =====================================================
// WATCHDOG — si no llega ningun MOVE en este tiempo
// (ms), se para automaticamente por seguridad.
// Evita que el robot siga moviendose si se cae Python.
// =====================================================

#define WATCHDOG_MS  600
unsigned long t_ultimo_move = 0;

// =====================================================
// DIRECCION MOTORES
// =====================================================

void setDirIzq(bool adelante) { digitalWrite(FR_IZQ, adelante ? LOW  : HIGH); }
void setDirDer(bool adelante) { digitalWrite(FR_DER, adelante ? HIGH : LOW);  }

// =====================================================
// CINEMÁTICA DIFERENCIAL
// =====================================================

void cinematica(float vx, float vy, int &ref_izq, int &ref_der) {

  float V = vx * VMAX;
  float w = -vy * WMAX;

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

  // FIX 1 — Referencia minima de giro
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
    motor_i.integral = 0;
    motor_i.u        = 0;
  }
  if (dir_der != dir_actual_der) {
    dacWrite(SV_SIGNAL_DER, 0);
    motor_d.integral = 0;
    motor_d.u        = 0;
  }

  setDirIzq(dir_izq);
  setDirDer(dir_der);
  dir_actual_izq = dir_izq;
  dir_actual_der = dir_der;

  motor_i.ref = ri;
  motor_d.ref = rd;
}

// =====================================================
// ENABLE MOTORES
// =====================================================

void enableMotores(bool on) {

  if (on) {
    pinMode(EN_IZQ, OUTPUT); digitalWrite(EN_IZQ, LOW);
    pinMode(EN_DER, OUTPUT); digitalWrite(EN_DER, LOW);
  } else {
    dacWrite(SV_SIGNAL_IZQ, 0);
    dacWrite(SV_SIGNAL_DER, 0);
    pinMode(EN_IZQ, INPUT);
    pinMode(EN_DER, INPUT);
    motor_i.integral = 0; motor_i.u = 0;
    motor_d.integral = 0; motor_d.u = 0;
  }
  motors_enabled = on;
}

// =====================================================
// STEP PI  (FIX 2, 3, 4 — DAC_MIN independiente por motor)
// =====================================================

void stepPI(MotorState &m, float T, int dac_min) {

  m.error    = (float)m.ref - (float)m.medida;
  m.integral += m.error * T;
  m.integral  = constrain(m.integral, -INTEGRAL_MAX, INTEGRAL_MAX); // FIX 2

  float u_raw = kp * m.error + ki * m.integral;                     // FIX 3
  m.u = fabsf(u_raw);
  m.u = constrain(m.u, 0.0f, 255.0f);

  // FIX 4 — Feedforward zona muerta con offset por motor
  if (m.ref != 0) {
    m.dac = (int)constrain(m.u + dac_min, (float)dac_min, 255.0f);
  } else {
    m.dac = 0;
  }
}

// =====================================================
// PROCESAR COMANDO SERIAL
// =====================================================

void procesarComando(String cmd) {

  cmd.trim();

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

    // Refrescar watchdog
    t_ultimo_move = millis();
  }
}

void leerSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      procesarComando(serialBuffer);
      serialBuffer = "";
    } else {
      serialBuffer += c;
    }
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  pinMode(FR_IZQ, OUTPUT);
  pinMode(FR_DER, OUTPUT);
  setDirIzq(true);
  setDirDer(true);
  dacWrite(SV_SIGNAL_IZQ, 0);
  dacWrite(SV_SIGNAL_DER, 0);
  enableMotores(false);

  attachInterrupt(digitalPinToInterrupt(ENC_IZQ), isr_izq, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_DER), isr_der, RISING);

  t_prev        = millis();
  t_ultimo_move = millis();

  Serial.println("ESP32 READY");
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  leerSerial();

  // ── Watchdog: si Python deja de enviar MOVE, parar ──────────────────────
  if (motors_enabled &&
      (millis() - t_ultimo_move) > WATCHDOG_MS) {
    motor_i.ref = 0;
    motor_d.ref = 0;
  }

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
      stepPI(motor_i, T, DAC_MIN_IZQ);
      stepPI(motor_d, T, DAC_MIN_DER);
      dacWrite(SV_SIGNAL_IZQ, motor_i.dac);
      dacWrite(SV_SIGNAL_DER, motor_d.dac);
    } else {
      motor_i.error = 0.0f; motor_i.dac = 0;
      motor_d.error = 0.0f; motor_d.dac = 0;
    }

    // Telemetría → Python
    Serial.print("TEL,");
    Serial.print(motor_i.medida);  Serial.print(",");
    Serial.print(motor_d.medida);  Serial.print(",");
    Serial.print(motor_i.ref);     Serial.print(",");
    Serial.print(motor_d.ref);     Serial.print(",");
    Serial.print(motor_i.error,1); Serial.print(",");
    Serial.print(motor_d.error,1); Serial.print(",");
    Serial.print(motor_i.dac);     Serial.print(",");
    Serial.println(motor_d.dac);
  }
}
