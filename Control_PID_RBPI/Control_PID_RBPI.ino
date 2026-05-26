/*
 * Control_PID_Serial.ino
 * ESP32 — Control PI Posicional
 * Robot diferencial con comunicación Serial USB → Raspberry Pi
 *
 * NO requiere librerías externas.
 *
 * Comunicación Serial a 115200 baud:
 *   → Comandos recibidos desde la RPi:
 *       ENABLE,true / ENABLE,false
 *       MOVE,vx,vy
 *   ← Telemetría enviada a la RPi (cada SAMPLE_MS ms):
 *       TEL,pi,pd,ri,rd,ei,ed,daci,dacd
 *
 * FIXES aplicados:
 *  1. Referencia mínima en giros  → evita alarma del driver
 *  2. Anti-windup correcto        → integral limitada a 255/ki
 *  3. fabsf solo al DAC final     → integral correcta en reversa
 *  4. Feedforward zona muerta     → arranque seguro con ref baja
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
// PARÁMETROS
// =====================================================

#define SAMPLE_MS   200
#define PULSOS_MAX  22

// =====================================================
// CINEMÁTICA
// =====================================================

const float R_RUEDA = 0.1397f;
const float L_BASE  = 1.12f;

const float VMAX = 4.0f;
const float WMAX = 7.4f;

const float OMEGA_MAX = VMAX / R_RUEDA;

// =====================================================
// PI POSICIONAL
// =====================================================

float kp = 6.0f;
float ki = 3.0f;

const float INTEGRAL_MAX     = 255.0f / 3.0f;  // Anti-windup FIX 2
const int   REF_MIN_GIRO     = 3;              // Ref. mínima FIX 1
const int   DAC_MIN_ARRANQUE = 70;              // Feedforward FIX 4

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
// BUFFER SERIAL
// =====================================================

String serial_buf = "";

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
    pinMode(EN_IZQ, OUTPUT);
    digitalWrite(EN_IZQ, LOW);
    pinMode(EN_DER, OUTPUT);
    digitalWrite(EN_DER, LOW);
  } else {
    dacWrite(SV_SIGNAL_IZQ, 0);
    dacWrite(SV_SIGNAL_DER, 0);
    pinMode(EN_IZQ, INPUT);
    pinMode(EN_DER, INPUT);
    motor_i.integral = 0;
    motor_i.u        = 0;
    motor_d.integral = 0;
    motor_d.u        = 0;
  }

  motors_enabled = on;

  // Confirmar por Serial a la RPi
  Serial.println(on ? "ENABLE,true" : "ENABLE,false");
}

// =====================================================
// STEP PI
// =====================================================

void stepPI(MotorState &m, float T) {

  m.error     = (float)m.ref - (float)m.medida;
  m.integral += m.error * T;
  m.integral  = constrain(m.integral, -INTEGRAL_MAX, INTEGRAL_MAX); // FIX 2

  float u_raw = kp * m.error + ki * m.integral; // FIX 3
  m.u = fabsf(u_raw);
  m.u = constrain(m.u, 0.0f, 255.0f);

  // FIX 4 — Feedforward zona muerta
  if (m.ref != 0) {
    m.dac = (int)constrain(m.u + DAC_MIN_ARRANQUE,
                           (float)DAC_MIN_ARRANQUE, 255.0f);
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

  Serial.println("[ESP32] Listo — esperando comandos por Serial");

  t_prev = millis();
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  // — Leer comandos desde Serial (RPi) —
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (serial_buf.length() > 0) {
        procesarComando(serial_buf);
        serial_buf = "";
      }
    } else if (c != '\r') {
      serial_buf += c;
    }
  }

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

    // — Telemetría por Serial a la RPi —
    Serial.printf("TEL,%ld,%ld,%d,%d,%.1f,%.1f,%d,%d\n",
                  motor_i.medida, motor_d.medida,
                  motor_i.ref,    motor_d.ref,
                  motor_i.error,  motor_d.error,
                  motor_i.dac,    motor_d.dac);
  }
}
