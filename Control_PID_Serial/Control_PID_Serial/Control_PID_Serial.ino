/*
 * Control_PI_Posicional_SERIAL.ino
 * ESP32 — Control PI Posicional
 * Robot diferencial con interfaz Serial
 *
 * FIXES aplicados:
 *  1. Referencia mínima en giros  → evita alarma del driver
 *  2. Anti-windup correcto        → integral limitada a 255/ki
 *  3. fabsf solo al DAC final     → integral correcta en reversa
 *  4. Feedforward zona muerta     → arranque seguro con ref baja
 */

// =====================================================
// FORWARD DECLARATIONS
// El preprocesador de Arduino IDE genera prototipos
// automáticos antes de los structs, lo que causa error
// en funciones que reciben structs por referencia.
// Declararlos aquí explícitamente lo evita.
// =====================================================

struct MotorState;
void stepPI(MotorState &m, float T);

// =====================================================
// SERIAL
// =====================================================

String serialBuffer = "";

// =====================================================
// PINES
// =====================================================

#define SV_SIGNAL_IZQ  25
#define SV_SIGNAL_DER  26

#define FR_IZQ         14
#define FR_DER         27

#define EN_IZQ         21
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

const float R_RUEDA = 0.1397f;
const float L_BASE  = 1.12f;

const float VMAX = 4.0f;
const float WMAX = 2.0f;

const float OMEGA_MAX = VMAX / R_RUEDA;

// =====================================================
// PI POSICIONAL
// =====================================================

float kp = 6.0f;
float ki = 3.0f;

// -------------------------------------------------------
// FIX 2 — Anti-windup: limitar integral a DAC_MAX / ki
// Con ki=3 → 255/3 = 85. Así kp*e + ki*integral ≤ 255
// cuando el error ya convergió.
// -------------------------------------------------------
const float INTEGRAL_MAX = 255.0f / 3.0f;  // = 85.0f  ← recalcula si cambias ki

// -------------------------------------------------------
// FIX 1 — Referencia mínima para giros
// Cuando el robot gira en el lugar la cinemática puede
// dar refs de ±6 o menos; con esto garantizamos que el
// driver siempre recibe una señal suficiente para arrancar.
// Ajusta según el umbral real de tu driver.
// -------------------------------------------------------
const int REF_MIN_GIRO = 12;

// -------------------------------------------------------
// FIX 4 — Feedforward de zona muerta
// Offset fijo que se suma al DAC cuando ref ≠ 0,
// compensa el umbral físico del driver sin esperar que
// el integrador lo acumule solo.
// Empieza en 60 y sube/baja de 10 en 10 según tu driver.
// -------------------------------------------------------
const int DAC_MIN_ARRANQUE = 60;

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
// TELEMETRÍA
// =====================================================

struct TeleData {
  long  pul_izq;
  long  pul_der;
  int   ref_i;
  int   ref_d;
  float err_i;
  float err_d;
  int   dac_i;
  int   dac_d;
};

static TeleData last_tele = { 0, 0, 0, 0, 0.0f, 0.0f, 0, 0 };

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
// DIRECCIÓN MOTORES
// =====================================================

void setDirIzq(bool adelante) {
  digitalWrite(FR_IZQ, adelante ? LOW : HIGH);
}

void setDirDer(bool adelante) {
  digitalWrite(FR_DER, adelante ? HIGH : LOW);
}

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

  ref_izq = (int)roundf(
    constrain(omega_l / OMEGA_MAX, -1.0f, 1.0f) * PULSOS_MAX
  );

  ref_der = (int)roundf(
    constrain(omega_r / OMEGA_MAX, -1.0f, 1.0f) * PULSOS_MAX
  );

  // ---------------------------------------------------
  // FIX 1 — Referencia mínima de giro
  // Si la cinemática produce una referencia distinta de
  // cero pero menor que REF_MIN_GIRO, la llevamos al
  // mínimo con el signo correcto. Así evitamos que el
  // driver se alarme por señal insuficiente.
  // ---------------------------------------------------
  if (ref_izq != 0)
    ref_izq = (ref_izq > 0)
      ? max(ref_izq,  REF_MIN_GIRO)
      : min(ref_izq, -REF_MIN_GIRO);

  if (ref_der != 0)
    ref_der = (ref_der > 0)
      ? max(ref_der,  REF_MIN_GIRO)
      : min(ref_der, -REF_MIN_GIRO);
}

// =====================================================
// APLICAR MOTORES
// =====================================================

void aplicarMotores(int ri, int rd) {

  bool dir_izq = (ri >= 0);
  bool dir_der = (rd >= 0);

  // Al cambiar dirección: cortar DAC, resetear estado PI
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
}

// =====================================================
// STEP PI — lógica de control para un motor
// =====================================================
//
// FIX 3: el esfuerzo u_raw conserva su signo durante
// el cálculo. Esto hace que el término integral sea
// correcto cuando el motor opera en reversa (dir=false).
// Solo al final, justo antes de escribir el DAC,
// tomamos fabsf() porque la magnitud la necesita el DAC
// y la dirección ya la manejan los pines FR.
//
// FIX 4: si la referencia es distinta de cero se suma
// DAC_MIN_ARRANQUE como offset de zona muerta.
//
// =====================================================

void stepPI(MotorState &m, float T) {

  m.error = (float)m.ref - (float)m.medida;

  m.integral += m.error * T;

  // FIX 2 — Anti-windup: techo igual a DAC_MAX / ki
  m.integral = constrain(m.integral, -INTEGRAL_MAX, INTEGRAL_MAX);

  // FIX 3 — u_raw con signo correcto; fabsf solo al DAC
  float u_raw = kp * m.error + ki * m.integral;

  // Magnitud del esfuerzo (dirección → pin FR)
  m.u = fabsf(u_raw);
  m.u = constrain(m.u, 0.0f, 255.0f);

  // FIX 4 — Feedforward zona muerta
  if (m.ref != 0) {
    m.dac = (int)constrain(
      m.u + DAC_MIN_ARRANQUE,
      (float)DAC_MIN_ARRANQUE,
      255.0f
    );
  } else {
    m.dac = 0;
  }
}

// =====================================================
// PROCESAR SERIAL
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

    Serial.print("REF,");
    Serial.print(ri);
    Serial.print(",");
    Serial.println(rd);
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

  // Motores
  pinMode(FR_IZQ, OUTPUT);
  pinMode(FR_DER, OUTPUT);

  setDirIzq(true);
  setDirDer(true);

  dacWrite(SV_SIGNAL_IZQ, 0);
  dacWrite(SV_SIGNAL_DER, 0);

  enableMotores(false);

  // Encoders
  attachInterrupt(digitalPinToInterrupt(ENC_IZQ), isr_izq, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_DER), isr_der, RISING);

  t_prev = millis();

  Serial.println("ESP32 READY");
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  leerSerial();

  unsigned long ahora = millis();

  if (ahora - t_prev >= SAMPLE_MS) {

    t_prev = ahora;

    // --------------------------------------------------
    // LEER ENCODERS
    // --------------------------------------------------

    noInterrupts();
    long pi = cnt_izq;
    long pd = cnt_der;
    cnt_izq = 0;
    cnt_der = 0;
    interrupts();

    // --------------------------------------------------
    // SIGNO LÓGICO
    // --------------------------------------------------

    motor_i.medida = dir_actual_izq ?  pi : -pi;
    motor_d.medida = dir_actual_der ?  pd : -pd;

    // --------------------------------------------------
    // CONTROL PI
    // --------------------------------------------------

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

    // --------------------------------------------------
    // TELEMETRÍA
    // --------------------------------------------------

    last_tele = {
      motor_i.medida, motor_d.medida,
      motor_i.ref,    motor_d.ref,
      motor_i.error,  motor_d.error,
      motor_i.dac,    motor_d.dac
    };

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
