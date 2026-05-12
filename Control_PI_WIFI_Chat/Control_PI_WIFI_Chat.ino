/*
 * Control_PI_Posicional_WIFI.ino
 * ESP32 — Control PI Posicional
 * Robot diferencial con interfaz web
 */

#include <WiFi.h>
#include <WebServer.h>
#include "web_page.h"

// =====================================================
// ACCESS POINT
// =====================================================

const char* AP_SSID = "GrandRoot";
const char* AP_PASS = "robot1234";

WebServer server(80);

// =====================================================
// PINES
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

const float R_RUEDA = 0.1397f;
const float L_BASE  = 1.12f;

const float VMAX = 4.0f;

// reducir para pruebas
const float WMAX = 2.0f;

const float OMEGA_MAX =
    VMAX / R_RUEDA;

// =====================================================
// PI POSICIONAL
// u = kp*e + ki*integral
// =====================================================

float kp = 6.0f;
float ki = 3.0f;

// =====================================================
// ENCODERS
// =====================================================

volatile long cnt_izq = 0;
volatile long cnt_der = 0;

void IRAM_ATTR isr_izq() {
  cnt_izq++;
}

void IRAM_ATTR isr_der() {
  cnt_der++;
}

// =====================================================
// DIRECCIÓN LÓGICA
// =====================================================

bool dir_actual_izq = true;
bool dir_actual_der = true;

// =====================================================
// TELEMETRÍA
// =====================================================

struct TeleData {
  long pul_izq;
  long pul_der;
  int ref_i;
  int ref_d;
  float err_i;
  float err_d;
  int dac_i;
  int dac_d;
};

static TeleData last_tele = {
  0, 0,
  0, 0,
  0.0f, 0.0f,
  0, 0
};

// =====================================================
// ESTADO MOTOR
// =====================================================

struct MotorState {
  int ref;
  long medida;
  float error;
  float integral;
  float u;
  int dac;
};

MotorState motor_i = {0, 0, 0.0f, 0.0f, 0.0f, 0};
MotorState motor_d = {0, 0, 0.0f, 0.0f, 0.0f, 0};

// =====================================================

bool motors_enabled = false;

unsigned long t_prev = 0;

// =====================================================
// DIRECCIÓN MOTORES
// =====================================================

// IZQ:
// LOW  = adelante
// HIGH = atrás

void setDirIzq(bool adelante) {

  digitalWrite(
    FR_IZQ,
    adelante ? LOW : HIGH
  );
}

// DER:
// HIGH = adelante
// LOW  = atrás

void setDirDer(bool adelante) {

  digitalWrite(
    FR_DER,
    adelante ? HIGH : LOW
  );
}

// =====================================================
// CINEMÁTICA DIFERENCIAL
// =====================================================

void cinematica(float vx, float vy, int &ref_izq, int &ref_der) {
  float V = vx * VMAX;
  float w = -vy * WMAX;

  float omega_r = V / R_RUEDA + (L_BASE / (2.0f * R_RUEDA)) * w;
  float omega_l = V / R_RUEDA - (L_BASE / (2.0f * R_RUEDA)) * w;

  float scale = max(max(fabsf(omega_r), fabsf(omega_l)), OMEGA_MAX) / OMEGA_MAX;

  omega_r /= scale;
  omega_l /= scale;

  ref_izq = (int)roundf(constrain(omega_l / OMEGA_MAX, -1.0f, 1.0f) * PULSOS_MAX);
  ref_der = (int)roundf(constrain(omega_r / OMEGA_MAX, -1.0f, 1.0f) * PULSOS_MAX);
}

// =====================================================
// APLICAR MOTORES
// =====================================================

void aplicarMotores(int ri, int rd) {
  bool dir_izq = (ri >= 0);
  bool dir_der = (rd >= 0);

  // Apagar antes de invertir dirección
  if (dir_izq != dir_actual_izq) {
    dacWrite(SV_SIGNAL_IZQ, 0);
    motor_i.integral = 0;
    motor_i.u = 0;
  }

  if (dir_der != dir_actual_der) {
    dacWrite(SV_SIGNAL_DER, 0);
    motor_d.integral = 0;
    motor_d.u = 0;
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
    motor_i.u = 0;

    motor_d.integral = 0;
    motor_d.u = 0;
  }

  motors_enabled = on;
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);

  // =====================================================
  // MOTORES
  // =====================================================

  pinMode(FR_IZQ, OUTPUT);
  pinMode(FR_DER, OUTPUT);

  setDirIzq(true);
  setDirDer(true);

  dacWrite(SV_SIGNAL_IZQ, 0);
  dacWrite(SV_SIGNAL_DER, 0);

  enableMotores(false);

  // =====================================================
  // ENCODERS
  // =====================================================

  attachInterrupt(digitalPinToInterrupt(ENC_IZQ), isr_izq, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_DER), isr_der, RISING);

  // =====================================================
  // WIFI AP
  // =====================================================

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP iniciado. IP: ");
  Serial.println(WiFi.softAPIP());

  // =====================================================
  // ENDPOINTS HTTP
  // =====================================================

  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", HTML);
  });

  // POST /api/mover - Enviar referencia de velocidad
  server.on("/api/mover", HTTP_POST, []() {
    String body = server.arg("plain");
    float vx = 0.0f;
    float vy = 0.0f;

    int idx = body.indexOf("\"vx\":");
    if (idx >= 0) {
      vx = constrain(body.substring(idx + 5).toFloat(), -1.0f, 1.0f);
    }

    idx = body.indexOf("\"vy\":");
    if (idx >= 0) {
      vy = constrain(body.substring(idx + 5).toFloat(), -1.0f, 1.0f);
    }

    if (motors_enabled) {
      int ri, rd;
      cinematica(vx, vy, ri, rd);
      aplicarMotores(ri, rd);

      String resp = "{\"ok\":true,\"ref_izq\":" + String(ri) + ",\"ref_der\":" + String(rd) + "}";
      server.send(200, "application/json", resp);
    } else {
      server.send(200, "application/json", "{\"ok\":false,\"msg\":\"Motors disabled\"}");
    }
  });

  // GET /api/tele - Obtener telemetría
  server.on("/api/tele", HTTP_GET, []() {
    String motor = server.arg("motor");
    String json;

    if (motor == "derecho") {
      json = "{\"ref\":" + String(last_tele.ref_d) + ",\"pulsos\":" + String(last_tele.pul_der) + ",\"error\":" + String(last_tele.err_d, 1) + ",\"esfuerzo\":" + String(last_tele.dac_d) + "}";
    } else {
      json = "{\"ref\":" + String(last_tele.ref_i) + ",\"pulsos\":" + String(last_tele.pul_izq) + ",\"error\":" + String(last_tele.err_i, 1) + ",\"esfuerzo\":" + String(last_tele.dac_i) + "}";
    }

    server.send(200, "application/json", json);
  });

  // POST /api/enable - Habilitar/deshabilitar motores
  server.on("/api/enable", HTTP_POST, []() {
    String body = server.arg("plain");
    enableMotores(body.indexOf("true") >= 0);
    server.send(200, "application/json", String("{\"enabled\":") + (motors_enabled ? "true" : "false") + "}");
  });

  server.begin();

  t_prev = millis();
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  server.handleClient();

  unsigned long ahora = millis();

  if (ahora - t_prev >= SAMPLE_MS) {
    t_prev = ahora;

    // ==========================================
    // LEER ENCODERS
    // ==========================================

    noInterrupts();
    long pi = cnt_izq;
    long pd = cnt_der;
    cnt_izq = 0;
    cnt_der = 0;
    interrupts();

    // ==========================================
    // SIGNO LÓGICO
    // ==========================================

    motor_i.medida = dir_actual_izq ? pi : -pi;
    motor_d.medida = dir_actual_der ? pd : -pd;

    // ==========================================
    // CONTROL PI
    // ==========================================

    if (motors_enabled) {
      float T = SAMPLE_MS / 1000.0f;

      // ======================================
      // MOTOR IZQUIERDO
      // ======================================

      motor_i.error = (float)motor_i.ref - (float)motor_i.medida;
      motor_i.integral += motor_i.error * T;

      // Antiwindup
      motor_i.integral = constrain(motor_i.integral, -100.0f, 100.0f);

      motor_i.u = kp * motor_i.error + ki * motor_i.integral;
      motor_i.u = fabsf(motor_i.u);
      motor_i.u = constrain(motor_i.u, 0.0f, 255.0f);
      motor_i.dac = (int)motor_i.u;

      dacWrite(SV_SIGNAL_IZQ, motor_i.dac);

      // ======================================
      // MOTOR DERECHO
      // ======================================

      motor_d.error = (float)motor_d.ref - (float)motor_d.medida;
      motor_d.integral += motor_d.error * T;

      // Antiwindup
      motor_d.integral = constrain(motor_d.integral, -100.0f, 100.0f);

      motor_d.u = kp * motor_d.error + ki * motor_d.integral;
      motor_d.u = fabsf(motor_d.u);
      motor_d.u = constrain(motor_d.u, 0.0f, 255.0f);
      motor_d.dac = (int)motor_d.u;

      dacWrite(SV_SIGNAL_DER, motor_d.dac);
    } else {
      motor_i.error = 0.0f;
      motor_d.error = 0.0f;
      motor_i.dac = 0;
      motor_d.dac = 0;
    }

    // ==========================================
    // TELEMETRÍA
    // ==========================================

    last_tele = {
      motor_i.medida, motor_d.medida,
      motor_i.ref, motor_d.ref,
      motor_i.error, motor_d.error,
      motor_i.dac, motor_d.dac
    };

    // ==========================================
    // SERIAL
    // ==========================================

    Serial.print("I:");
    Serial.print(motor_i.medida);
    Serial.print(",");
    Serial.print(motor_i.error, 1);
    Serial.print(",");
    Serial.print(motor_i.dac);

    Serial.print("   D:");
    Serial.print(motor_d.medida);
    Serial.print(",");
    Serial.print(motor_d.error, 1);
    Serial.print(",");
    Serial.println(motor_d.dac);
  }
}