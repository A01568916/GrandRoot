/*
 * Control_PID_WIFI.ino — ESP32: Control PID Dual Motor (derecho + izquierdo) con interfaz web
 *
 * Modo Access Point — sin laptop, el celular/PC se conecta al WiFi del ESP32:
 *   1. Conectarse al WiFi  →  "GrandRoot"  (contraseña: robot1234)
 *   2. Abrir navegador     →  http://192.168.4.1
 *
 * Conexiones (drivers BLDC WS55-220):
 *   SV_SIGNAL_IZQ: GPIO 25  | SV_SIGNAL_DER: GPIO 26
 *   FR_IZQ:        GPIO 14  | FR_DER:        GPIO 27
 *   EN_IZQ:        GPIO 13  | EN_DER:        GPIO 33
 *   ENC_IZQ:       GPIO 32  | ENC_DER:       GPIO 34
 */

#include <WiFi.h>
#include <WebServer.h>
#include "web_page.h"

// ── Access Point ──────────────────────────────────────────────────────────
const char* AP_SSID = "GrandRoot";
const char* AP_PASS = "robot1234";

WebServer server(80);

// ── Pines (drivers BLDC WS55-220) ───────────────────────────────────────────
#define SV_SIGNAL_IZQ  25   // DAC → velocidad motor izquierdo
#define SV_SIGNAL_DER  26   // DAC → velocidad motor derecho
#define FR_IZQ         14   // LOW = adelante, HIGH = atrás
#define FR_DER         27   // HIGH = adelante, LOW = atrás  (lógica invertida)
#define EN_IZQ         13   // OUTPUT+LOW = activo | INPUT = desactivado
#define EN_DER         33   // idem
#define ENC_IZQ        32
#define ENC_DER        34

// ── Parámetros de control ─────────────────────────────────────────────────
#define PULSOS_MAX   22
#define SAMPLE_MS    200

// ── Cinemática — parámetros físicos del robot ─────────────────────────────
const float R_RUEDA = 0.127f;              // radio rueda [m]
const float L_BASE  = 1.12f;              // distancia entre centros de ruedas [m]
const float VMAX    = 3.0f;               // velocidad lineal máxima [m/s]
const float WMAX    = 2.0f * VMAX / L_BASE; // velocidad angular máxima [rad/s]
const float OMEGA_MAX = VMAX / R_RUEDA;   // [rad/s]

// ── PID parámetros (igual para ambos motores) ─────────────────────────────
float kp           = 0.8f;
float ki           = 0.8f / 2.0f;
float kd           = (0.8f / 2.0f) / 10.0f;

// ── ISR encoders ──────────────────────────────────────────────────────────
volatile long cnt_izq = 0, cnt_der = 0;
void IRAM_ATTR isr_izq() { cnt_izq++; }
void IRAM_ATTR isr_der() { cnt_der++; }

// ── Telemetría (leída por /api/tele) ──────────────────────────────────────
struct TeleData { long pul_izq, pul_der; float err_i, err_d; int dac_i, dac_d; };
static TeleData last_tele = {0, 0, 0.0f, 0.0f, 0, 0};

// ── Estado de cada motor (PID + control) ──────────────────────────────────
struct MotorState {
  int ref;           // referencia (pulsos/s)
  long medida;       // pulsos medidos en este ciclo
  float error;       // referencia - medida
  float ua;          // acumulado control
  float errorant;    // error anterior
  float errorantant; // error anteanterior
  int dac;           // valor DAC (0-255)
};

MotorState motor_i = {0, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0};
MotorState motor_d = {0, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0};

bool motors_enabled = false;
int dac_izq = 0, dac_der = 0;
bool dir_prev_izq = true, dir_prev_der = true;

unsigned long t_prev = 0;

// ── Helpers dirección WS55-220 ───────────────────────────────────────────────
// FR_IZQ: LOW = adelante, HIGH = atrás
// FR_DER: HIGH = adelante, LOW = atrás  (driver derecho tiene lógica invertida)
void setDirIzq(bool adelante) { digitalWrite(FR_IZQ, adelante ? LOW  : HIGH); }
void setDirDer(bool adelante) { digitalWrite(FR_DER, adelante ? HIGH : LOW);  }

// ── Cinemática inversa diferencial ────────────────────────────────────────
void cinematica(float vx, float vy, int &ref_izq, int &ref_der) {
  float V = vx * VMAX;
  float w = -vy * WMAX;  // negado: derecha(+vy) → giro derecha
  float omega_r = V / R_RUEDA + (L_BASE / (2.0f * R_RUEDA)) * w;
  float omega_l = V / R_RUEDA - (L_BASE / (2.0f * R_RUEDA)) * w;
  float scale = max(max(fabsf(omega_r), fabsf(omega_l)), OMEGA_MAX) / OMEGA_MAX;
  omega_r /= scale;
  omega_l /= scale;
  ref_izq = (int)roundf(constrain(omega_l / OMEGA_MAX, -1.0f, 1.0f) * PULSOS_MAX);
  ref_der = (int)roundf(constrain(omega_r / OMEGA_MAX, -1.0f, 1.0f) * PULSOS_MAX);
}

// ── Aplicar referencias a motores ─────────────────────────────────────────
void aplicarMotores(int ri, int rd) {
  bool dir_izq = (ri >= 0);
  bool dir_der = (rd >= 0);

  // Si cambia dirección: apagar DAC antes de invertir FR
  if (dir_izq != dir_prev_izq) { dacWrite(SV_SIGNAL_IZQ, 0); dir_prev_izq = dir_izq; }
  if (dir_der != dir_prev_der) { dacWrite(SV_SIGNAL_DER, 0); dir_prev_der = dir_der; }

  setDirIzq(dir_izq);
  setDirDer(dir_der);

  motor_i.ref = ri;
  motor_d.ref = rd;
}

// ── Habilitar / deshabilitar ambos motores a la vez ───────────────────────
void enableMotores(bool on) {
  if (on) {
    pinMode(EN_IZQ, OUTPUT); digitalWrite(EN_IZQ, LOW);
    pinMode(EN_DER, OUTPUT); digitalWrite(EN_DER, LOW);
  } else {
    dacWrite(SV_SIGNAL_IZQ, 0); dac_izq = 0;
    dacWrite(SV_SIGNAL_DER, 0); dac_der = 0;
    pinMode(EN_IZQ, INPUT);   // alta impedancia → driver se desactiva
    pinMode(EN_DER, INPUT);
    // Resetear estados PID
    motor_i.ua = 0.0f; motor_i.errorant = 0.0f; motor_i.errorantant = 0.0f;
    motor_d.ua = 0.0f; motor_d.errorant = 0.0f; motor_d.errorantant = 0.0f;
  }
  motors_enabled = on;
}

// ── Setup ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // Pines motores
  pinMode(FR_IZQ, OUTPUT);
  pinMode(FR_DER, OUTPUT);
  setDirIzq(true);  // adelante por defecto
  setDirDer(true);
  dacWrite(SV_SIGNAL_IZQ, 0);
  dacWrite(SV_SIGNAL_DER, 0);
  enableMotores(false);  // desactivado por defecto

  // Encoders
  attachInterrupt(digitalPinToInterrupt(ENC_IZQ), isr_izq, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_DER), isr_der, RISING);

  // WiFi AP
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP iniciado. IP: ");
  Serial.println(WiFi.softAPIP());

  // Página principal
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", HTML);
  });

  // Ruta /api/mover → recibe {vx, vy}, aplica cinemática, mueve motores
  server.on("/api/mover", HTTP_POST, []() {
    String body = server.arg("plain");
    float vx = 0.0f, vy = 0.0f;
    int idx;
    idx = body.indexOf("\"vx\":");
    if (idx >= 0) vx = constrain(body.substring(idx + 5).toFloat(), -1.0f, 1.0f);
    idx = body.indexOf("\"vy\":");
    if (idx >= 0) vy = constrain(body.substring(idx + 5).toFloat(), -1.0f, 1.0f);
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

  // Ruta /api/tele → telemetría individual por motor
  server.on("/api/tele", HTTP_GET, []() {
    String motor = server.arg("motor");
    String json;
    if (motor == "derecho") {
      json = "{\"pulsos\":" + String(last_tele.pul_der) +
             ",\"error\":" + String(last_tele.err_d, 1) +
             ",\"esfuerzo\":" + String(last_tele.dac_d) + "}";
    } else {  // izquierdo
      json = "{\"pulsos\":" + String(last_tele.pul_izq) +
             ",\"error\":" + String(last_tele.err_i, 1) +
             ",\"esfuerzo\":" + String(last_tele.dac_i) + "}";
    }
    server.send(200, "application/json", json);
  });

  // Ruta /api/enable → activa o desactiva ambos motores
  server.on("/api/enable", HTTP_POST, []() {
    String body = server.arg("plain");
    enableMotores(body.indexOf("true") >= 0);
    server.send(200, "application/json",
      String("{\"enabled\":") + (motors_enabled ? "true" : "false") + "}");
  });

  server.begin();
  t_prev = millis();
}

// ── Loop ──────────────────────────────────────────────────────────────────
void loop() {
  server.handleClient();

  unsigned long ahora = millis();
  if (ahora - t_prev >= SAMPLE_MS) {
    t_prev = ahora;

    noInterrupts();
    long pi = cnt_izq; long pd = cnt_der;
    cnt_izq = 0; cnt_der = 0;
    interrupts();

    motor_i.medida = pi;
    motor_d.medida = pd;

    if (motors_enabled) {
      // ── PID Motor Izquierdo ────────────────────────────────────────────
      motor_i.error = (float)motor_i.ref - (float)motor_i.medida;
      int esfuerzo_i = (int)(kp * motor_i.error
                           + kp * motor_i.errorant
                           + ki
                           + kd * motor_i.error
                           - 2.0f * kd * motor_i.errorant
                           + kd * motor_i.errorantant
                           + motor_i.ua);
      motor_i.ua = (float)esfuerzo_i;
      motor_i.errorantant = motor_i.errorant;
      motor_i.errorant = motor_i.error;

      if (esfuerzo_i > 255) esfuerzo_i = 255;
      if (esfuerzo_i < 0)   esfuerzo_i = 0;

      motor_i.dac = esfuerzo_i;
      dac_izq = esfuerzo_i;
      dacWrite(SV_SIGNAL_IZQ, esfuerzo_i);

      // ── PID Motor Derecho ──────────────────────────────────────────────
      motor_d.error = (float)motor_d.ref - (float)motor_d.medida;
      int esfuerzo_d = (int)(kp * motor_d.error
                           + kp * motor_d.errorant
                           + ki
                           + kd * motor_d.error
                           - 2.0f * kd * motor_d.errorant
                           + kd * motor_d.errorantant
                           + motor_d.ua);
      motor_d.ua = (float)esfuerzo_d;
      motor_d.errorantant = motor_d.errorant;
      motor_d.errorant = motor_d.error;

      if (esfuerzo_d > 255) esfuerzo_d = 255;
      if (esfuerzo_d < 0)   esfuerzo_d = 0;

      motor_d.dac = esfuerzo_d;
      dac_der = esfuerzo_d;
      dacWrite(SV_SIGNAL_DER, esfuerzo_d);

    } else {
      motor_i.error = 0.0f;
      motor_d.error = 0.0f;
      motor_i.dac = 0;
      motor_d.dac = 0;
    }

    // Actualizar telemetría (el navegador la lee con /api/tele)
    last_tele = { pi, pd, motor_i.error, motor_d.error, motor_i.dac, motor_d.dac };

    // Monitor serial (opcional)
    Serial.print("I:");  Serial.print(motor_i.medida); Serial.print(",");
    Serial.print(motor_i.error, 1); Serial.print(",");
    Serial.print(motor_i.dac);      Serial.print("  ");
    Serial.print("D:");  Serial.print(motor_d.medida); Serial.print(",");
    Serial.print(motor_d.error, 1); Serial.print(",");
    Serial.println(motor_d.dac);
  }
}
