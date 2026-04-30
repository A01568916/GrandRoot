// control_remoto_AP.ino — ESP32: servidor web EMBEBIDO en modo Access Point
//                          Sin laptop. El celular se conecta al WiFi del ESP32.
//
// Conexión desde celular:
//   1. Conectarse al WiFi  →  "GrandRoot"  (contraseña: robot1234)
//   2. Abrir navegador     →  http://192.168.4.1
//
// Conexiones Puente H (idénticas a control_remoto_puenteH):
//   ENA GPIO 25 | ENB GPIO 26
//   IN1 GPIO 27 | IN2 GPIO 18
//   IN3 GPIO 19 | IN4 GPIO 14
//   ENC_IZQ GPIO 32 | ENC_DER GPIO 34

// ┌─────────────────────────────────────────────────────────────────────────┐
// │  BLOQUE NUEVO — Librerías WiFi y Servidor Web Asíncrono                │
// │                                                                         │
// │  Instalar desde Library Manager de Arduino IDE:                        │
// │    • "ESP Async WebServer"  por lacamera  (o me-no-dev en GitHub)      │
// │    • "AsyncTCP"             por dvarrel   (o me-no-dev en GitHub)      │
// └─────────────────────────────────────────────────────────────────────────┘
#include <WiFi.h>
#include <WebServer.h>     // biblioteca integrada — sin problemas de threading
#include "web_page.h"   // HTML embebido

// ── Configuración del Access Point ───────────────────────────────────────
const char* AP_SSID = "GrandRoot";
const char* AP_PASS = "robot1234";  // mín. 8 caracteres

WebServer server(80);

// ── Pines (mismos que control_remoto_puenteH) ─────────────────────────────
#define ENA      25   // DAC → velocidad motor izquierdo
#define ENB      26   // DAC → velocidad motor derecho
#define IN1_IZQ  27   // Dirección izq (+)
#define IN2_IZQ  18   // Dirección izq (−) complemento
#define IN3_DER  19   // Dirección der (+)
#define IN4_DER  14   // Dirección der (−) complemento
#define ENC_IZQ  32
#define ENC_DER  34

// ── Parámetros de control ─────────────────────────────────────────────────
#define PULSOS_MAX  98
#define SAMPLE_MS   1000

// ── Cinemática — parámetros físicos del robot ─────────────────────────────
const float R_RUEDA = 0.127f;              // radio rueda [m]
const float L_BASE  = 1.12f;              // distancia entre centros de ruedas [m]
const float VMAX    = 3.0f;               // velocidad lineal máxima [m/s]
const float WMAX    = 2.0f * VMAX / L_BASE; // velocidad angular máxima [rad/s]
const float OMEGA_MAX = VMAX / R_RUEDA;   // [rad/s]

// ── ISR encoders ──────────────────────────────────────────────────────────
volatile long cnt_izq = 0, cnt_der = 0;
void IRAM_ATTR isr_izq() { cnt_izq++; }
void IRAM_ATTR isr_der() { cnt_der++; }

// ── Última telemetría (leída por /api/tele) ───────────────────────────────
struct TeleData { long pul_izq, pul_der; int dac_i, dac_d; };
static TeleData last_tele = {0, 0, 0, 0};

// ── Estado motores ────────────────────────────────────────────────────────
int dac_izq = 0, dac_der = 0;
bool dir_prev_izq = true, dir_prev_der = true;

// ── Helpers dirección puente H ────────────────────────────────────────────
void setDirIzq(bool adelante) {
  digitalWrite(IN1_IZQ, adelante ? HIGH : LOW);
  digitalWrite(IN2_IZQ, adelante ? LOW  : HIGH);
}
void setDirDer(bool adelante) {
  digitalWrite(IN3_DER, adelante ? HIGH : LOW);
  digitalWrite(IN4_DER, adelante ? LOW  : HIGH);
}
void frenarIzq() { digitalWrite(IN1_IZQ, LOW); digitalWrite(IN2_IZQ, LOW); dacWrite(ENA, 0); }
void frenarDer() { digitalWrite(IN3_DER, LOW); digitalWrite(IN4_DER, LOW); dacWrite(ENB, 0); }

// ── Cinemática inversa diferencial ────────────────────────────────────────
void cinematica(float vx, float vy, int &ref_izq, int &ref_der) {
  float V = vx * VMAX;
  float w = -vy * WMAX;  // negado: joystick der(+vy) → giro derecha
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
  if (dir_izq != dir_prev_izq) { frenarIzq(); dir_prev_izq = dir_izq; }
  if (dir_der != dir_prev_der) { frenarDer(); dir_prev_der = dir_der; }

  if (ri == 0) { frenarIzq(); dac_izq = 0; }
  else { setDirIzq(ri > 0); dac_izq = map(abs(ri), 0, PULSOS_MAX, 0, 255); dacWrite(ENA, dac_izq); }

  if (rd == 0) { frenarDer(); dac_der = 0; }
  else { setDirDer(rd > 0); dac_der = map(abs(rd), 0, PULSOS_MAX, 0, 255); dacWrite(ENB, dac_der); }
}

// HTML → definido en web_page.h (separado para evitar que el preprocesador
//         de Arduino IDE rompa el raw string literal R"HTML(...)HTML")

// ── Setup ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // Pines motores
  pinMode(IN1_IZQ, OUTPUT); pinMode(IN2_IZQ, OUTPUT);
  pinMode(IN3_DER, OUTPUT); pinMode(IN4_DER, OUTPUT);
  frenarIzq(); frenarDer();

  // Encoders
  attachInterrupt(digitalPinToInterrupt(ENC_IZQ), isr_izq, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_DER), isr_der, RISING);

  // ┌───────────────────────────────────────────────────────────────────────┐
  // │  BLOQUE NUEVO — Inicio WiFi AP y rutas del servidor web              │
  // └───────────────────────────────────────────────────────────────────────┘
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP iniciado. IP: "); Serial.println(WiFi.softAPIP());

  // Ruta principal → sirve el HTML
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
    int ri, rd;
    cinematica(vx, vy, ri, rd);
    aplicarMotores(ri, rd);
    String resp = "{\"ok\":true,\"ref_izq\":" + String(ri) + ",\"ref_der\":" + String(rd) + "}";
    server.send(200, "application/json", resp);
  });

  // Ruta /api/tele → el navegador la consulta cada segundo (polling)
  server.on("/api/tele", HTTP_GET, []() {
    String json = "{\"pul_izq\":" + String(last_tele.pul_izq) +
                  ",\"dac_izq\":" + String(last_tele.dac_i)   +
                  ",\"pul_der\":" + String(last_tele.pul_der) +
                  ",\"dac_der\":" + String(last_tele.dac_d)   + "}";
    server.send(200, "application/json", json);
  });

  server.begin();
  // └───────────────────────────────────────────────────────────────────────┘
}

// ── Loop ──────────────────────────────────────────────────────────────────
unsigned long t_prev = 0;

void loop() {
  server.handleClient();  // atender peticiones HTTP

  unsigned long ahora = millis();
  if (ahora - t_prev >= SAMPLE_MS) {
    t_prev = ahora;

    noInterrupts();
    long pi = cnt_izq; long pd = cnt_der;
    cnt_izq = 0; cnt_der = 0;
    interrupts();

    // Actualizar telemetría (el navegador la lee con /api/tele)
    last_tele = { pi, pd, dac_izq, dac_der };
  }
}
