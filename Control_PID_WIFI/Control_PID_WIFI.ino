/*
 * Control_PID_WIFI.ino — ESP32: Control PID Motor Derecho con interfaz web embebida
 *
 * Modo Access Point — sin laptop, el celular/PC se conecta al WiFi del ESP32:
 *   1. Conectarse al WiFi  →  "GrandRoot"  (contraseña: robot1234)
 *   2. Abrir navegador     →  http://192.168.4.1
 *
 * Conexiones:
 *   DAC Der (SV_SIGNAL): GPIO 26
 *   Encoder Der        : GPIO 32  (pull-down externo a GND)
 *   FR  Der            : GPIO 27  (HIGH = adelante — negado físicamente)
 */

#include <WiFi.h>
#include <WebServer.h>
#include "web_page.h"

// ── Access Point ──────────────────────────────────────────────────────────
const char* AP_SSID = "GrandRoot";
const char* AP_PASS = "robot1234";

WebServer server(80);

// ── Pines ─────────────────────────────────────────────────────────────────
#define SV_SIGNAL_DER  26
#define ENC_DER        32
#define FR_DER         27

// ── Parámetros ────────────────────────────────────────────────────────────
#define PULSOS_MAX   100
#define SAMPLE_MS    1000

// ── Variables PID ─────────────────────────────────────────────────────────
volatile long pulsosDer = 0;

long  medidaDer    = 0;
float kp           = 0.8f;
float ki           = 0.8f / 2.0f;
float kd           = (0.8f / 2.0f) / 10.0f;
float referencia   = 0.0f;
float ua           = 0.0f;
float errorDer     = 0.0f;
int   esfuerzoDer  = 0;
float errorant     = 0.0f;
float errorantant  = 0.0f;

bool  motorActivo  = false;

// ── Telemetría (leída por /api/tele) ──────────────────────────────────────
struct TeleData { long pulsos; float error; int esfuerzo; };
static TeleData last_tele = { 0, 0.0f, 0 };

unsigned long tAnterior = 0;

// ── ISR encoder ───────────────────────────────────────────────────────────
void IRAM_ATTR contarPulsoDer() { pulsosDer++; }

// ── Habilitar / deshabilitar motor ────────────────────────────────────────
void enableMotor(bool on) {
  if (!on) {
    dacWrite(SV_SIGNAL_DER, 0);
    ua = 0.0f; errorant = 0.0f; errorantant = 0.0f;
  }
  motorActivo = on;
}

// ── Setup ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // Motor
  pinMode(FR_DER, OUTPUT);
  digitalWrite(FR_DER, HIGH);   // negado: HIGH = adelante real
  dacWrite(SV_SIGNAL_DER, 0);

  // Encoder
  pinMode(ENC_DER, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_DER), contarPulsoDer, RISING);

  // WiFi AP
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP iniciado. IP: ");
  Serial.println(WiFi.softAPIP());

  // ── Rutas HTTP ──────────────────────────────────────────────────────────

  // Página principal
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", HTML);
  });

  // Establecer referencia
  server.on("/api/setref", HTTP_POST, []() {
    String body = server.arg("plain");
    int idx = body.indexOf("\"ref\":");
    if (idx >= 0) {
      float r = body.substring(idx + 6).toFloat();
      referencia = constrain(r, 0.0f, (float)PULSOS_MAX);
    }
    server.send(200, "application/json",
      "{\"ok\":true,\"ref\":" + String(referencia, 1) + "}");
  });

  // Telemetría
  server.on("/api/tele", HTTP_GET, []() {
    String json = "{\"pulsos\":"    + String(last_tele.pulsos)    +
                  ",\"error\":"     + String(last_tele.error, 1)  +
                  ",\"esfuerzo\":"  + String(last_tele.esfuerzo)  +
                  ",\"referencia\":" + String(referencia, 1)      +
                  ",\"activo\":"    + (motorActivo ? "true" : "false") + "}";
    server.send(200, "application/json", json);
  });

  // Habilitar / deshabilitar
  server.on("/api/enable", HTTP_POST, []() {
    String body = server.arg("plain");
    enableMotor(body.indexOf("true") >= 0);
    server.send(200, "application/json",
      String("{\"enabled\":") + (motorActivo ? "true" : "false") + "}");
  });

  server.begin();
  tAnterior = millis();
}

// ── Loop ──────────────────────────────────────────────────────────────────
void loop() {
  server.handleClient();

  if (millis() - tAnterior >= SAMPLE_MS) {
    tAnterior = millis();

    noInterrupts();
    medidaDer = pulsosDer; pulsosDer = 0;
    interrupts();

    if (motorActivo) {
      errorDer    = referencia - (float)medidaDer;
      esfuerzoDer = (int)(kp * errorDer
                        + kp * errorant
                        + ki
                        + kd * errorDer
                        - 2.0f * kd * errorant
                        + kd * errorantant
                        + ua);
      ua          = (float)esfuerzoDer;
      errorantant = errorant;
      errorant    = errorDer;

      if (esfuerzoDer > 255) esfuerzoDer = 255;
      if (esfuerzoDer < 0)   esfuerzoDer = 0;

      dacWrite(SV_SIGNAL_DER, esfuerzoDer);
    } else {
      errorDer    = 0.0f;
      esfuerzoDer = 0;
    }

    last_tele = { medidaDer, errorDer, esfuerzoDer };

    // Monitor serial (opcional)
    Serial.print(medidaDer);    Serial.print(",");
    Serial.print(errorDer, 1);  Serial.print(",");
    Serial.println(esfuerzoDer);
  }
}
