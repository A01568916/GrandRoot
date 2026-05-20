/*
 * Control_PID_WiFi.ino
 * ESP32 — Control PI Posicional
 * Robot diferencial con interfaz WiFi (Access Point + WebSocket)
 *
 * Librerías necesarias (instalar en Arduino IDE Library Manager):
 *   - ESPAsyncWebServer  (by lacamera / me-no-dev)
 *   - AsyncTCP           (by dvarrel / me-no-dev)
 *
 * Red WiFi creada: "ESP32-Robot"  password: "robot1234"
 * IP del ESP32:    192.168.4.1
 * WebSocket:       ws://192.168.4.1/ws
 *
 * Comandos WebSocket (idénticos al protocolo Serial original):
 *   → ENABLE,true / ENABLE,false
 *   → MOVE,vx,vy
 *   ← TEL,pi,pd,ri,rd,ei,ed,daci,dacd   (cada SAMPLE_MS ms)
 *
 * FIXES aplicados (heredados del original):
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
// LIBRERÍAS WiFi + WebSocket
// =====================================================

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "index_html.h"   // Página web embebida

// =====================================================
// CONFIGURACIÓN RED
// =====================================================

const char* AP_SSID     = "ESP32-Robot";
const char* AP_PASSWORD = "robot1234";   // Mínimo 8 caracteres

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

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
const float WMAX = 2.0f;

const float OMEGA_MAX = VMAX / R_RUEDA;

// =====================================================
// PI POSICIONAL
// =====================================================

float kp = 6.0f;
float ki = 3.0f;

const float INTEGRAL_MAX    = 255.0f / 3.0f;  // Anti-windup FIX 2
const int   REF_MIN_GIRO    = 10;              // Ref. mínima FIX 1
const int   DAC_MIN_ARRANQUE = 60;             // Feedforward FIX 4

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
// DIRECCIÓN MOTORES
// =====================================================

void setDirIzq(bool adelante) {
  digitalWrite(FR_IZQ, adelante ? LOW : HIGH);
}

void setDirDer(bool adelante) {
  digitalWrite(FR_DER, adelante ? LOW : HIGH);
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
}

// =====================================================
// STEP PI
// =====================================================

void stepPI(MotorState &m, float T) {

  m.error    = (float)m.ref - (float)m.medida;
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
// PROCESAR COMANDO  (idéntico al original)
// =====================================================

void procesarComando(String cmd) {

  cmd.trim();

  // ENABLE,true  /  ENABLE,false
  if (cmd.startsWith("ENABLE,")) {
    bool on = (cmd.substring(7) == "true");
    enableMotores(on);
    // Respuesta de confirmación (broadcast a todos los clientes)
    String resp = "ENABLE,";
    resp += (on ? "true" : "false");
    ws.textAll(resp);
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
// CALLBACK WEBSOCKET
// =====================================================

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {

  if (type == WS_EVT_CONNECT) {
    Serial.printf("[WS] Cliente #%u conectado desde %s\n",
                  client->id(),
                  client->remoteIP().toString().c_str());

  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WS] Cliente #%u desconectado\n", client->id());
    // Si no quedan clientes, apagamos motores por seguridad
    if (ws.count() == 0) {
      enableMotores(false);
      Serial.println("[WS] Sin clientes — motores desactivados");
    }

  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    // Solo procesamos frames de texto completos
    if (info->final && info->index == 0 && info->len == len
        && info->opcode == WS_TEXT) {
      String msg = "";
      for (size_t i = 0; i < len; i++) msg += (char)data[i];
      procesarComando(msg);
    }

  } else if (type == WS_EVT_ERROR) {
    Serial.printf("[WS] Error cliente #%u\n", client->id());
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

  // — WiFi Access Point —
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("[WiFi] AP iniciado — IP: ");
  Serial.println(WiFi.softAPIP());   // Siempre 192.168.4.1

  // — WebSocket —
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // — Servidor HTTP — sirve la página embebida en index_html.h —
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", INDEX_HTML);
  });

  server.begin();
  Serial.println("[HTTP] Servidor listo en http://192.168.4.1");

  t_prev = millis();
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  // Limpieza periódica de clientes WS desconectados
  ws.cleanupClients();

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

    // — Telemetría por WebSocket —
    // Solo enviamos si hay al menos un cliente conectado
    if (ws.count() > 0) {
      char buf[80];
      snprintf(buf, sizeof(buf),
               "TEL,%ld,%ld,%d,%d,%.1f,%.1f,%d,%d",
               motor_i.medida, motor_d.medida,
               motor_i.ref,    motor_d.ref,
               motor_i.error,  motor_d.error,
               motor_i.dac,    motor_d.dac);
      ws.textAll(buf);
    }

    // Debug por Serial (opcional, puedes comentar esta sección)
    Serial.printf("TEL,%ld,%ld,%d,%d,%.1f,%.1f,%d,%d\n",
                  motor_i.medida, motor_d.medida,
                  motor_i.ref,    motor_d.ref,
                  motor_i.error,  motor_d.error,
                  motor_i.dac,    motor_d.dac);
  }
}
