/*
 * Control_PID_WiFi.ino
 * ESP32 — Control PI Posicional + Hub WebSocket para dashboard
 *
 * ─── ARQUITECTURA ─────────────────────────────────────────────────────────
 *
 *   ESP32 SENSORES (GPS + IMU)
 *           │ Serial1 TX (GPIO 4)    JSON cada 200 ms
 *           ▼
 *   ESP32 PID (este)                    ESP32 PID también:
 *     · Serial2 RX (GPIO 16)              · Sirve dashboard HTML
 *     · WiFi AP "ESP32-Robot"             · Acepta comandos MOVE/ENABLE/PARAM
 *     · WS ws://192.168.4.1/ws            · Reenvía telemetría motores + sensores
 *           ▲
 *           │ WebSocket
 *   Dashboard (navegador)
 *
 * ─── MENSAJES ENTRANTES POR WS ────────────────────────────────────────────
 *   → ENABLE,true / ENABLE,false
 *   → MOVE,vx,vy
 *   → PARAM,nombre,valor
 *
 * ─── MENSAJES SALIENTES POR WS ────────────────────────────────────────────
 *   ← TEL,pi,pd,ri,rd,ei,ed,daci,dacd      (motores)
 *   ← SENS,{...json crudo del ESP32 SENSORES...}   (sensores)
 *
 *  Librerías Arduino IDE:
 *    - ESPAsyncWebServer  (lacamera / me-no-dev)
 *    - AsyncTCP           (dvarrel / me-no-dev)
 *
 *  Nota: NO usamos ArduinoJson aquí — solo reenviamos la línea JSON tal cual
 *  llega del ESP32 SENSORES, el dashboard la parsea con JSON.parse().
 *  Esto ahorra RAM y mantiene este firmware más simple.
 */

// =====================================================
// FORWARD DECLARATIONS
// =====================================================
struct MotorState;
void stepPI(MotorState &m, float T);

// =====================================================
// LIBRERÍAS
// =====================================================
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "index_html.h"

// =====================================================
// CONFIG RED
// =====================================================
const char* AP_SSID     = "ESP32-Robot";
const char* AP_PASSWORD = "robot1234";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

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
// LINK CON ESP32 SENSORES (Serial2)
// =====================================================
// ESP32_SENSORES.GPIO4 (TX1) → ESP32_PID.GPIO16 (RX2)
// GND COMÚN entre ambos ESP32 (¡obligatorio!)
#define LINK_RX_PIN   16
#define LINK_TX_PIN   17   // no usado pero hay que declararlo
#define LINK_BAUD     115200

// =====================================================
// PARÁMETROS
// =====================================================
#define SAMPLE_MS   200

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
// BUFFER DEL LINK DE SENSORES
// =====================================================
String        linkBuffer = "";
String        ultimoSensJSON = "";        // último JSON crudo recibido
unsigned long t_ultimoSens   = 0;          // millis() de la última recepción

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
void setDirDer(bool adelante) { digitalWrite(FR_DER, adelante ? LOW : HIGH); }

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

  if (ri == 0) { dacWrite(SV_SIGNAL_IZQ, 0); resetMotor(motor_i); }
  if (rd == 0) { dacWrite(SV_SIGNAL_DER, 0); resetMotor(motor_d); }

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
    resetMotor(motor_i);
    resetMotor(motor_d);
  }
  motors_enabled = on;
}

// =====================================================
// STEP PI
// =====================================================
void stepPI(MotorState &m, float T) {
  if (m.ref == 0) { resetMotor(m); return; }

  m.error    = (float)m.ref - (float)m.medida;
  m.integral += m.error * T;
  m.integral  = constrain(m.integral, -INTEGRAL_MAX, INTEGRAL_MAX);

  float u_raw = kp * m.error + ki * m.integral;
  m.u = fabsf(u_raw);
  m.u = constrain(m.u, 0.0f, 255.0f);

  m.dac = (int)constrain(m.u + DAC_MIN_ARRANQUE,
                         (float)DAC_MIN_ARRANQUE, 255.0f);
}

// =====================================================
// PARÁMETROS  PARAM,nombre,valor
// =====================================================
void procesarParam(String cmd) {
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
    Serial.printf("[PARAM] Desconocido: %s\n", nombre.c_str());
    return;
  }

  Serial.printf("[PARAM] %s = %.4f\n", nombre.c_str(), valor);

  String resp = "PARAM_OK," + nombre + "," + String(valor, 4);
  ws.textAll(resp);
}

// =====================================================
// PROCESAR COMANDO WS
// =====================================================
void procesarComando(String cmd) {
  cmd.trim();

  if (cmd.startsWith("ENABLE,")) {
    bool on = (cmd.substring(7) == "true");
    enableMotores(on);
    String resp = "ENABLE,";
    resp += (on ? "true" : "false");
    ws.textAll(resp);
  }
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
  else if (cmd.startsWith("PARAM,")) {
    procesarParam(cmd.substring(6));
  }
}

// =====================================================
// CALLBACK WEBSOCKET
// =====================================================
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {

  if (type == WS_EVT_CONNECT) {
    Serial.printf("[WS] Cliente #%u conectado desde %s\n",
                  client->id(), client->remoteIP().toString().c_str());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WS] Cliente #%u desconectado\n", client->id());
    if (ws.count() == 0) {
      enableMotores(false);
      Serial.println("[WS] Sin clientes — motores desactivados");
    }
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
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
// LEER LINK SERIAL2 (datos del ESP32 SENSORES)
// =====================================================
// Acumula caracteres hasta encontrar '\n', luego guarda la línea
// como `ultimoSensJSON` y la reenvía por WS prefijada con "SENS,".
//
void leerLinkSensores() {
  while (Serial2.available() > 0) {
    char c = (char)Serial2.read();

    if (c == '\n') {
      if (linkBuffer.length() > 0 && linkBuffer.startsWith("{")) {
        ultimoSensJSON = linkBuffer;
        t_ultimoSens   = millis();

        // Reenviar al dashboard inmediatamente — bajo ritmo, no satura
        if (ws.count() > 0) {
          String msg = "SENS," + ultimoSensJSON;
          ws.textAll(msg);
        }
      }
      linkBuffer = "";
    } else if (c != '\r') {
      linkBuffer += c;
      // Protección contra basura sin '\n'
      if (linkBuffer.length() > 400) linkBuffer = "";
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

  // Link al ESP32 SENSORES
  Serial2.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  Serial.printf("[LINK] Serial2 RX=%d ← ESP32 SENSORES @ %d baud\n",
                LINK_RX_PIN, LINK_BAUD);

  // WiFi AP
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("[WiFi] AP IP: ");
  Serial.println(WiFi.softAPIP());

  // WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // HTTP
  // El HTML embebido pesa ~430 KB con Leaflet y Chart.js dentro,
  // demasiado para send_P() que intentaría tenerlo todo en RAM.
  // Usamos un chunked response que lee directamente de PROGMEM
  // de a pedacitos. Cada cliente lleva su propio índice en `index`.
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    const size_t total = strlen_P(INDEX_HTML);
    AsyncWebServerResponse *response = request->beginChunkedResponse(
      "text/html",
      [total](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
        // index = bytes ya enviados; maxLen = espacio disponible ahora
        if (index >= total) return 0;
        size_t restante = total - index;
        size_t n = (restante < maxLen) ? restante : maxLen;
        memcpy_P(buffer, INDEX_HTML + index, n);
        return n;
      }
    );
    response->addHeader("Cache-Control", "public, max-age=3600");
    request->send(response);
  });

  // Favicon vacío para evitar 404 spammeando la consola del navegador
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(204);
  });

  // Handler para cualquier otra ruta — responde 404 limpio sin spam
  server.onNotFound([](AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not Found");
  });

  server.begin();
  Serial.println("[HTTP] Servidor listo en http://192.168.4.1");

  t_prev = millis();
}

// =====================================================
// LOOP
// =====================================================
void loop() {

  ws.cleanupClients();

  // ── Procesar continuamente el link de sensores ──────────────────────────
  leerLinkSensores();

  unsigned long ahora = millis();

  if (ahora - t_prev >= SAMPLE_MS) {

    t_prev = ahora;

    // ── Leer encoders ──
    noInterrupts();
    long pi = cnt_izq;
    long pd = cnt_der;
    cnt_izq = 0;
    cnt_der = 0;
    interrupts();

    motor_i.medida = dir_actual_izq ?  pi : -pi;
    motor_d.medida = dir_actual_der ?  pd : -pd;

    // ── Control PI ──
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

    // ── Telemetría motores por WS ──
    if (ws.count() > 0) {
      char buf[100];
      snprintf(buf, sizeof(buf),
               "TEL,%ld,%ld,%d,%d,%.1f,%.1f,%d,%d",
               motor_i.medida, motor_d.medida,
               motor_i.ref,    motor_d.ref,
               motor_i.error,  motor_d.error,
               motor_i.dac,    motor_d.dac);
      ws.textAll(buf);
    }

    // Log USB
    Serial.printf("TEL,%ld,%ld,%d,%d,%.1f,%.1f,%d,%d  |  SENS_age=%lums\n",
                  motor_i.medida, motor_d.medida,
                  motor_i.ref,    motor_d.ref,
                  motor_i.error,  motor_d.error,
                  motor_i.dac,    motor_d.dac,
                  ahora - t_ultimoSens);
  }
}
