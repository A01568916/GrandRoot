/*
 * Control_PID_WiFi.ino   [VERSIÓN PARCHEADA]
 * ESP32 — Control PI Posicional + Hub WebSocket para dashboard
 *
 * ─── CAMBIOS RESPECTO A LA VERSIÓN ANTERIOR ───────────────────────────────
 *   [PATCH 1] Buffer RX de Serial2 ampliado a 1024 bytes ANTES del begin().
 *             Causa principal del corte de comunicación con sensores.
 *
 *   [PATCH 2] Serial2.begin(...) ya no reserva GPIO 17 como TX. Pasamos -1
 *             porque no le mandamos nada al ESP de sensores.
 *
 *   [PATCH 3] Encoders configurados como INPUT_PULLUP (ENC_IZQ=18).
 *             Para ENC_DER=34 (input-only sin pull interno) se debe
 *             agregar un RESISTOR FÍSICO de 10kΩ entre GPIO 34 y 3.3V.
 *             Si no tienes el resistor a la mano, define DESACTIVAR_IRQS_DER
 *             en 1 para diagnóstico — desactiva temporalmente la interrupción
 *             del motor derecho para confirmar que el ruido en GPIO 34
 *             era la causa del problema.
 *
 *   [PATCH 4] Diagnóstico en el log: además de SENS_age muestra cuántos
 *             bytes hay en el buffer de Serial2 y cuántas IRQs disparó cada
 *             encoder en el último ciclo. Si ves miles de IRQs/200ms en uno
 *             de los encoders cuando el robot no se mueve → es ruido.
 *
 * ─── ARQUITECTURA ─────────────────────────────────────────────────────────
 *
 *   ESP32 SENSORES (GPS + IMU)
 *           │ Serial1 TX (GPIO 4)    JSON cada 200 ms
 *           ▼
 *   ESP32 PID (este)
 *     · Serial2 RX (GPIO 16)
 *     · WiFi AP "ESP32-Robot"
 *     · WS ws://192.168.4.1/ws
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
// SWITCH DE DIAGNÓSTICO
// =====================================================
// [PATCH 3] Pon en 1 SOLO para probar si el ruido en el encoder derecho
//           (GPIO 34, input-only sin pull-up interno) es lo que bloquea
//           el UART. Con esto desactivado el motor derecho NO contará
//           pulsos. Es solo para diagnóstico. Vuelve a 0 cuando agregues
//           el pull-up físico de 10kΩ entre GPIO 34 y 3.3V.
#define DESACTIVAR_IRQS_DER  0

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
#define ENC_DER        34   // ⚠ input-only, NECESITA pull-up físico 10kΩ→3V3

// =====================================================
// LINK CON ESP32 SENSORES (Serial2)
// =====================================================
// ESP32_SENSORES.GPIO4 (TX1) → ESP32_PID.GPIO16 (RX2)
// GND COMÚN entre ambos ESP32 (¡obligatorio!)
#define LINK_RX_PIN   16
// [PATCH 2] Ya NO definimos LINK_TX_PIN porque no le mandamos nada al ESP
//           de sensores. Pasamos -1 al begin() para liberar GPIO 17.
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
#define LINK_BUF_SIZE   384
char          linkBuf[LINK_BUF_SIZE];
size_t        linkBufLen = 0;
char          ultimoSensJSON[LINK_BUF_SIZE] = "";
unsigned long t_ultimoSens   = 0;

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
void leerLinkSensores() {
  while (Serial2.available() > 0) {
    char c = (char)Serial2.read();

    if (c == '\n') {
      linkBuf[linkBufLen] = '\0';

      if (linkBufLen > 0 && linkBuf[0] == '{') {
        strncpy(ultimoSensJSON, linkBuf, LINK_BUF_SIZE - 1);
        ultimoSensJSON[LINK_BUF_SIZE - 1] = '\0';
        t_ultimoSens = millis();

        if (ws.count() > 0) {
          char wsbuf[LINK_BUF_SIZE + 8];
          int n = snprintf(wsbuf, sizeof(wsbuf), "SENS,%s", linkBuf);
          if (n > 0 && n < (int)sizeof(wsbuf)) {
            ws.textAll(wsbuf, n);
          }
        }
      }

      linkBufLen = 0;
    }
    else if (c != '\r') {
      if (linkBufLen < LINK_BUF_SIZE - 1) {
        linkBuf[linkBufLen++] = c;
      } else {
        linkBufLen = 0;
      }
    }
  }
}

// =====================================================
// SETUP
// =====================================================
void setup() {

  Serial.begin(115200);

  Serial.println("\n============================================");
  Serial.println("  GrandRoot — ESP32 PID [PARCHEADO]");
  Serial.println("============================================");

  // Motores
  pinMode(FR_IZQ, OUTPUT);
  pinMode(FR_DER, OUTPUT);
  setDirIzq(true);
  setDirDer(true);
  dacWrite(SV_SIGNAL_IZQ, 0);
  dacWrite(SV_SIGNAL_DER, 0);
  enableMotores(false);

  // [PATCH 3] Encoders con pull-up donde sea posible.
  //   - ENC_IZQ = 18 → soporta INPUT_PULLUP interno ✅
  //   - ENC_DER = 34 → input-only, NO tiene pull-up interno.
  //     Hay que poner RESISTOR FÍSICO de 10kΩ entre GPIO 34 y 3.3V.
  pinMode(ENC_IZQ, INPUT_PULLUP);
  pinMode(ENC_DER, INPUT);   // pull-up físico obligatorio fuera del MCU

  attachInterrupt(digitalPinToInterrupt(ENC_IZQ), isr_izq, RISING);
#if DESACTIVAR_IRQS_DER
  Serial.println("[ENCODER] ⚠ IRQ derecha DESACTIVADA (modo diagnóstico)");
#else
  attachInterrupt(digitalPinToInterrupt(ENC_DER), isr_der, RISING);
#endif

  // [PATCH 1] Aumentar buffer RX de Serial2 ANTES del begin().
  //           Sin esto, si el loop se atrasa un instante (por WS, por IRQs
  //           del encoder, etc.) se desbordan los 256 bytes default y se
  //           pierden líneas completas del JSON. Causa #1 del corte.
  Serial2.setRxBufferSize(1024);

  // [PATCH 2] Link al ESP32 SENSORES — pasamos -1 como TX porque no le
  //           mandamos nada al sensor. Esto libera GPIO 17.
  Serial2.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, -1);
  Serial.printf("[LINK] Serial2 RX=%d ← ESP32 SENSORES @ %d baud, bufferRX=1024\n",
                LINK_RX_PIN, LINK_BAUD);

  // WiFi AP
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("[WiFi] AP IP: ");
  Serial.println(WiFi.softAPIP());

  // WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // HTTP
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    const size_t total = strlen_P(INDEX_HTML);
    AsyncWebServerResponse *response = request->beginChunkedResponse(
      "text/html",
      [total](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
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

  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(204);
  });

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

    // [PATCH 4] Log de diagnóstico ampliado.
    //   - SENS_age: ms desde el último JSON recibido. Si crece sin parar → no
    //               llegan bytes (cableado, GND, o se desactivó el otro ESP).
    //   - rx_avail: bytes en el buffer de Serial2 ahora mismo. Si oscila
    //               cerca de 1024 → buffer al límite, el loop no está
    //               vaciándolo a tiempo.
    //   - pulse_i / pulse_d: pulsos contados en este ciclo. Si el robot
    //               está QUIETO y ves valores grandes (cientos) en pulse_d
    //               → ruido en GPIO 34, agrega el pull-up.
    Serial.printf("TEL,%ld,%ld,%d,%d,%.1f,%.1f,%d,%d  |  SENS_age=%lums  "
                  "rx_avail=%d  pulse_i=%ld pulse_d=%ld  heap=%u\n",
                  motor_i.medida, motor_d.medida,
                  motor_i.ref,    motor_d.ref,
                  motor_i.error,  motor_d.error,
                  motor_i.dac,    motor_d.dac,
                  ahora - t_ultimoSens,
                  Serial2.available(),
                  pi, pd,
                  ESP.getFreeHeap());
  }
}
