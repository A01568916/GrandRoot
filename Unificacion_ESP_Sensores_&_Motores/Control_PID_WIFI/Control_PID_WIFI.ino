/*
 * Control_PID_WIFI_espnow.ino  [FreeRTOS + ESP-NOW]
 * ESP32 — Control PI Posicional + Hub WebSocket para dashboard
 *
 * ─── CAMBIOS RESPECTO A LA VERSIÓN UART ────────────────────────────────────
 *
 *  • UART (Serial2 / GPIO 16) eliminado completamente.
 *    Ya NO hay cable entre los dos ESP32.
 *
 *  • ESP-NOW recibe los datos del ESP32 SENSORES en un callback de interrupción.
 *    El callback copia la struct a un buffer y notifica a la tarea receptora
 *    mediante una FreeRTOS Queue — sin bloquear el loop de control PI.
 *
 *  • Control PI + WebSocket corren en Core 1 (loop de Arduino), igual que antes.
 *
 *  • Tarea FreeRTOS en Core 0 toma los paquetes de la Queue, construye el JSON
 *    y lo reenvía por WebSocket. Así el WebSocket textAll() no bloquea el PI.
 *
 * ─── PRIMERO QUE HACER ──────────────────────────────────────────────────────
 *
 *  1. Enciende el ESP32 SENSORES y copia su MAC del Monitor Serie.
 *     Línea:  [ESP-NOW] Mi MAC: XX:XX:XX:XX:XX:XX
 *  2. Pega esa MAC abajo como PEER_MAC.
 *  3. Sube este sketch al ESP32 PID.
 *
 * ─── ARQUITECTURA ──────────────────────────────────────────────────────────
 *
 *   ESP32 SENSORES
 *     tareaIMU   (Core 0) ─┐
 *     tareaGPS   (Core 1) ─┴─► esp_now_send() ──► aire ──►
 *
 *   ESP32 PID (este)
 *     onRxESPNOW() ISR ──► Queue ──► tareaWS (Core 0) ──► ws.textAll()
 *     loop()            (Core 1) ──► PI + encoders + ws telemetría
 *
 *  Librerías (ya en el core ESP32 de Arduino, no hay que instalar nada extra):
 *    ESPAsyncWebServer  (lacamera / me-no-dev)
 *    AsyncTCP           (dvarrel / me-no-dev)
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
#include <esp_now.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "index_html.h"

// =====================================================
// MAC DEL ESP32 SENSORES
// =====================================================
// Reemplaza con la MAC que muestra el Monitor Serie del ESP32 SENSORES.
// Formato: { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF }
//30:76:F5:E7:47:F8
uint8_t PEER_MAC[6] = { 0x30, 0x76, 0xF5, 0xE7, 0x47, 0xF8 };

// =====================================================
// CONFIG RED
// =====================================================
const char* AP_SSID     = "ESP32-Robot";
const char* AP_PASSWORD = "robot1234";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// =====================================================
// ESTRUCTURA DE DATOS ESP-NOW
// Debe ser idéntica a la del ESP32 SENSORES
// =====================================================
struct __attribute__((packed)) PaqueteSensores {
  double   lat;
  double   lon;
  float    x;
  float    y;
  uint8_t  sats;
  float    ax, ay, az;
  float    gx, gy, gz;
  uint8_t  gps_ok;
  uint8_t  imu_ok;
  uint32_t ts;
};

// =====================================================
// QUEUE ESP-NOW → TAREA WS
// =====================================================
// La ISR onRxESPNOW copia el paquete aquí.
// tareaWS lo consume y arma el JSON para WebSocket.
// Capacidad 4 elementos — si el WebSocket se atrasa, no pierde datos recientes.
#define QUEUE_LEN  4
QueueHandle_t xColaESPNOW;

// Último JSON de sensores (para el diagnóstico del log)
char          ultimoSensJSON[320] = "";
unsigned long t_ultimoSens        = 0;

// =====================================================
// SWITCH DE DIAGNÓSTICO (encoders)
// =====================================================
#define DESACTIVAR_IRQS_DER  0

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
void setDirDer(bool adelante) { digitalWrite(FR_DER, adelante ? LOW  : HIGH); }

// =====================================================
// CINEMÁTICA DIFERENCIAL
// =====================================================
void cinematica(float vx, float vy, int &ref_izq, int &ref_der) {
  float V = vx * VMAX;
  float w = vy * WMAX;
  float omega_r = V / R_RUEDA + (L_BASE / (2.0f * R_RUEDA)) * w;
  float omega_l = V / R_RUEDA - (L_BASE / (2.0f * R_RUEDA)) * w;
  float scale = max(max(fabsf(omega_r), fabsf(omega_l)), OMEGA_MAX) / OMEGA_MAX;
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
  if (dir_izq != dir_actual_izq) { dacWrite(SV_SIGNAL_IZQ, 0); resetMotor(motor_i); }
  if (dir_der != dir_actual_der) { dacWrite(SV_SIGNAL_DER, 0); resetMotor(motor_d); }
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
  m.dac = (int)constrain(m.u + DAC_MIN_ARRANQUE, (float)DAC_MIN_ARRANQUE, 255.0f);
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
  else if (nombre == "vmax")             { VMAX = valor; OMEGA_MAX = VMAX / R_RUEDA; }
  else if (nombre == "wmax")             { WMAX             = valor; }
  else if (nombre == "kp")               { kp               = valor; }
  else if (nombre == "ki")               { ki = valor; INTEGRAL_MAX = 255.0f / ki; }
  else if (nombre == "ref_min_gir")      { REF_MIN_GIRO     = (int)valor; }
  else if (nombre == "dac_min_arranque") { DAC_MIN_ARRANQUE = (int)valor; }
  else { Serial.printf("[PARAM] Desconocido: %s\n", nombre.c_str()); return; }
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
    ws.textAll(String("ENABLE,") + (on ? "true" : "false"));
  }
  else if (cmd.startsWith("MOVE,")) {
    if (!motors_enabled) return;
    int p1 = cmd.indexOf(',');
    int p2 = cmd.indexOf(',', p1 + 1);
    if (p2 < 0) return;
    float vx = constrain(cmd.substring(p1 + 1, p2).toFloat(), -1.0f, 1.0f);
    float vy = constrain(cmd.substring(p2 + 1).toFloat(),      -1.0f, 1.0f);
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
    Serial.printf("[WS] Cliente #%u conectado\n", client->id());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WS] Cliente #%u desconectado\n", client->id());
    if (ws.count() == 0) { enableMotores(false); }
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len
        && info->opcode == WS_TEXT) {
      String msg = "";
      for (size_t i = 0; i < len; i++) msg += (char)data[i];
      procesarComando(msg);
    }
  }
}

// =====================================================
// CALLBACK ESP-NOW RX  (se ejecuta en contexto de interrupción Wi-Fi)
// =====================================================
// ¡IMPORTANTE! Esta función corre en una ISR del driver Wi-Fi.
// No se puede llamar a Serial, ws, malloc, ni ninguna función que bloquee.
// Solo copiamos el paquete a la Queue y regresamos inmediatamente.
//
// En Arduino ESP32 core 3.x la firma del callback RX cambió:
//   core 2.x:  (const uint8_t* mac, const uint8_t* data, int len)
//   core 3.x:  (const esp_now_recv_info_t* info, const uint8_t* data, int len)
//
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void IRAM_ATTR onRxESPNOW(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
#else
void IRAM_ATTR onRxESPNOW(const uint8_t* mac, const uint8_t* data, int len) {
#endif
  if (len != sizeof(PaqueteSensores)) return;  // tamaño inesperado: ignorar

  PaqueteSensores pkt;
  memcpy(&pkt, data, sizeof(pkt));

  // xQueueSendFromISR no bloquea — si la cola está llena descarta el paquete más
  // antiguo (pdFALSE = no hacer reschedule desde la ISR; lo maneja FreeRTOS solo).
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(xColaESPNOW, &pkt, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

// =====================================================
// TAREA FREERTOS — WEBSOCKET DE SENSORES (Core 0)
// =====================================================
// Espera paquetes en la Queue y los reenvía como JSON por WebSocket.
// Al correr en Core 0, no compite con el loop PI de Core 1.
//
void tareaWS_Sensores(void* pvParameters) {
  PaqueteSensores pkt;

  for (;;) {
    // Espera bloqueante: duerme hasta que llegue un paquete (o 500 ms máximo)
    if (xQueueReceive(xColaESPNOW, &pkt, pdMS_TO_TICKS(500)) == pdTRUE) {

      t_ultimoSens = millis();

      if (ws.count() > 0) {
        // Armar JSON idéntico al que mandaba la versión UART
        char json[320];
        int n = snprintf(json, sizeof(json),
          "SENS,{\"lat\":%.10f,\"lon\":%.10f,\"x\":%.3f,\"y\":%.3f,"
          "\"sats\":%u,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
          "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
          "\"gps_ok\":%d,\"imu_ok\":%d,\"ts\":%lu}",
          pkt.lat, pkt.lon, pkt.x, pkt.y,
          pkt.sats,
          pkt.ax, pkt.ay, pkt.az,
          pkt.gx, pkt.gy, pkt.gz,
          pkt.gps_ok, pkt.imu_ok,
          (unsigned long)pkt.ts
        );
        if (n > 0 && n < (int)sizeof(json)) {
          strncpy(ultimoSensJSON, json, sizeof(ultimoSensJSON) - 1);
          ws.textAll(json, n);
        }
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
  Serial.println("  GrandRoot — ESP32 PID [FreeRTOS+ESP-NOW]");
  Serial.println("============================================");

  // Motores
  pinMode(FR_IZQ, OUTPUT);  pinMode(FR_DER, OUTPUT);
  setDirIzq(true);          setDirDer(true);
  dacWrite(SV_SIGNAL_IZQ, 0);
  dacWrite(SV_SIGNAL_DER, 0);
  enableMotores(false);

  // Encoders
  pinMode(ENC_IZQ, INPUT_PULLUP);
  pinMode(ENC_DER, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_IZQ), isr_izq, RISING);
#if !DESACTIVAR_IRQS_DER
  attachInterrupt(digitalPinToInterrupt(ENC_DER), isr_der, RISING);
#endif

  // ── Wi-Fi AP ─────────────────────────────────────────────────────────
  // AP y ESP-NOW coexisten en el mismo canal.
  // ESP-NOW usará automáticamente el canal del AP.
  WiFi.mode(WIFI_AP_STA);   // AP para el dashboard + STA para ESP-NOW
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("[WiFi] AP IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("[WiFi] Mi MAC STA: ");
  Serial.println(WiFi.macAddress());

  // ── ESP-NOW ───────────────────────────────────────────────────────────
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW][ERROR] Falló init. Reiniciando...");
    delay(3000); ESP.restart();
  }
  esp_now_register_recv_cb(onRxESPNOW);

  // Registrar peer (el ESP32 SENSORES) — solo para que el stack valide
  // los paquetes entrantes; el canal 0 = usar el canal actual del AP.
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, PEER_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  Serial.println("[ESP-NOW] Receptor listo.");

  // ── Queue ESP-NOW → tareaWS ───────────────────────────────────────────
  xColaESPNOW = xQueueCreate(QUEUE_LEN, sizeof(PaqueteSensores));

  // ── Tarea WS Sensores en Core 0 ───────────────────────────────────────
  xTaskCreatePinnedToCore(
    tareaWS_Sensores,
    "tareaWS_Sens",
    6144,
    NULL,
    2,
    NULL,
    0
  );

  // ── WebSocket ─────────────────────────────────────────────────────────
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // ── HTTP ──────────────────────────────────────────────────────────────
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    const size_t total = strlen_P(INDEX_HTML);
    AsyncWebServerResponse *response = request->beginChunkedResponse(
      "text/html",
      [total](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
        if (index >= total) return 0;
        size_t n = min(total - index, maxLen);
        memcpy_P(buffer, INDEX_HTML + index, n);
        return n;
      }
    );
    response->addHeader("Cache-Control", "public, max-age=3600");
    request->send(response);
  });
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *r){ r->send(204); });
  server.onNotFound([](AsyncWebServerRequest *r){ r->send(404, "text/plain", "Not Found"); });
  server.begin();
  Serial.println("[HTTP] Servidor listo en http://192.168.4.1");

  t_prev = millis();
}

// =====================================================
// LOOP  — Control PI + telemetría motores (Core 1)
// =====================================================
// Este loop ya no lee UART. Todo lo de sensores llega por ESP-NOW
// a la Queue y se procesa en tareaWS_Sensores (Core 0).
// El loop queda libre para el control de tiempo real.
//
void loop() {
  ws.cleanupClients();

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

    // ── Log diagnóstico ──
    Serial.printf("TEL,%ld,%ld,%d,%d,%.1f,%.1f,%d,%d  |  "
                  "SENS_age=%lums  pulse_i=%ld pulse_d=%ld  heap=%u\n",
                  motor_i.medida, motor_d.medida,
                  motor_i.ref,    motor_d.ref,
                  motor_i.error,  motor_d.error,
                  motor_i.dac,    motor_d.dac,
                  ahora - t_ultimoSens,
                  pi, pd,
                  ESP.getFreeHeap());
  }
}
