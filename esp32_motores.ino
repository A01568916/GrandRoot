/*
 ╔══════════════════════════════════════════════════════════════════════════════╗
 ║  GrandRoot — ESP32 MOTORES                                                   ║
 ║  Recibe comandos JSON por USB-Serial desde la Raspberry Pi                  ║
 ║  Controla dos motores DC con control PI posicional + encoders               ║
 ║                                                                              ║
 ║  PINOUT (segun tu tabla):                                                    ║
 ║    Driver Derecho:                                                           ║
 ║      D27 → FR_Derecha      (direccion motor derecho)                        ║
 ║      D26 → SV_Driver_DER   (señal de velocidad, DAC)                        ║
 ║      D33 → Enable_Derecha  (habilitar driver)                               ║
 ║      D34 → PG_DER_IN       (encoder motor derecho, INPUT ONLY)              ║
 ║                                                                              ║
 ║    Driver Izquierdo:                                                         ║
 ║      D13 → Enable_Izquierda (habilitar driver)                              ║
 ║      D14 → FR_Izquierda     (direccion motor izquierdo)                     ║
 ║      D25 → SV_Driver_IZQ    (señal de velocidad, DAC)                       ║
 ║      D32 → PG_IZQ_IN        (encoder motor izquierdo)                       ║
 ║                                                                              ║
 ║  Nota sobre D34: es INPUT ONLY en el ESP32, no tiene pull-up interno.       ║
 ║  Usa resistencia pull-up externa de 10k si el encoder la necesita.          ║
 ║                                                                              ║
 ║  Comandos JSON que acepta (un objeto por linea terminado en \n):            ║
 ║    {"cmd":"ENABLE","val":true}          — habilitar / deshabilitar motores  ║
 ║    {"cmd":"MOVE","vx":1.0,"vy":0.0}    — mover (vx: adelante, vy: giro)    ║
 ║    {"cmd":"STOP"}                       — parada inmediata                  ║
 ║    {"cmd":"EMERGENCY"}                  — parada de emergencia              ║
 ║                                                                              ║
 ║  Telemetria que publica por Serial cada SAMPLE_MS ms:                       ║
 ║    {"tel":true,"pi":5,"pd":5,"ri":10,"rd":10,"ei":5.0,"ed":5.0,           ║
 ║     "daci":120,"dacd":120,"enc":true,"ts":12345}                            ║
 ╚══════════════════════════════════════════════════════════════════════════════╝
*/

// ═══════════════════════════════════════════════════════════════════════════
// PINES — coinciden exactamente con tu pinout
// ═══════════════════════════════════════════════════════════════════════════

// Motor DERECHO
#define SV_DER    26    // DAC output → SV_Driver_DER (OpAmp+)
#define FR_DER    27    // Direccion motor derecho
#define EN_DER    33    // Enable driver derecho
#define ENC_DER   34    // Encoder derecho (PG_DER_IN) — INPUT ONLY, sin pull-up interno

// Motor IZQUIERDO
#define SV_IZQ    25    // DAC output → SV_Driver_IZQ (OpAmp+)
#define FR_IZQ    14    // Direccion motor izquierdo
#define EN_IZQ    13    // Enable driver izquierdo
#define ENC_IZQ   32    // Encoder izquierdo (PG_IZQ_IN)

// ═══════════════════════════════════════════════════════════════════════════
// PARAMETROS DE CONTROL
// ═══════════════════════════════════════════════════════════════════════════

// Tiempo de muestra del controlador PI en milisegundos
// ▼▼▼ AJUSTA AQUI LA VELOCIDAD ▼▼▼
// Pulsos por SAMPLE_MS que equivalen a velocidad maxima del robot.
// Ejemplo: PULSOS_MAX=22 a 200ms significa que a plena velocidad
// el encoder cuenta 22 pulsos en 200 ms.
// Para ir mas rapido: aumenta PULSOS_MAX o reduce SAMPLE_MS.
// Para ir mas lento: reduce PULSOS_MAX.
#define SAMPLE_MS    200
#define PULSOS_MAX   22     // pulsos por sample a velocidad maxima

// Ganancias del controlador PI
// kp: respuesta proporcional (cuanto corrige por diferencia instantanea)
// ki: respuesta integral (elimina el error de estado estacionario)
// Si el robot oscila mucho: reduce kp. Si va lento para corregir: sube ki.
float kp = 6.0f;
float ki = 3.0f;

// Anti-windup: limita cuanto puede acumularse el termino integral
// Calculado automaticamente desde ki para no saturar el DAC (0-255)
const float INTEGRAL_MAX = 255.0f / 3.0f;

// Zona muerta: voltaje minimo para vencer la friccion estatica del motor
// Si el motor no arranca con referencias bajas: sube este valor
const int DAC_MIN_ARRANQUE = 60;

// Referencia minima en giro para evitar alarma del driver
const int REF_MIN_GIRO = 10;

// ═══════════════════════════════════════════════════════════════════════════
// CINEMATICA DIFERENCIAL
// ═══════════════════════════════════════════════════════════════════════════

const float R_RUEDA = 0.1397f;   // radio de rueda en metros (ajusta al tuyo)
const float L_BASE  = 1.12f;     // distancia entre ruedas en metros (ajusta)
const float VMAX    = 4.0f;      // velocidad lineal maxima m/s
const float WMAX    = 2.0f;      // velocidad angular maxima rad/s
const float OMEGA_MAX = VMAX / R_RUEDA;

// ═══════════════════════════════════════════════════════════════════════════
// ESTADO DEL SISTEMA
// ═══════════════════════════════════════════════════════════════════════════

struct EstadoMotor {
  int   ref;          // pulsos/sample deseados
  long  medida;       // pulsos/sample medidos por encoder
  float error;        // diferencia ref - medida
  float integral;     // acumulador integral
  float u;            // salida del controlador (magnitud)
  int   dac;          // valor final enviado al DAC (0-255)
};

EstadoMotor mi = {0, 0, 0.0f, 0.0f, 0.0f, 0};  // motor izquierdo
EstadoMotor md = {0, 0, 0.0f, 0.0f, 0.0f, 0};  // motor derecho

bool motores_habilitados = false;
bool dir_actual_izq = true;    // true = adelante
bool dir_actual_der = true;

volatile long cnt_izq = 0;
volatile long cnt_der = 0;

unsigned long t_prev_control = 0;

// Buffer para el JSON entrante por Serial
String serial_buffer = "";

// ═══════════════════════════════════════════════════════════════════════════
// INTERRUPCIONES DE ENCODERS
// ═══════════════════════════════════════════════════════════════════════════

void IRAM_ATTR isr_izq() { cnt_izq++; }
void IRAM_ATTR isr_der() { cnt_der++; }

// ═══════════════════════════════════════════════════════════════════════════
// DIRECCION DE MOTORES
// ═══════════════════════════════════════════════════════════════════════════

void setDirIzq(bool adelante) {
  // El driver izquierdo tiene logica invertida respecto al derecho
  // (segun tu codigo original: IZQ LOW=adelante, DER HIGH=adelante)
  digitalWrite(FR_IZQ, adelante ? LOW : HIGH);
}

void setDirDer(bool adelante) {
  digitalWrite(FR_DER, adelante ? HIGH : LOW);
}

// ═══════════════════════════════════════════════════════════════════════════
// HABILITAR / DESHABILITAR MOTORES
// Al deshabilitar: se apaga el DAC y se suelta el pin Enable (INPUT)
// para que el driver quede en estado de alta impedancia
// ═══════════════════════════════════════════════════════════════════════════

void habilitarMotores(bool on) {
  if (on) {
    pinMode(EN_IZQ, OUTPUT);
    digitalWrite(EN_IZQ, LOW);   // LOW = enable en los drivers usados
    pinMode(EN_DER, OUTPUT);
    digitalWrite(EN_DER, LOW);
    Serial.println("[MOTOR] Motores habilitados.");
  } else {
    dacWrite(SV_IZQ, 0);
    dacWrite(SV_DER, 0);
    pinMode(EN_IZQ, INPUT);      // soltar el pin para tri-state
    pinMode(EN_DER, INPUT);
    // Reset del estado del controlador
    mi.integral = 0; mi.u = 0; mi.dac = 0; mi.ref = 0;
    md.integral = 0; md.u = 0; md.dac = 0; md.ref = 0;
    Serial.println("[MOTOR] Motores deshabilitados.");
  }
  motores_habilitados = on;
}

// ═══════════════════════════════════════════════════════════════════════════
// APLICAR REFERENCIA A LOS DOS MOTORES
// Detecta cambio de direccion y resetea integral para evitar kick
// ═══════════════════════════════════════════════════════════════════════════

void aplicarReferencias(int ri, int rd) {
  bool nueva_dir_izq = (ri >= 0);
  bool nueva_dir_der = (rd >= 0);

  if (nueva_dir_izq != dir_actual_izq) {
    dacWrite(SV_IZQ, 0);
    mi.integral = 0; mi.u = 0;
  }
  if (nueva_dir_der != dir_actual_der) {
    dacWrite(SV_DER, 0);
    md.integral = 0; md.u = 0;
  }

  setDirIzq(nueva_dir_izq);
  setDirDer(nueva_dir_der);
  dir_actual_izq = nueva_dir_izq;
  dir_actual_der = nueva_dir_der;

  mi.ref = ri;
  md.ref = rd;
}

// ═══════════════════════════════════════════════════════════════════════════
// CINEMATICA DIFERENCIAL
// Convierte vx (avance, -1..1) y vy (giro, -1..1) a pulsos/sample
// para cada rueda
// ═══════════════════════════════════════════════════════════════════════════

void cinematica(float vx, float vy, int &ref_izq, int &ref_der) {
  float V = vx * VMAX;
  float w = -vy * WMAX;

  float omega_r = V / R_RUEDA + (L_BASE / (2.0f * R_RUEDA)) * w;
  float omega_l = V / R_RUEDA - (L_BASE / (2.0f * R_RUEDA)) * w;

  // Normalizar si alguna rueda supera el maximo
  float escala = max(max(fabsf(omega_r), fabsf(omega_l)), OMEGA_MAX) / OMEGA_MAX;
  omega_r /= escala;
  omega_l /= escala;

  ref_izq = (int)roundf(constrain(omega_l / OMEGA_MAX, -1.0f, 1.0f) * PULSOS_MAX);
  ref_der = (int)roundf(constrain(omega_r / OMEGA_MAX, -1.0f, 1.0f) * PULSOS_MAX);

  // Referencia minima de giro para evitar alarma del driver
  if (ref_izq != 0)
    ref_izq = (ref_izq > 0) ? max(ref_izq, REF_MIN_GIRO) : min(ref_izq, -REF_MIN_GIRO);
  if (ref_der != 0)
    ref_der = (ref_der > 0) ? max(ref_der, REF_MIN_GIRO) : min(ref_der, -REF_MIN_GIRO);
}

// ═══════════════════════════════════════════════════════════════════════════
// PASO DEL CONTROLADOR PI
// Se llama cada SAMPLE_MS con el estado actual del motor
// ═══════════════════════════════════════════════════════════════════════════

void stepPI(EstadoMotor &m, float T) {
  m.error     = (float)m.ref - (float)m.medida;
  m.integral += m.error * T;
  m.integral  = constrain(m.integral, -INTEGRAL_MAX, INTEGRAL_MAX);  // anti-windup

  // La salida es magnitud (fabsf), la direccion la maneja FR_IZQ / FR_DER
  float u_raw = kp * m.error + ki * m.integral;
  m.u = fabsf(u_raw);
  m.u = constrain(m.u, 0.0f, 255.0f);

  // Feedforward: si hay referencia, sumar offset de zona muerta
  if (m.ref != 0) {
    m.dac = (int)constrain(m.u + DAC_MIN_ARRANQUE, (float)DAC_MIN_ARRANQUE, 255.0f);
  } else {
    m.dac = 0;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// PROCESAR COMANDO JSON RECIBIDO POR SERIAL
// Formato esperado: una linea JSON completa terminada en \n
// ═══════════════════════════════════════════════════════════════════════════

void procesarComando(String& linea) {
  linea.trim();
  if (linea.length() == 0) return;

  // Parsing manual para no necesitar la libreria ArduinoJson
  // Los comandos son simples y tienen estructura fija

  if (linea.indexOf("\"ENABLE\"") >= 0) {
    bool on = linea.indexOf("true") >= 0;
    habilitarMotores(on);

  } else if (linea.indexOf("\"STOP\"") >= 0 || linea.indexOf("\"EMERGENCY\"") >= 0) {
    habilitarMotores(false);
    Serial.println("[MOTOR][WARN] Parada recibida.");

  } else if (linea.indexOf("\"MOVE\"") >= 0) {
    if (!motores_habilitados) {
      Serial.println("[MOTOR][WARN] MOVE ignorado — motores no habilitados.");
      return;
    }

    // Extraer vx y vy del JSON
    // Formato: {"cmd":"MOVE","vx":0.5,"vy":0.0}
    float vx = 0.0f, vy = 0.0f;

    int idx_vx = linea.indexOf("\"vx\":");
    int idx_vy = linea.indexOf("\"vy\":");
    if (idx_vx >= 0) vx = linea.substring(idx_vx + 5).toFloat();
    if (idx_vy >= 0) vy = linea.substring(idx_vy + 5).toFloat();

    vx = constrain(vx, -1.0f, 1.0f);
    vy = constrain(vy, -1.0f, 1.0f);

    int ri, rd;
    cinematica(vx, vy, ri, rd);
    aplicarReferencias(ri, rd);

  } else {
    // Comando desconocido — log para diagnostico
    Serial.print("[MOTOR][WARN] Comando no reconocido: ");
    Serial.println(linea);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// PUBLICAR TELEMETRIA
// ═══════════════════════════════════════════════════════════════════════════

void publicarTelemetria() {
  Serial.printf(
    "{\"tel\":true,\"pi\":%ld,\"pd\":%ld,\"ri\":%d,\"rd\":%d,"
    "\"ei\":%.1f,\"ed\":%.1f,\"daci\":%d,\"dacd\":%d,"
    "\"enc\":%s,\"ts\":%lu}\n",
    mi.medida, md.medida,
    mi.ref,    md.ref,
    mi.error,  md.error,
    mi.dac,    md.dac,
    motores_habilitados ? "true" : "false",
    millis()
  );
}

// ═══════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n============================================");
  Serial.println("  GrandRoot — ESP32 Motores");
  Serial.println("  Control PI + Encoders");
  Serial.println("============================================");

  // Pines de direccion
  pinMode(FR_IZQ, OUTPUT); setDirIzq(true);
  pinMode(FR_DER, OUTPUT); setDirDer(true);

  // DAC en 0
  dacWrite(SV_IZQ, 0);
  dacWrite(SV_DER, 0);

  // Motores deshabilitados al inicio por seguridad
  habilitarMotores(false);

  // Encoders con interrupcion RISING
  // D34 es INPUT ONLY — no usar INPUT_PULLUP (no tiene resistencia interna)
  pinMode(ENC_IZQ, INPUT_PULLUP);
  pinMode(ENC_DER, INPUT);          // D34: usa pull-up externo de 10k si es necesario
  attachInterrupt(digitalPinToInterrupt(ENC_IZQ), isr_izq, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_DER), isr_der, RISING);

  Serial.printf("[MOTOR] Pines configurados. SAMPLE_MS=%d PULSOS_MAX=%d\n",
                SAMPLE_MS, PULSOS_MAX);
  Serial.printf("[MOTOR] kp=%.1f ki=%.1f DAC_MIN=%d\n", kp, ki, DAC_MIN_ARRANQUE);
  Serial.println("[MOTOR] Esperando comandos de la Raspberry Pi...");
  Serial.println("--------------------------------------------");

  t_prev_control = millis();
}

// ═══════════════════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════════════════

void loop() {

  // ── Leer comandos JSON que lleguen por Serial (no bloqueante) ────────────
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      // Linea completa recibida — procesar
      procesarComando(serial_buffer);
      serial_buffer = "";
    } else {
      serial_buffer += c;
      // Proteccion contra buffer overflow (linea malformada)
      if (serial_buffer.length() > 200) {
        Serial.println("[MOTOR][ERROR] Buffer overflow — descartando linea.");
        serial_buffer = "";
      }
    }
  }

  // ── Ciclo de control PI cada SAMPLE_MS ───────────────────────────────────
  unsigned long ahora = millis();
  if (ahora - t_prev_control >= SAMPLE_MS) {
    t_prev_control = ahora;

    // Leer y resetear contadores de encoder de forma atomica
    noInterrupts();
    long pi = cnt_izq;
    long pd = cnt_der;
    cnt_izq = 0;
    cnt_der = 0;
    interrupts();

    // Aplicar signo segun direccion actual
    mi.medida = dir_actual_izq ?  pi : -pi;
    md.medida = dir_actual_der ?  pd : -pd;

    // Ejecutar PI y enviar al DAC
    if (motores_habilitados) {
      float T = SAMPLE_MS / 1000.0f;
      stepPI(mi, T);
      stepPI(md, T);
      dacWrite(SV_IZQ, mi.dac);
      dacWrite(SV_DER, md.dac);
    } else {
      mi.error = 0.0f; md.error = 0.0f;
      mi.dac   = 0;    md.dac   = 0;
    }

    // Publicar telemetria hacia la Raspberry Pi
    publicarTelemetria();
  }
}
