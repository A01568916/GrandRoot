/*
 * Carrito_Vision.ino
 * ESP32 — Control directo puente H (L298N o similar)
 * Sin encoders — control on/off con PWM de velocidad
 *
 * Pines:
 *   Enable A (PWM izq) → 26
 *   In1               → 27
 *   In2               → 18
 *   In3               → 19
 *   In4               → 14
 *   Enable B (PWM der) → 25
 *
 * Protocolo Serial (mismo que robot_vision_serial.py):
 *   PC → ESP32 :  ENABLE,true / ENABLE,false
 *   PC → ESP32 :  MOVE,vx,vy        (vx,vy en [-1,1])
 *   ESP32 → PC :  TEL,vx,vy,dir     (eco para que el .py no cambie)
 */

// =====================================================
// PINES
// =====================================================

#define EN_A   26    // PWM motor izquierdo
#define IN1    27
#define IN2    18

#define IN3    19
#define IN4    14
#define EN_B   25    // PWM motor derecho

// =====================================================
// PWM (LEDC)
// =====================================================

#define PWM_FREQ     1000
#define PWM_BITS     8     // 0-255

// =====================================================
// PARÁMETROS
// =====================================================

// Velocidad base cuando vx > 0 (0-255)
// Cambia este valor para ajustar la velocidad del carrito
#define VELOCIDAD_BASE  180

// Umbral minimo de vx para considerar movimiento
#define VX_UMBRAL  0.05f

// =====================================================
// ESTADO
// =====================================================

bool motors_enabled = false;
String serialBuffer = "";

#define WATCHDOG_MS  600
unsigned long t_ultimo_move = 0;

// =====================================================
// CONTROL MOTORES
// =====================================================

// Detener ambos motores
void detener() {
  ledcWrite(EN_A, 0);
  ledcWrite(EN_B, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// Aplicar direccion y velocidad a cada lado
//   vel: 0-255
//   dir: true=adelante, false=atras
void setMotorIzq(int vel, bool adelante) {
  digitalWrite(IN1, adelante ? HIGH : LOW);
  digitalWrite(IN2, adelante ? LOW  : HIGH);
  ledcWrite(EN_A, vel);
}

void setMotorDer(int vel, bool adelante) {
  digitalWrite(IN3, adelante ? HIGH : LOW);
  digitalWrite(IN4, adelante ? LOW  : HIGH);
  ledcWrite(EN_B, vel);
}

// Convierte vx,vy [-1,1] en comandos de motor
void aplicarMovimiento(float vx, float vy) {

  if (fabsf(vx) < VX_UMBRAL && fabsf(vy) < VX_UMBRAL) {
    detener();
    return;
  }

  bool adelante = (vx >= 0.0f);
  int  vel      = VELOCIDAD_BASE;

  // Sin encoders: la velocidad es fija (VELOCIDAD_BASE).
  // vy controla giro reduciendo el motor del lado al que gira.
  // vy > 0 = girar derecha → motor derecho mas lento
  // vy < 0 = girar izquierda → motor izquierdo mas lento

  int vel_izq = vel;
  int vel_der = vel;

  if (vy > 0.1f) {
    // Girar derecha: reducir motor derecho
    vel_der = (int)(vel * (1.0f - fabsf(vy)));
    vel_der = max(0, vel_der);
  } else if (vy < -0.1f) {
    // Girar izquierda: reducir motor izquierdo
    vel_izq = (int)(vel * (1.0f - fabsf(vy)));
    vel_izq = max(0, vel_izq);
  }

  setMotorIzq(vel_izq, adelante);
  setMotorDer(vel_der, adelante);
}

void enableMotores(bool on) {
  if (!on) detener();
  motors_enabled = on;
}

// =====================================================
// SERIAL
// =====================================================

void procesarComando(String cmd) {

  cmd.trim();

  if (cmd.startsWith("ENABLE,")) {
    bool on = (cmd.substring(7) == "true");
    enableMotores(on);
    Serial.print("ENABLE,");
    Serial.println(on ? "true" : "false");
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

    aplicarMovimiento(vx, vy);
    t_ultimo_move = millis();

    // Eco de telemetria — mantiene compatible el .py sin cambios
    String dir = "RECTO";
    if      (vy >  0.1f) dir = "DERECHA";
    else if (vy < -0.1f) dir = "IZQUIERDA";

    Serial.print("TEL,");
    Serial.print(vx, 3); Serial.print(",");
    Serial.print(vy, 3); Serial.print(",");
    Serial.println(dir);
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

  // Pines de direccion
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // PWM (API Core 3.x)
  ledcAttach(EN_A, PWM_FREQ, PWM_BITS);
  ledcAttach(EN_B, PWM_FREQ, PWM_BITS);

  detener();

  t_ultimo_move = millis();

  Serial.println("ESP32 READY");
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  leerSerial();

  // Watchdog: si Python deja de enviar, parar
  if (motors_enabled &&
      (millis() - t_ultimo_move) > WATCHDOG_MS) {
    detener();
  }
}
