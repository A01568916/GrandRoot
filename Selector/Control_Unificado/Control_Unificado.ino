/*
 * Control Unificado: On/Off | P | PI | PID — Motor DC Derecho
 * Hardware: ESP32
 *   DAC Der: pin 26
 *   Enc Der: pin 32
 *   FR  Der: pin 27
 *
 * Protocolo serial (115200 baud):
 *   Recibe: "ref,modo\n"              ref = pulsos deseados (0–110), modo = 0|1|2|3
 *   Envía:  "pul_der,err_der,esf_der\n"
 *
 * Modos:
 *   0 → On/Off
 *   1 → P
 *   2 → PI
 *   3 → PID
 *
 * Consideraciones de cambio de modo:
 *   - El modo solo se acepta cuando referencia == 0 Y esfuerzo actual == 0.
 *   - Al cambiar de modo se reinician TODAS las variables internas del controlador
 *     (integradores, errores anteriores) para evitar wind-up o transitorios.
 *   - El motor se lleva a 0 (dacWrite 0) antes de aplicar el nuevo modo.
 *   - Mientras el motor esté en movimiento (esfuerzoDer > 0) el GUI debe
 *     deshabilitar el selector; el ESP confirma el modo activo en cada trama.
 */

#define SV_SIGNAL_DER  26
#define ENC_DER        32
#define FR_DER         27

#define PULSOS_MAX     110
#define SAMPLE_MS      1000

// ── Ganancias ──────────────────────────────────────────────────
const float KP_P   = 1.3;
const float KP_PI  = 1.3;
const float KI_PI  = 0.5;
const float KP_PID = 0.5;
const float KI_PID = KP_PID/2;   // kp/2
const float KD_PID = KI_PID/10;  // ki/10

// ── Estado del encoder ─────────────────────────────────────────
volatile long pulsosDer = 0;
void IRAM_ATTR contarPulsoDer() { pulsosDer++; }

// ── Variables compartidas del control ──────────────────────────
long  medidaDer   = 0;
float referencia  = 0.0;
float errorDer    = 0.0;
int   esfuerzoDer = 0;
float porcentaje = 0;

// ── Variables internas de los controladores ────────────────────
float ua         = 0.0;   // acumulador PI / PID (u(k-1))
float errorant   = 0.0;   // error(k-1)  — PID
float errorantant= 0.0;   // error(k-2)  — PID

// ── Modo activo ────────────────────────────────────────────────
//   0=On/Off  1=P  2=PI  3=PID
int modoActivo   = 0;
int modoPendiente= 0;   // modo solicitado desde el GUI, aún no aplicado

unsigned long tAnterior = 0;

// ══════════════════════════════════════════════════════════════
//  resetControlador — limpia TODAS las variables internas
//  Llamar SIEMPRE antes de activar un nuevo modo.
// ══════════════════════════════════════════════════════════════
void resetControlador() {
    ua          = 0.0;
    errorant    = 0.0;
    errorantant = 0.0;
    errorDer    = 0.0;
    esfuerzoDer = 0;
    dacWrite(SV_SIGNAL_DER, 0);   // motor a cero de forma inmediata
}

// ══════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(50);

    pinMode(ENC_DER, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENC_DER), contarPulsoDer, RISING);

    pinMode(FR_DER, OUTPUT);
    digitalWrite(FR_DER, HIGH);

    dacWrite(SV_SIGNAL_DER, 0);
    tAnterior = millis();
}

// ══════════════════════════════════════════════════════════════
void loop() {

    // ── 1. Leer serial: formato "ref,modo\n"  o  "R\n" (reset DAC) ──
    if (Serial.available()) {
        String linea = Serial.readStringUntil('\n');
        linea.trim();

        // Comando de reset inmediato: escribe 0 al DAC sin tocar el control
        if (linea == "R") {
            dacWrite(SV_SIGNAL_DER, 0);
            esfuerzoDer = 0;
            resetControlador();   // limpia integradores también
            return;               // no procesar como ref/modo
        }

        float  rRef  = 0.0;
        int    rModo = modoActivo;

        int comas = 0;
        for (char c : linea) if (c == ',') comas++;

        if (comas >= 1) {
            // Dos campos: ref,modo
            int idx = linea.indexOf(',');
            rRef  = linea.substring(0, idx).toFloat();
            rModo = linea.substring(idx + 1).toInt();
        } else {
            // Solo referencia (compatibilidad hacia atrás)
            rRef  = linea.toFloat();
            rModo = modoActivo;
        }

        referencia    = constrain(rRef,  0.0, (float)PULSOS_MAX);
        modoPendiente = constrain(rModo, 0,   3);
    }

    // ── 2. Cambio de modo (solo cuando es seguro) ─────────────
    //    Condición: referencia == 0 Y esfuerzo actual == 0
    //    Esto garantiza que el motor esté detenido antes del cambio.
    if (modoPendiente != modoActivo) {
        if (referencia == 0.0 && esfuerzoDer == 0) {
            resetControlador();          // limpia integradores y para motor
            modoActivo = modoPendiente;
        }
        // Si no se cumple la condición, se ignora el cambio este ciclo;
        // el GUI debe reenviar el modo mientras el motor baje a cero.
    }

    // ── 3. Ciclo de control cada SAMPLE_MS ───────────────────
    if (millis() - tAnterior >= SAMPLE_MS) {
        tAnterior = millis();

        // Leer encoder de forma atómica
        noInterrupts();
        medidaDer = pulsosDer;
        pulsosDer = 0;
        interrupts();

        errorDer = referencia - medidaDer;

        // ── Ley de control según modo ─────────────────────────
        switch (modoActivo) {

            // ─────────────────────────────────────────
            case 0:  // On/Off
            // ─────────────────────────────────────────
                if (medidaDer < referencia) esfuerzoDer = 255;
                else                        esfuerzoDer = 0;
                break;

            // ─────────────────────────────────────────
            case 1:  // Proporcional
            // ─────────────────────────────────────────
                esfuerzoDer = (int)(KP_P * errorDer);
                break;

            // ─────────────────────────────────────────
            case 2:  // PI  (velocidad: u(k) = Ki·e(k) + u(k-1))
            // ─────────────────────────────────────────
                esfuerzoDer = (int)(KP_PI * errorDer + KI_PI * errorDer + ua);
                ua = esfuerzoDer;
                break;

            // ─────────────────────────────────────────
            case 3:  // PID  (forma recursiva de posición)
            // ─────────────────────────────────────────
                esfuerzoDer = (int)(
                    KP_PID * errorDer
                    + KP_PID * errorant
                    + KI_PID * errorDer
                    + KD_PID * errorDer
                    - 2.0f * KD_PID * errorant
                    + KD_PID * errorantant
                    + ua
                );
                ua          = esfuerzoDer;
                errorantant = errorant;
                errorant    = errorDer;
                break;
        }

        // ── Saturación de la salida ───────────────────────────
        if (esfuerzoDer > 255) esfuerzoDer = 255;
        if (esfuerzoDer < 0)   esfuerzoDer = 0;

        // ── Anti wind-up: limitar acumulador PI/PID ───────────
        //    Si la salida saturó, el acumulador no debe crecer más
        if (ua > 255) ua = 255;
        if (ua < 0)   ua = 0;

        dacWrite(SV_SIGNAL_DER, esfuerzoDer);
        porcentaje = (referencia > 0) ? (errorDer / referencia * 100.0f) : 0.0f;

        // ── Enviar trama: pul_der,err_der,esf_der,modo ────────
        Serial.print(medidaDer);        Serial.print(",");
        Serial.print(porcentaje,     1);  Serial.print(",");
        Serial.print(esfuerzoDer);      Serial.print(",");
        Serial.println(modoActivo);     // GUI usa esto para confirmar modo activo
    }
}
