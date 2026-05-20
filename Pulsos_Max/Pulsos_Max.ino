/*
 * Pulsos_Max — Medición de pulsos máximos a 3.3 V
 * Hardware: ESP32
 *   DAC Izq: pin 25  |  DAC Der: pin 26
 *   Enc Izq: pin 34  |  Enc Der: pin 32
 *   FR  Izq: pin 14  |  FR  Der: pin 27
 *
 * Secuencia:
 *   0–2 s  →  0 V  (motores detenidos)
 *   2 s+   →  3.3 V (DAC = 255)
 * Imprime cada 1000 ms:  t_ms, pulsos_izq, pulsos_der
 */

// ── Pines (drivers BLDC WS55-220) ───────────────────────────────────────────
#define DAC_IZQ  25   // DAC → velocidad motor izquierdo
#define DAC_DER  26   // DAC → velocidad motor derecho
#define FR_IZQ         14   // LOW = adelante, HIGH = atrás
#define FR_DER         27   // HIGH = adelante, LOW = atrás  
#define EN_IZQ         21   // OUTPUT+LOW = activo | INPUT = desactivado
#define EN_DER         33   // idem
#define ENC_IZQ        32
#define ENC_DER        34

#define SAMPLE_MS  200
#define ARRANQUE_MS 2000   // tiempo en 0 V antes de arrancar

volatile long pulsosIzq = 0;
volatile long pulsosDer = 0;

void IRAM_ATTR contarIzq() { pulsosIzq++; }
void IRAM_ATTR contarDer() { pulsosDer++; }

unsigned long tAnterior = 0;
bool arrancado = false;

void setup() {
    Serial.begin(115200);

    pinMode(ENC_IZQ, INPUT);
    pinMode(ENC_DER, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENC_IZQ), contarIzq, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_DER), contarDer, RISING);

    pinMode(FR_IZQ, OUTPUT);
    pinMode(FR_DER, OUTPUT);
    digitalWrite(FR_IZQ, LOW);
    digitalWrite(FR_DER, HIGH); 

    dacWrite(DAC_IZQ, 0);
    dacWrite(DAC_DER, 0);

    Serial.println("t_ms,pulsos_izq,pulsos_der");

    delay(ARRANQUE_MS);          // 2 s quieto

    dacWrite(DAC_IZQ, 255);      // 3.3 V
    dacWrite(DAC_DER, 255);
    arrancado = true;

    tAnterior = millis();
}

void loop() {
    if (millis() - tAnterior >= SAMPLE_MS) {
        tAnterior = millis();

        noInterrupts();
        long pIzq = pulsosIzq;  pulsosIzq = 0;
        long pDer = pulsosDer;  pulsosDer = 0;
        interrupts();

        Serial.print(millis());   Serial.print(",");
        Serial.print(pIzq);       Serial.print(",");
        Serial.println(pDer);
    }
}
