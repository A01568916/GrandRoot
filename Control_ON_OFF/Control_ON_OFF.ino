/*
 * Control On/Off — Pulsos de dos motores DC (Izquierdo y Derecho)
 * Hardware: ESP32
 *   DAC Izq: pin 25  | DAC Der: pin 26
 *   Enc Izq: pin 34  | Enc Der: pin 32
 *   FR  Izq: pin 14  | FR  Der: pin 27
 *
 * Protocolo serial (115200 baud):
 *   Recibe: "ref_izq,ref_der\n"                               (pulsos deseados, máx 390)
 *   Envía:  "pul_izq,err_izq,esf_izq,pul_der,err_der,esf_der\n"
 */

#define SV_SIGNAL_IZQ  25     // Salida DAC motor izquierdo
#define SV_SIGNAL_DER  26     // Salida DAC motor derecho
#define ENC_IZQ        34     // Encoder izquierdo (soporta interrupciones)
#define ENC_DER        32     // Encoder derecho   (soporta interrupciones)
#define FR_IZQ         14     // Dirección motor izquierdo (fijo: LOW = adelante)
#define FR_DER         27     // Dirección motor derecho   (fijo: HIGH = adelante, negado físicamente)

#define PULSOS_MAX     390    // Máximo de pulsos por periodo de muestreo
#define SAMPLE_MS      100    // Periodo de muestreo en milisegundos

volatile long pulsosIzq = 0;
volatile long pulsosDer = 0;

long  medidaIzq    = 0, medidaDer    = 0;
float referenciaIzq = 0, referenciaDer = 0;
float errorIzq     = 0, errorDer     = 0;
int   esfuerzoIzq  = 0, esfuerzoDer  = 0;

unsigned long tAnterior = 0;

void IRAM_ATTR contarPulsoIzq() { pulsosIzq++; }
void IRAM_ATTR contarPulsoDer() { pulsosDer++; }

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(50);

    pinMode(ENC_IZQ, INPUT);   // pull-down externo a GND
    pinMode(ENC_DER, INPUT);   // pull-down externo a GND
    attachInterrupt(digitalPinToInterrupt(ENC_IZQ), contarPulsoIzq, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_DER), contarPulsoDer, RISING);

    pinMode(FR_IZQ, OUTPUT);
    pinMode(FR_DER, OUTPUT);
    digitalWrite(FR_IZQ, LOW);
    digitalWrite(FR_DER, HIGH);  // negado: sentido opuesto físico → misma dirección real

    dacWrite(SV_SIGNAL_IZQ, 0);
    dacWrite(SV_SIGNAL_DER, 0);
    tAnterior = millis();
}

void loop() {
    // Actualizar referencias si la interfaz envió nuevos valores "ref_izq,ref_der\n"
    if (Serial.available()) {
        String linea = Serial.readStringUntil('\n');
        float rIzq, rDer;
        if (sscanf(linea.c_str(), "%f,%f", &rIzq, &rDer) == 2) {
            referenciaIzq = constrain(rIzq, 0, PULSOS_MAX);
            referenciaDer = constrain(rDer, 0, PULSOS_MAX);
        }
    }

    // Ejecutar control cada SAMPLE_MS milisegundos
    if (millis() - tAnterior >= SAMPLE_MS) {
        tAnterior = millis();

        // Leer pulsos de forma segura (deshabilitar ISRs momentáneamente)
        noInterrupts();
        medidaIzq = pulsosIzq;  pulsosIzq = 0;
        medidaDer = pulsosDer;  pulsosDer = 0;
        interrupts();

        // Ley de control On/Off con histéresis — Izquierdo
        // Enciende: medida < (referencia - 40) | Apaga: medida >= referencia
        errorIzq = referenciaIzq - medidaIzq;
        if (esfuerzoIzq == 0) {
            if (medidaIzq < (referenciaIzq - 40)) esfuerzoIzq = 255;
        } else {
            if (medidaIzq >= referenciaIzq)        esfuerzoIzq = 0;
        }
        dacWrite(SV_SIGNAL_IZQ, esfuerzoIzq);

        // Ley de control On/Off con histéresis — Derecho
        // Enciende: medida < (referencia - 40) | Apaga: medida >= referencia
        errorDer = referenciaDer - medidaDer;
        if (esfuerzoDer == 0) {
            if (medidaDer < (referenciaDer - 40)) esfuerzoDer = 255;
        } else {
            if (medidaDer >= referenciaDer)        esfuerzoDer = 0;
        }
        dacWrite(SV_SIGNAL_DER, esfuerzoDer);

        // Enviar datos: pul_izq,err_izq,esf_izq,pul_der,err_der,esf_der
        Serial.print(medidaIzq);       Serial.print(",");
        Serial.print(errorIzq,   1);   Serial.print(",");
        Serial.print(esfuerzoIzq);     Serial.print(",");
        Serial.print(medidaDer);       Serial.print(",");
        Serial.print(errorDer,   1);   Serial.print(",");
        Serial.println(esfuerzoDer);
    }
}
