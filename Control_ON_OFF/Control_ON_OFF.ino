/*
 * Control On/Off — Pulsos motor DC Derecho
 * Hardware: ESP32
 *   DAC Der: pin 26
 *   Enc Der: pin 32
 *   FR  Der: pin 27
 *
 * Protocolo serial (115200 baud):
 *   Recibe: "ref\n"                    (pulsos deseados, máx 98)
 *   Envía:  "pul_der,err_der,esf_der\n"
 */

#define SV_SIGNAL_DER  26     // Salida DAC motor derecho
#define ENC_DER        32     // Encoder derecho (soporta interrupciones)
#define FR_DER         27     // Dirección motor derecho (fijo: HIGH = adelante, negado físicamente)

#define PULSOS_MAX     110    // Máximo de pulsos por periodo de muestreo
#define SAMPLE_MS      1000   // Periodo de muestreo en milisegundos

volatile long pulsosDer = 0;

long  medidaDer   = 0;
float referencia  = 0;
float errorDer    = 0;
int   esfuerzoDer = 0;

unsigned long tAnterior = 0;

void IRAM_ATTR contarPulsoDer() { pulsosDer++; }

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(50);

    pinMode(ENC_DER, INPUT);   // pull-down externo a GND
    attachInterrupt(digitalPinToInterrupt(ENC_DER), contarPulsoDer, RISING);

    pinMode(FR_DER, OUTPUT);
    digitalWrite(FR_DER, HIGH);  // negado: sentido opuesto físico → misma dirección real

    dacWrite(SV_SIGNAL_DER, 0);
    tAnterior = millis();
}

void loop() {
    // Actualizar referencia si la interfaz envió un nuevo valor "ref\n"
    if (Serial.available()) {
        String linea = Serial.readStringUntil('\n');
        float rRef;
        if (sscanf(linea.c_str(), "%f", &rRef) == 1) {
            referencia = constrain(rRef, 0, PULSOS_MAX);
        }
    }

    // Ejecutar control cada SAMPLE_MS milisegundos
    if (millis() - tAnterior >= SAMPLE_MS) {
        tAnterior = millis();

        // Leer pulsos de forma segura (deshabilitar ISRs momentáneamente)
        noInterrupts();
        medidaDer = pulsosDer;  pulsosDer = 0;
        interrupts();

        // Ley de control On/Off con histéresis — Derecho
        // Enciende: medida < (referencia - 20) | Apaga: medida >= referencia
        errorDer = referencia - medidaDer;
        //if (esfuerzoDer == 0) {
           // if (medidaDer < (referencia - 20)) esfuerzoDer = 255;
        //} else {
        //    if (medidaDer >= referencia)        esfuerzoDer = 0;
        //}
        if (medidaDer < referencia) esfuerzoDer = 255;
        else                         esfuerzoDer = 0;

        dacWrite(SV_SIGNAL_DER, esfuerzoDer);

        // Enviar datos: pul_der,err_der,esf_der
        Serial.print(medidaDer);      Serial.print(",");
        Serial.print(errorDer,   1);  Serial.print(",");
        Serial.println(esfuerzoDer);
    }
}
