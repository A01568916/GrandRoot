#define SV_SIGNAL_IZQ 25
#define SV_SIGNAL_DER 26

#define ENC_DER 32
#define ENC_IZQ 34
#define EN_DER 33
#define EN_IZQ 13
#define FR_DER 27
#define FR_IZQ 14

// ===== CONSTANTES =====
const float DIST_POR_PULSO = 0.0585; // metros
const int STEP_DAC = 10;
const int MAX_DAC = 250;

const int TIEMPO_MEDICION = 4000; // ms
const int TIEMPO_RESET = 300;     // ms (ajustable)

// Pulsos
volatile int pulsosDer = 0;
volatile int pulsosIzq = 0;

// ===== INTERRUPCIONES =====
void IRAM_ATTR contarDer() {
  pulsosDer++;
}

void IRAM_ATTR contarIzq() {
  pulsosIzq++;
}

void setup() {
  Serial.begin(115200);

  pinMode(ENC_DER, INPUT_PULLUP);
  pinMode(ENC_IZQ, INPUT_PULLUP);

    // Pines de control
  pinMode(EN_DER, OUTPUT);
  pinMode(EN_IZQ, OUTPUT);
  pinMode(FR_DER, OUTPUT);
  pinMode(FR_IZQ, OUTPUT);

  // Activar motores (igual que tu caso E)
  digitalWrite(EN_DER, HIGH);
  digitalWrite(EN_IZQ, HIGH);
  digitalWrite(FR_DER, LOW); // adelante
  digitalWrite(FR_IZQ, HIGH); // adelante

  attachInterrupt(digitalPinToInterrupt(ENC_DER), contarDer, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IZQ), contarIzq, RISING);

  Serial.println("Prueba DAC con reset a 0V entre pasos...");
}

void loop() {

  for (int dac = 0; dac <= MAX_DAC; dac += STEP_DAC) {

    // ===== RESET A 0V =====
    dacWrite(SV_SIGNAL_DER, 0);
    dacWrite(SV_SIGNAL_IZQ, 0);

    delay(TIEMPO_RESET); // dejar que el driver se estabilice

    // ===== APLICAR NUEVO VALOR =====
    dacWrite(SV_SIGNAL_DER, dac);
    dacWrite(SV_SIGNAL_IZQ, dac);

    float voltaje = (dac / 255.0) * 3.3;

    // Reset pulsos protegido
    noInterrupts();
    pulsosDer = 0;
    pulsosIzq = 0;
    interrupts();

    unsigned long startTime = millis();

    // Medición 2 segundos
    while (millis() - startTime < TIEMPO_MEDICION) {
      // interrupciones trabajando
    }

    float tiempo = (millis() - startTime) / 1000.0;

    // Lectura segura
    noInterrupts();
    int pDer = pulsosDer;
    int pIzq = pulsosIzq;
    interrupts();

    // Velocidad
    float velDer = (pDer * DIST_POR_PULSO) / tiempo;
    float velIzq = (pIzq * DIST_POR_PULSO) / tiempo;

    // ===== OUTPUT =====
    Serial.println("=================================");
    Serial.print("DAC: "); Serial.println(dac);

    Serial.print("Voltaje: ");
    Serial.print(voltaje, 3);
    Serial.println(" V");

    Serial.print("Pulsos Der: ");
    Serial.print(pDer);
    Serial.print(" | Pulsos Izq: ");
    Serial.println(pIzq);

    Serial.print("Vel Der: ");
    Serial.print(velDer, 3);
    Serial.print(" m/s | Vel Izq: ");
    Serial.print(velIzq, 3);
    Serial.println(" m/s");
  }

  Serial.println("PRUEBA TERMINADA");

  // Apagar al final
  dacWrite(SV_SIGNAL_DER, 0);
  dacWrite(SV_SIGNAL_IZQ, 0);

  while (true);
}