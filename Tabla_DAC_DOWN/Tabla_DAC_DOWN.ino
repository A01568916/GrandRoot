#define SV_SIGNAL_IZQ 25
#define SV_SIGNAL_DER 26

#define ENC_DER 32
#define ENC_IZQ 34
#define ENC_DER 32
#define ENC_IZQ 34
#define EN_DER 33
#define EN_IZQ 13
#define FR_DER 27
#define FR_IZQ 14

// ===== CONSTANTES =====
const float DIST_POR_PULSO = 0.0585; // metros
const int STEP_DAC = 10;
const int Min_DAC = 0;
const int INTERVALO = 4000; // ms

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

  Serial.println("Inicio prueba DAC vs velocidad...");
}

void loop() {

  for (int dac = 250; dac >= Min_DAC; dac -= STEP_DAC) {

    // Aplicar DAC
    dacWrite(SV_SIGNAL_DER, dac);
    dacWrite(SV_SIGNAL_IZQ, dac);

    // Convertir a voltaje
    float voltaje = (dac / 255.0) * 3.3;

    // Reset pulsos (protegido)
    noInterrupts();
    pulsosDer = 0;
    pulsosIzq = 0;
    interrupts();

    unsigned long startTime = millis();

    // Medir durante 2 segundos
    while (millis() - startTime < INTERVALO) {
      // Loop vacío → interrupciones cuentan pulsos
    }

    // Tiempo real en segundos
    float tiempo = (millis() - startTime) / 1000.0;

    // ===== LECTURA SEGURA (MEJORA PRO) =====
    noInterrupts();
    int pDer = pulsosDer;
    int pIzq = pulsosIzq;
    interrupts();

    // Calcular velocidades
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

  // Detener motores
  dacWrite(SV_SIGNAL_DER, 0);
  dacWrite(SV_SIGNAL_IZQ, 0);

  while (true); // detener ejecución completamente
}