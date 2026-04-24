#define SV_SIGNAL_IZQ 25
#define SV_SIGNAL_DER 26
#define EN_DER 33
#define EN_IZQ 13
#define FR_DER 27
#define FR_IZQ 14

#define ENC_DER 32
#define ENC_IZQ 12

volatile int pulsosDer = 0;
volatile int pulsosIzq = 0;

int lastVoltage = 0;

// ===== INTERRUPCIONES =====
void IRAM_ATTR contarDer() {
  pulsosDer++;
}

void IRAM_ATTR contarIzq() {
  pulsosIzq++;
}

void setup() {
  Serial.begin(115200);

  // Pines motores
  pinMode(EN_DER, OUTPUT);
  pinMode(EN_IZQ, OUTPUT);
  pinMode(FR_DER, OUTPUT);
  pinMode(FR_IZQ, OUTPUT);

  // Encoders
  pinMode(ENC_DER, INPUT_PULLUP);
  pinMode(ENC_IZQ, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_DER), contarDer, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IZQ), contarIzq, RISING);

  Serial.println("Sistema listo: motores + encoders");
}

void loop() {

  // ===== CONTROL POR SERIAL (TU CÓDIGO) =====
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "A") {
      digitalWrite(EN_DER, HIGH);
      digitalWrite(EN_IZQ, HIGH);
      digitalWrite(FR_DER, HIGH);
      digitalWrite(FR_IZQ, HIGH);

    } else if (input == "B") {
      digitalWrite(EN_DER, LOW);
      digitalWrite(EN_IZQ, LOW);
      digitalWrite(FR_DER, LOW);
      digitalWrite(FR_IZQ, LOW);

    } else if (input == "C") {
      digitalWrite(EN_DER, HIGH);
      digitalWrite(EN_IZQ, HIGH);
      digitalWrite(FR_DER, LOW);
      digitalWrite(FR_IZQ, HIGH);

    } else if (input == "D") {
      digitalWrite(EN_DER, HIGH);
      digitalWrite(EN_IZQ, HIGH);
      digitalWrite(FR_DER, HIGH);
      digitalWrite(FR_IZQ, LOW);

    } else if (input == "E") {
      digitalWrite(EN_DER, HIGH);
      digitalWrite(EN_IZQ, HIGH);
      digitalWrite(FR_DER, LOW);
      digitalWrite(FR_IZQ, LOW);

    } else {
      int mV = input.toInt();

      if (mV < 0) mV = 0;
      if (mV > 3300) mV = 3300;

      lastVoltage = mV;

      int dacValue = (mV * 255) / 3300;

      dacWrite(SV_SIGNAL_IZQ, dacValue);
      dacWrite(SV_SIGNAL_DER, dacValue);

      Serial.print("Voltaje aplicado: ");
      Serial.println(lastVoltage);
    }
  }

  // ===== LECTURA DE ENCODERS (CADA 1 SEGUNDO) =====
  static unsigned long lastTime = 0;

  if (millis() - lastTime >= 1000) {
    lastTime = millis();

    Serial.print("Pulsos Der: ");
    Serial.print(pulsosDer);
    Serial.print(" | Pulsos Izq: ");
    Serial.println(pulsosIzq);

    pulsosDer = 0;
    pulsosIzq = 0;
  }
}