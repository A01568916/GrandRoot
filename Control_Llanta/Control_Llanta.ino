#define SV_SIGNAL_IZQ 25
#define SV_SIGNAL_DER 26
#define EN_DER 33
#define EN_IZQ 13
#define FR_DER 27
#define FR_IZQ 14

//SV SIGNAL DER GPIO 26
//SV SIGNAL IZQ GPIO 25
// ENABLE DER GPIO33
// ENABLE IZQ 13
// FR DER 27
// FR IZQ 14
int lastVoltage = 0;

void setup() {
  Serial.begin(115200);

  // Inicializar con pull-down para evitar estados flotantes
  pinMode(EN_DER, INPUT_PULLDOWN);
  pinMode(EN_IZQ, INPUT_PULLDOWN);
  pinMode(FR_DER, INPUT_PULLDOWN);
  pinMode(FR_IZQ, INPUT_PULLDOWN);

  delay(10); // pequeño tiempo de estabilización


  pinMode(EN_DER, OUTPUT);
  pinMode(EN_IZQ, OUTPUT);
  pinMode(FR_DER, OUTPUT); //LOW PARA REVERSA HIGH PARA DELANTE
  pinMode(FR_IZQ, OUTPUT); //LOW PARA DELANTE HIGH PARA REVERSA

  Serial.println("Ingresa el caso (A, B, C, D) o voltaje en mV (0-3300):");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Elimina espacios y saltos de línea

    // ===== CASOS POR LETRA =====
    if (input == "A") { //ACTIVA TODO, MOTORES ENABLE, FR'S EN 1
      digitalWrite(EN_DER, HIGH);
      digitalWrite(EN_IZQ, HIGH);
      digitalWrite(FR_DER, HIGH);
      digitalWrite(FR_IZQ, HIGH);

    } else if (input == "B") { //APAGA TODO, MOTORES APAGADOS, FR'S EN 0
      digitalWrite(EN_DER, LOW);
      digitalWrite(EN_IZQ, LOW);
      digitalWrite(FR_DER, LOW);
      digitalWrite(FR_IZQ, LOW);

    } else if (input == "C") { //MOTORES PRENDIDOS, LLANTAS ROTACION DIFERENCIAL, DER AVANZA, IZQ RETROCEDE
      digitalWrite(EN_DER, HIGH);
      digitalWrite(EN_IZQ, HIGH);
      digitalWrite(FR_DER, LOW);
      digitalWrite(FR_IZQ, HIGH);

    } else if (input == "D") { //MOTORES PRENDIDOS, LLANTAS ROTACION DIFERENCIAL, IZQ AVANZA, DER RETROCEDE
      digitalWrite(EN_DER, HIGH);
      digitalWrite(EN_IZQ, HIGH);
      digitalWrite(FR_DER, HIGH);
      digitalWrite(FR_IZQ, LOW);

    }else if (input == "E") {//MOTORES PRENDIDOS, LLANTAS EN MISMA DIRECCION HACIA DELANTE, FR'S EN 0
      digitalWrite(EN_DER, HIGH);
      digitalWrite(EN_IZQ, HIGH);
      digitalWrite(FR_DER, LOW);
      digitalWrite(FR_IZQ, LOW);

    }
    else {
      int mV = input.toInt(); // Convierte texto a número

      // Validación
      if (mV < 0) mV = 0;
      if (mV > 3300) mV = 3300;

      lastVoltage = mV; // Guardar valor

      int dacValue = (mV * 255) / 3300;

      dacWrite(SV_SIGNAL_IZQ, dacValue);
      dacWrite(SV_SIGNAL_DER, dacValue);

      Serial.print("Voltaje aplicado: ");
      Serial.println(lastVoltage);
    } 
}}


