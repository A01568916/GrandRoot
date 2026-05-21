// GrandRoot — Control de LEDs por Serial
// Recibe 'R' para LED rojo, 'B' para LED azul

const int PIN_ROJO = 2;
const int PIN_AZUL = 4;

void setup() {
  Serial.begin(9600);
  pinMode(PIN_ROJO, OUTPUT);
  pinMode(PIN_AZUL, OUTPUT);
  digitalWrite(PIN_ROJO, LOW);
  digitalWrite(PIN_AZUL, LOW);
  Serial.println("[ESP32] Listo");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 'R') {
      digitalWrite(PIN_ROJO, HIGH);
      digitalWrite(PIN_AZUL, LOW);
      Serial.println("[ESP32] LED Rojo ON");
    }
    else if (cmd == 'B') {
      digitalWrite(PIN_AZUL, HIGH);
      digitalWrite(PIN_ROJO, LOW);
      Serial.println("[ESP32] LED Azul ON");
    }
  }
}
