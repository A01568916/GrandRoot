/*
 https://esp32io.com/tutorials/esp32-gps
 */

#include <TinyGPS++.h>

#define GPS_BAUDRATE 9600  // The default baudrate 9600

TinyGPSPlus gps;  

#define RXD2 16
#define TXD2 17

double lat0 = 28.7385775000;   // punto de referencia
double lon0 = -106.1217361670;

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
}

double gpsToX(double lat, double lon) {
    return (lon - lon0) * cos(lat0 * PI / 180) * 111320;
  }

double gpsToY(double lat, double lon) {
  return (lat - lat0) * 111320;
  }

void loop() {
  //Serial.println(gps.satellites.value());
  if (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      if (gps.location.isValid()) {

        double lat = gps.location.lat();
        double lon = gps.location.lng();

        double x = gpsToX(lat, lon);
        double y = gpsToY(lat, lon);

        Serial.print(lat,10);
        Serial.print(",");
        Serial.print(lon,10);
        Serial.print(",");
        Serial.print(x);
        Serial.print(",");
        Serial.print(y);
        Serial.print(",");
        Serial.println(gps.satellites.value());

      } else {
        Serial.println(F("- location: ESTOY AGARRANDO SEÑAL CARNAL"));
      }
      /*Serial.print(F("- GPS date&time: "));
      if (gps.date.isValid() && gps.time.isValid()) {
        Serial.print(gps.date.year());
        Serial.print(F("-"));
        Serial.print(gps.date.month());
        Serial.print(F("-"));
        Serial.print(gps.date.day());
        Serial.print(F(" "));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        Serial.println(gps.time.second());
      } else {
        Serial.println(F("INVALID"));
      }

      Serial.println();
    }*/
  }
  if (millis() > 10000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}
}
