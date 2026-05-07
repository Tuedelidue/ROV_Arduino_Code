#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

const unsigned long statusIntervalMs = 500;
const unsigned long errorIntervalMs = 1000;
const float tiltThresholdDeg = 10.0;

Adafruit_BNO055 bno28 = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BNO055 bno29 = Adafruit_BNO055(55, 0x29, &Wire);
Adafruit_BNO055 *bno = nullptr;

unsigned long lastStatusTime = 0;
bool bnoReady = false;

void printTiltDirection(float roll, float pitch) {
  bool tilted = false;

  Serial.print(" | Richtung: ");

  if (pitch > tiltThresholdDeg) {
    Serial.print("vorne ");
    tilted = true;
  } else if (pitch < -tiltThresholdDeg) {
    Serial.print("hinten ");
    tilted = true;
  }

  if (roll > tiltThresholdDeg) {
    Serial.print("rechts ");
    tilted = true;
  } else if (roll < -tiltThresholdDeg) {
    Serial.print("links ");
    tilted = true;
  }

  if (!tilted) {
    Serial.print("gerade");
  }
}

void scanI2C() {
  Serial.println("I2C-Scan startet...");

  byte foundCount = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C-Geraet gefunden bei 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      foundCount++;
    }
  }

  if (foundCount == 0) {
    Serial.println("Kein I2C-Geraet gefunden.");
  }
}

bool initBno() {
  Serial.println("Teste BNO055 auf Adresse 0x28...");
  if (bno28.begin()) {
    bno = &bno28;
    Serial.println("BNO055 auf 0x28 erkannt.");
    return true;
  }

  Serial.println("Teste BNO055 auf Adresse 0x29...");
  if (bno29.begin()) {
    bno = &bno29;
    Serial.println("BNO055 auf 0x29 erkannt.");
    return true;
  }

  return false;
}

void setup() {
  Serial.begin(9600);
  delay(1500);

  Serial.println();
  Serial.println("BNO055 Test fuer Arduino Mega 2560");
  Serial.println("Serial Monitor: 9600 Baud");
  Serial.println("I2C: SDA = Pin 20, SCL = Pin 21");

  Wire.begin();
  Wire.setClock(100000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif

  scanI2C();

  bnoReady = initBno();
  if (!bnoReady) {
    Serial.println("FEHLER: BNO055 wurde nicht erkannt.");
    Serial.println("Pruefe SDA D20, SCL D21, VCC, GND und Adresse 0x28/0x29.");
    return;
  }

  bno->setExtCrystalUse(true);
  Serial.println("BNO055 initialisiert.");
}

void loop() {
  unsigned long now = millis();

  if (!bnoReady) {
    if (now - lastStatusTime >= errorIntervalMs) {
      lastStatusTime = now;
      Serial.println("BNO055 nicht bereit. Keine Sensordaten.");
    }
    return;
  }

  if (now - lastStatusTime < statusIntervalMs) {
    return;
  }
  lastStatusTime = now;

  imu::Vector<3> euler = bno->getVector(Adafruit_BNO055::VECTOR_EULER);

  // VECTOR_EULER: X = Heading/Yaw, Y = Roll, Z = Pitch
  float yaw = euler.x();
  float roll = euler.y();
  float pitch = euler.z();

  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(" deg | Pitch: ");
  Serial.print(pitch);
  Serial.print(" deg | Yaw/Heading: ");
  Serial.print(yaw);
  Serial.print(" deg");
  printTiltDirection(roll, pitch);
  Serial.println();
}
