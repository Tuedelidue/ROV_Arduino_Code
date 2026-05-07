#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

const byte leftHorizontalEscPin = 9;
const byte rightHorizontalEscPin = 10;
const byte verticalEsc1Pin = 11;
const byte verticalEsc2Pin = 12;

const byte joystick1XPin = A8;
const byte joystick1YPin = A9;
const byte joystick1ButtonPin = 24;

const byte joystick2XPin = A10;
const byte joystick2YPin = A11;
const byte joystick2ButtonPin = 25;

const int neutralPulse = 1500;
const int minPulse = 1400;
const int maxPulse = 1600;
const int joystickCenter = 512;
const int joystickDeadzone = 60;
const int maxOutputOffset = 100;
const int maxYawCorrection = 30;
const float yawKp = 1.5;
const unsigned long statusIntervalMs = 300;
const unsigned long debounceMs = 50;

Servo leftHorizontalEsc;
Servo rightHorizontalEsc;
Servo verticalEsc1;
Servo verticalEsc2;
Adafruit_BNO055 bno28 = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BNO055 bno29 = Adafruit_BNO055(55, 0x29, &Wire);
Adafruit_BNO055 *bno = nullptr;

unsigned long lastStatusTime = 0;
int lastLeftHorizontalPulse = neutralPulse;
int lastRightHorizontalPulse = neutralPulse;
int lastVerticalPulse = neutralPulse;
int lastYawCorrection = 0;
bool bnoReady = false;
bool stabilizeMode = false;
bool lastJoystick1ButtonReading = HIGH;
bool joystick1ButtonState = HIGH;
unsigned long lastJoystick1DebounceTime = 0;
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;
float targetYaw = 0.0;
float yawError = 0.0;

// Wandelt einen Analogwert in einen sicheren Ausschlag von -100 bis +100 us um.
int readAxisWithDeadzone(int rawValue) {
  int centeredValue = rawValue - joystickCenter;

  if (abs(centeredValue) <= joystickDeadzone) {
    return 0;
  }

  if (centeredValue > 0) {
    return map(centeredValue, joystickDeadzone + 1, 511, 0, maxOutputOffset);
  }

  return map(centeredValue, -joystickDeadzone - 1, -512, 0, -maxOutputOffset);
}

// Begrenzt alle Pulse hart auf den sicheren Testbereich und sendet sie an die ESCs.
void sendAllThrusters(int leftHorizontalPulse, int rightHorizontalPulse, int verticalPulse) {
  lastLeftHorizontalPulse = constrain(leftHorizontalPulse, minPulse, maxPulse);
  lastRightHorizontalPulse = constrain(rightHorizontalPulse, minPulse, maxPulse);
  lastVerticalPulse = constrain(verticalPulse, minPulse, maxPulse);

  leftHorizontalEsc.writeMicroseconds(lastLeftHorizontalPulse);
  rightHorizontalEsc.writeMicroseconds(lastRightHorizontalPulse);
  verticalEsc1.writeMicroseconds(lastVerticalPulse);
  verticalEsc2.writeMicroseconds(lastVerticalPulse);
}

// Sofortiger Stopp: alle vier ESCs bekommen Neutral.
void stopAllThrusters() {
  sendAllThrusters(neutralPulse, neutralPulse, neutralPulse);
}

// Behandelt den 0/360-Grad-Uebergang.
float calculateYawError(float target, float current) {
  float error = target - current;

  while (error > 180.0) {
    error -= 360.0;
  }

  while (error < -180.0) {
    error += 360.0;
  }

  return error;
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

void readBnoValues() {
  if (!bnoReady || bno == nullptr) {
    return;
  }

  imu::Vector<3> euler = bno->getVector(Adafruit_BNO055::VECTOR_EULER);

  // VECTOR_EULER: X = Heading/Yaw, Y = Roll, Z = Pitch
  yaw = euler.x();
  roll = euler.y();
  pitch = euler.z();
}

void resetBnoValues() {
  roll = 0.0;
  pitch = 0.0;
  yaw = 0.0;
  targetYaw = 0.0;
  yawError = 0.0;
  lastYawCorrection = 0;
}

// Joystick 1 schaltet Stabilize bei kurzem Druck ein/aus.
void updateStabilizeButton() {
  bool reading = digitalRead(joystick1ButtonPin);

  if (reading != lastJoystick1ButtonReading) {
    lastJoystick1DebounceTime = millis();
  }

  if (millis() - lastJoystick1DebounceTime > debounceMs) {
    if (reading != joystick1ButtonState) {
      joystick1ButtonState = reading;

      if (joystick1ButtonState == LOW) {
        if (!bnoReady) {
          stabilizeMode = false;
          Serial.println("Stabilize nicht moeglich: BNO055 nicht erkannt.");
        } else {
          readBnoValues();
          stabilizeMode = !stabilizeMode;

          if (stabilizeMode) {
            targetYaw = yaw;
            yawError = 0.0;
            lastYawCorrection = 0;
            Serial.println("Stabilize Mode: aktiviert");
          } else {
            resetBnoValues();
            Serial.println("Stabilize Mode: deaktiviert");
          }
        }
      }
    }
  }

  lastJoystick1ButtonReading = reading;
}

void printStatus(
    int rawX1,
    int rawY1,
    int rawX2,
    int rawY2,
    int throttle,
    int steering,
    int verticalCommand) {
  Serial.print("rawX1=");
  Serial.print(rawX1);
  Serial.print(" rawY1=");
  Serial.print(rawY1);
  Serial.print(" rawX2=");
  Serial.print(rawX2);
  Serial.print(" rawY2=");
  Serial.print(rawY2);
  Serial.print(" throttle=");
  Serial.print(throttle);
  Serial.print(" steering=");
  Serial.print(steering);
  Serial.print(" verticalCommand=");
  Serial.print(verticalCommand);
  Serial.print(" roll=");
  Serial.print(roll);
  Serial.print(" pitch=");
  Serial.print(pitch);
  Serial.print(" yaw=");
  Serial.print(yaw);
  Serial.print(" stabilize=");
  Serial.print(stabilizeMode ? "an" : "aus");
  Serial.print(" targetYaw=");
  Serial.print(targetYaw);
  Serial.print(" yawError=");
  Serial.print(yawError);
  Serial.print(" yawCorrection=");
  Serial.print(lastYawCorrection);
  Serial.print(" leftHorizontalPulse=");
  Serial.print(lastLeftHorizontalPulse);
  Serial.print(" rightHorizontalPulse=");
  Serial.print(lastRightHorizontalPulse);
  Serial.print(" verticalPulse=");
  Serial.println(lastVerticalPulse);
}

void setup() {
  Serial.begin(9600);

  pinMode(joystick1ButtonPin, INPUT_PULLUP);
  pinMode(joystick2ButtonPin, INPUT_PULLUP);

  leftHorizontalEsc.attach(leftHorizontalEscPin);
  rightHorizontalEsc.attach(rightHorizontalEscPin);
  verticalEsc1.attach(verticalEsc1Pin);
  verticalEsc2.attach(verticalEsc2Pin);

  // Direkt nach dem Attach alle ESCs neutral halten, damit sie sicher initialisieren.
  stopAllThrusters();
  delay(5000);

  Wire.begin();
  Wire.setClock(100000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif

  bnoReady = initBno();
  if (bnoReady) {
    bno->setExtCrystalUse(true);
    delay(1000);
    readBnoValues();
    Serial.println("BNO055 initialisiert.");
  } else {
    Serial.println("FEHLER: BNO055 wurde nicht erkannt. Manuelles Fahren bleibt aktiv.");
  }

  Serial.println("ROV 4-Thruster-Joystick-Test bereit.");
  Serial.println("Joystick 1: VRy = vorwaerts/rueckwaerts, VRx = drehen");
  Serial.println("Joystick 2: VRy = vertikal, VRx wird nur angezeigt");
  Serial.println("Joystick-Button 1 = Stabilize an/aus");
  Serial.println("Joystick-Button 2 = Not-Stopp");
}

void loop() {
  int rawX1 = analogRead(joystick1XPin);
  int rawY1 = analogRead(joystick1YPin);
  int rawX2 = analogRead(joystick2XPin);
  int rawY2 = analogRead(joystick2YPin);

  updateStabilizeButton();
  if (stabilizeMode) {
    readBnoValues();
  } else {
    resetBnoValues();
  }

  bool stopPressed = digitalRead(joystick2ButtonPin) == LOW;

  int throttle = readAxisWithDeadzone(rawY1);
  int steering = readAxisWithDeadzone(rawX1);
  int verticalCommand = readAxisWithDeadzone(rawY2);

  if (stopPressed) {
    throttle = 0;
    steering = 0;
    verticalCommand = 0;
    if (stabilizeMode) {
      Serial.println("Stabilize Mode: deaktiviert durch Not-Stopp");
    }
    stabilizeMode = false;
    resetBnoValues();
    stopAllThrusters();
  } else {
    int yawCorrection = 0;

    if (stabilizeMode && bnoReady) {
      yawError = calculateYawError(targetYaw, yaw);
      yawCorrection = constrain((int)(yawError * yawKp), -maxYawCorrection, maxYawCorrection);
    } else {
      yawError = 0.0;
    }
    lastYawCorrection = yawCorrection;

    int leftHorizontalPulse = neutralPulse + throttle + steering;
    int rightHorizontalPulse = neutralPulse + throttle - steering;
    int verticalPulse = neutralPulse + verticalCommand;

    if (stabilizeMode && bnoReady) {
      leftHorizontalPulse += yawCorrection;
      rightHorizontalPulse -= yawCorrection;
    }

    sendAllThrusters(leftHorizontalPulse, rightHorizontalPulse, verticalPulse);
  }

  unsigned long now = millis();
  if (now - lastStatusTime >= statusIntervalMs) {
    lastStatusTime = now;
    printStatus(rawX1, rawY1, rawX2, rawY2, throttle, steering, verticalCommand);
  }
}
