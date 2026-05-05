#include <Arduino.h>
#include <Servo.h>

const byte leftHorizontalEscPin = 9;
const byte rightHorizontalEscPin = 10;
const byte verticalEsc1Pin = 11;
const byte verticalEsc2Pin = 12;

const byte joystick1XPin = A0;
const byte joystick1YPin = A1;
const byte joystick1ButtonPin = 2;

const byte joystick2XPin = A2;
const byte joystick2YPin = A3;
const byte joystick2ButtonPin = 3;

const int neutralPulse = 1500;
const int minPulse = 1400;
const int maxPulse = 1600;
const int joystickCenter = 512;
const int joystickDeadzone = 60;
const int maxOutputOffset = 100;
const unsigned long statusIntervalMs = 300;

Servo leftHorizontalEsc;
Servo rightHorizontalEsc;
Servo verticalEsc1;
Servo verticalEsc2;

unsigned long lastStatusTime = 0;
int lastLeftHorizontalPulse = neutralPulse;
int lastRightHorizontalPulse = neutralPulse;
int lastVerticalPulse = neutralPulse;

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

  Serial.println("ROV 4-Thruster-Joystick-Test bereit.");
  Serial.println("Joystick 1: VRy = vorwaerts/rueckwaerts, VRx = drehen");
  Serial.println("Joystick 2: VRy = vertikal, VRx wird nur angezeigt");
  Serial.println("Joystick-Button 1 oder 2 = sofortiger Stopp");
}

void loop() {
  int rawX1 = analogRead(joystick1XPin);
  int rawY1 = analogRead(joystick1YPin);
  int rawX2 = analogRead(joystick2XPin);
  int rawY2 = analogRead(joystick2YPin);

  bool stopPressed = digitalRead(joystick1ButtonPin) == LOW || digitalRead(joystick2ButtonPin) == LOW;

  int throttle = readAxisWithDeadzone(rawY1);
  int steering = readAxisWithDeadzone(rawX1);
  int verticalCommand = readAxisWithDeadzone(rawY2);

  if (stopPressed) {
    throttle = 0;
    steering = 0;
    verticalCommand = 0;
    stopAllThrusters();
  } else {
    int leftHorizontalPulse = neutralPulse + throttle + steering;
    int rightHorizontalPulse = neutralPulse + throttle - steering;
    int verticalPulse = neutralPulse + verticalCommand;

    sendAllThrusters(leftHorizontalPulse, rightHorizontalPulse, verticalPulse);
  }

  unsigned long now = millis();
  if (now - lastStatusTime >= statusIntervalMs) {
    lastStatusTime = now;
    printStatus(rawX1, rawY1, rawX2, rawY2, throttle, steering, verticalCommand);
  }
}
