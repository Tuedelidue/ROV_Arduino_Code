#include <Servo.h>

const byte escPin = 9;

// Keep all test pulses close to neutral. This sketch has no full-throttle command.
const int neutralPulse = 1500;
const int forwardPulse = 1600;
const int reversePulse = 1400;

Servo esc;

void sendPulse(int pulse) {
  esc.writeMicroseconds(pulse);
  Serial.print("ESC signal: ");
  Serial.print(pulse);
  Serial.println(" us");
}

void setup() {
  Serial.begin(9600);
  esc.attach(escPin);

  Serial.println("ESC single-thruster test");
  Serial.println("Sending neutral for 5 seconds...");
  // Give the ESC a stable neutral signal before accepting manual commands.
  sendPulse(neutralPulse);
  delay(5000);

  Serial.println("Commands:");
  Serial.println("n = neutral 1500 us");
  Serial.println("f = slow forward 1600 us");
  Serial.println("r = slow reverse 1400 us");
  Serial.println("s = stop/neutral 1500 us");
}

void loop() {
  if (Serial.available() <= 0) {
    return;
  }

  char command = Serial.read();

  switch (command) {
    case 'n':
      sendPulse(neutralPulse);
      break;
    case 'f':
      sendPulse(forwardPulse);
      break;
    case 'r':
      sendPulse(reversePulse);
      break;
    case 's':
      sendPulse(neutralPulse);
      break;
    case '\n':
    case '\r':
      break;
    default:
      Serial.print("Unknown command: ");
      Serial.println(command);
      break;
  }
}
