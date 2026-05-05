# Arduino ROV-Steuerung

Dieses Projekt enthaelt Arduino-Code fuer eine Arduino-basierte ROV-Steuerung.

## Hardware

- Board: Arduino Mega 2560
- Aktueller Port: COM9

## Upload

Der Upload erfolgt ueber VS Code Tasks in `.vscode/tasks.json`.

Verfuegbare Upload-Tasks:

- `Upload Arduino` fuer den Blink-Test
- `Upload ESC Test` fuer den ESC/Thruster-Test

## Vorhandene Tests

- `Tests/Blink_Test/Blink_Test.ino`
- `Tests/ESC_Test/ESC_Test.ino`

## Sicherheit

Thruster/ESC nie unkontrolliert mit montiertem Propeller testen. Fuer erste Tests den Propeller entfernen oder den Thruster sicher fixieren.
