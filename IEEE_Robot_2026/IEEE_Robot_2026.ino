#include <Arduino.h>

#define RELAY_PIN A6  // Adjust to your actual relay pin


void setup() {
  Serial.begin(115200);  // USB serial to Pi
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // relay off
}

void loop() {
  if (Serial.available() >= 2) {  // command byte + data byte
    uint8_t cmd = Serial.read();
    uint8_t value = Serial.read();

    switch (cmd) {
      case 0x01:  // Relay
        if (value == 0x00) {
          digitalWrite(RELAY_PIN, LOW);  // OFF (active-low)
          Serial.write(0xAA);
        } else if (value == 0x01) {
          digitalWrite(RELAY_PIN, HIGH);  // ON (active-low)
          Serial.write(0xAA);
        } else {
          Serial.write(0xFF);
        }
        break;


      default:
        Serial.write(0xFF);  // ERROR: unknown command
        break;
    }
  }
}