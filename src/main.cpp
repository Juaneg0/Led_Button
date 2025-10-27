#include <Arduino.h>

#define LED_PIN 13
#define SWITCH_PIN 15

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SWITCH_PIN, INPUT);
}

void loop() {
  if (digitalRead(SWITCH_PIN) == HIGH) {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("LED ON");
  } else {
    digitalWrite(LED_PIN, LOW);
    Serial.println("LED OFF");
  }
}

