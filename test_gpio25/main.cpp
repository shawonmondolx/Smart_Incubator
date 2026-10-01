#include <Arduino.h>

int counter = 0;

void setup() {
  Serial.begin(115200);
  pinMode(25, OUTPUT);
  Serial.println("ESP32 Formatted and Reset. Starting counter...");
}

void loop() {
  digitalWrite(25, HIGH);
  delay(500);
  digitalWrite(25, LOW);
  delay(500);
  counter++;
  Serial.printf("Counter: %d\n", counter);
}
