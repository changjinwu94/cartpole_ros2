// UNO R4 Minima. External 5.1k pull-ups to 5V on D2..D6.
// A=D2, B=D3, Z=D4; Light-ON black outputs: left=D5, right=D6.
#include <Arduino.h>
#include <stdint.h>
volatile uint32_t encoderBits = 0;
volatile uint32_t invalidTransitions = 0;
volatile uint8_t previousAB;
uint32_t sequenceNumber = 0;
const int8_t quadrature[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
void encoderChange() {
  uint8_t ab = (digitalRead(2) << 1) | digitalRead(3);
  if ((previousAB ^ ab) == 3) invalidTransitions++;
  encoderBits += quadrature[(previousAB << 2) | ab];
  previousAB = ab;
}
void setup() {
  for (int pin = 2; pin <= 6; pin++) pinMode(pin, INPUT);
  Serial.begin(115200);
  previousAB = (digitalRead(2) << 1) | digitalRead(3);
  attachInterrupt(digitalPinToInterrupt(2), encoderChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), encoderChange, CHANGE);
}
void loop() {
  static uint32_t last = 0;
  uint32_t now = micros();
  if ((uint32_t)(now - last) < 5000) return; // 200Hz, no blocking delay
  last = now;
  noInterrupts();
  int32_t count = (int32_t)encoderBits;
  uint32_t invalid = invalidTransitions;
  interrupts();
  // White Dark-ON wires unused. Black Light-ON: HIGH means blocked OR open wire.
  Serial.print("CP1,"); Serial.print(sequenceNumber++);
  Serial.print(','); Serial.print(now);
  Serial.print(','); Serial.print(count);
  Serial.print(','); Serial.print(digitalRead(5) == HIGH);
  Serial.print(','); Serial.print(digitalRead(6) == HIGH);
  Serial.print(','); Serial.print(digitalRead(4));
  Serial.print(','); Serial.print(invalid);
  Serial.println(",1");
}
