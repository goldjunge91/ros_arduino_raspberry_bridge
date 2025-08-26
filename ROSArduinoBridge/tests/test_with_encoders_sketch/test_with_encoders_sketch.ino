/* Compilation test for encoder abstraction layer WITH encoders enabled */

#define USE_BASE
// #define NO_ENCODERS  // Comment out to enable encoders
#define ARDUINO_ENC_COUNTER  // Enable Arduino encoder counter
#define SPARKFUN_TB6612

// Include required headers
#include "commands.h"
#include "encoder_driver.h"

void setup() {
  Serial.begin(115200);
  
  // Test encoder abstraction functions
  bool available = encodersAvailable();
  int count = getEncoderCount();
  long enc0 = readEncoder(0);
  long enc1 = readEncoder(1);
  long encDrive = readEncoder(DRIVE);
  long encSteer = readEncoder(STEER);
  long encLeft = readEncoder(LEFT);
  long encRight = readEncoder(RIGHT);
  
  resetEncoder(0);
  resetEncoders();
  setEncoderDirection(0, 1);
  
  Serial.println("Encoder abstraction layer compiled successfully WITH encoders!");
  Serial.print("Encoders available: ");
  Serial.println(available ? "true" : "false");
  Serial.print("Encoder count: ");
  Serial.println(count);
  Serial.print("Encoder 0: ");
  Serial.println(enc0);
  Serial.print("Encoder 1: ");
  Serial.println(enc1);
  Serial.print("DRIVE encoder: ");
  Serial.println(encDrive);
  Serial.print("STEER encoder: ");
  Serial.println(encSteer);
  Serial.print("LEFT encoder: ");
  Serial.println(encLeft);
  Serial.print("RIGHT encoder: ");
  Serial.println(encRight);
}

void loop() {
  // Test reading encoders in loop
  if (encodersAvailable()) {
    for (int i = 0; i < getEncoderCount(); i++) {
      long value = readEncoder(i);
      Serial.print("Encoder ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(value);
    }
    delay(1000);
  }
}