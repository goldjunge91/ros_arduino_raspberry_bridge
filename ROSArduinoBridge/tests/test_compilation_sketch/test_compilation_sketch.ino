/* Simple compilation test for encoder abstraction layer */

#define USE_BASE
#define NO_ENCODERS
#define SPARKFUN_TB6612

// Include the encoder driver
#include "encoder_driver.h"

void setup() {
  Serial.begin(115200);
  
  // Test encoder abstraction functions
  bool available = encodersAvailable();
  int count = getEncoderCount();
  long enc0 = readEncoder(0);
  long enc1 = readEncoder(1);
  
  resetEncoder(0);
  resetEncoders();
  setEncoderDirection(0, 1);
  
  Serial.println("Encoder abstraction layer compiled successfully!");
  Serial.print("Encoders available: ");
  Serial.println(available ? "true" : "false");
  Serial.print("Encoder count: ");
  Serial.println(count);
}

void loop() {
  // Empty loop
}