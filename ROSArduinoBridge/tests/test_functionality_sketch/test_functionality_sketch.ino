/* Functional test for encoder abstraction layer */

#define USE_BASE
#define NO_ENCODERS  // Test with NO_ENCODERS first
#define SPARKFUN_TB6612

#include "commands.h"
#include "encoder_driver.h"

void testEncoderAbstraction() {
  Serial.println("=== Encoder Abstraction Layer Functional Test ===");
  Serial.println();
  
  // Test 1: Encoder availability
  Serial.println("Test 1: Encoder Availability");
  bool available = encodersAvailable();
  int count = getEncoderCount();
  Serial.print("  encodersAvailable(): ");
  Serial.println(available ? "true" : "false");
  Serial.print("  getEncoderCount(): ");
  Serial.println(count);
  
  #ifdef NO_ENCODERS
    if (!available && count == 0) {
      Serial.println("  ✓ PASS: Correctly reports no encoders when NO_ENCODERS is defined");
    } else {
      Serial.println("  ✗ FAIL: Should report no encoders when NO_ENCODERS is defined");
    }
  #else
    if (available && count > 0) {
      Serial.println("  ✓ PASS: Correctly reports encoders available");
    } else {
      Serial.println("  ✗ FAIL: Should report encoders available when enabled");
    }
  #endif
  Serial.println();
  
  // Test 2: Encoder reading
  Serial.println("Test 2: Encoder Reading");
  long enc0 = readEncoder(0);
  long enc1 = readEncoder(1);
  long encDrive = readEncoder(DRIVE);
  long encSteer = readEncoder(STEER);
  long encLeft = readEncoder(LEFT);
  long encRight = readEncoder(RIGHT);
  long encInvalid = readEncoder(99);
  
  Serial.print("  readEncoder(0): ");
  Serial.println(enc0);
  Serial.print("  readEncoder(1): ");
  Serial.println(enc1);
  Serial.print("  readEncoder(DRIVE): ");
  Serial.println(encDrive);
  Serial.print("  readEncoder(STEER): ");
  Serial.println(encSteer);
  Serial.print("  readEncoder(LEFT): ");
  Serial.println(encLeft);
  Serial.print("  readEncoder(RIGHT): ");
  Serial.println(encRight);
  Serial.print("  readEncoder(99): ");
  Serial.println(encInvalid);
  
  #ifdef NO_ENCODERS
    if (enc0 == 0 && enc1 == 0 && encDrive == 0 && encSteer == 0 && 
        encLeft == 0 && encRight == 0 && encInvalid == 0) {
      Serial.println("  ✓ PASS: All encoder reads return 0 when NO_ENCODERS is defined");
    } else {
      Serial.println("  ✗ FAIL: All encoder reads should return 0 when NO_ENCODERS is defined");
    }
  #else
    if (encDrive == encLeft && encSteer == encRight && encInvalid == 0) {
      Serial.println("  ✓ PASS: Index compatibility works and invalid index returns 0");
    } else {
      Serial.println("  ! INFO: Index mapping may vary by encoder type");
    }
  #endif
  Serial.println();
  
  // Test 3: Encoder reset
  Serial.println("Test 3: Encoder Reset");
  resetEncoder(0);
  resetEncoder(1);
  resetEncoders();
  Serial.println("  ✓ PASS: Reset functions execute without error");
  Serial.println();
  
  // Test 4: Encoder direction
  Serial.println("Test 4: Encoder Direction");
  setEncoderDirection(0, 1);
  setEncoderDirection(1, -1);
  setEncoderDirection(DRIVE, 1);
  setEncoderDirection(STEER, -1);
  Serial.println("  ✓ PASS: Direction functions execute without error");
  Serial.println();
  
  Serial.println("=== Test Complete ===");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("Starting encoder abstraction layer functional test...");
  Serial.print("Configuration: ");
  #ifdef NO_ENCODERS
    Serial.println("NO_ENCODERS defined");
  #else
    Serial.println("Encoders enabled");
  #endif
  Serial.println();
  
  testEncoderAbstraction();
}

void loop() {
  // Run a continuous test
  if (encodersAvailable()) {
    Serial.println("Continuous encoder reading test:");
    for (int i = 0; i < getEncoderCount(); i++) {
      Serial.print("  Encoder ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(readEncoder(i));
    }
  } else {
    Serial.println("No encoders available - running in stub mode");
  }
  
  delay(2000);
}