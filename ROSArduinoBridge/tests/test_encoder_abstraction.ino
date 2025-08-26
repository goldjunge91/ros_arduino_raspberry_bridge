/* *************************************************************
   Unit tests for encoder abstraction layer
   
   This file contains test functions to verify the encoder abstraction
   layer works correctly with and without encoders enabled.
   
   To run these tests:
   1. Include this file in your Arduino project
   2. Call runEncoderTests() from your setup() function
   3. Monitor serial output for test results
   ************************************************************ */

#ifdef USE_BASE

void testEncoderAvailability() {
  Serial.println("Testing encoder availability...");
  
  #ifdef NO_ENCODERS
    if (!encodersAvailable()) {
      Serial.println("✓ encodersAvailable() correctly returns false when NO_ENCODERS is defined");
    } else {
      Serial.println("✗ encodersAvailable() should return false when NO_ENCODERS is defined");
    }
    
    if (getEncoderCount() == 0) {
      Serial.println("✓ getEncoderCount() correctly returns 0 when NO_ENCODERS is defined");
    } else {
      Serial.println("✗ getEncoderCount() should return 0 when NO_ENCODERS is defined");
    }
  #else
    if (encodersAvailable()) {
      Serial.println("✓ encodersAvailable() correctly returns true when encoders are enabled");
    } else {
      Serial.println("✗ encodersAvailable() should return true when encoders are enabled");
    }
    
    if (getEncoderCount() > 0) {
      Serial.println("✓ getEncoderCount() correctly returns > 0 when encoders are enabled");
      Serial.print("  Encoder count: ");
      Serial.println(getEncoderCount());
    } else {
      Serial.println("✗ getEncoderCount() should return > 0 when encoders are enabled");
    }
  #endif
}

void testEncoderReading() {
  Serial.println("Testing encoder reading...");
  
  // Test reading from valid encoder indices
  long enc0 = readEncoder(0);
  long enc1 = readEncoder(1);
  
  #ifdef NO_ENCODERS
    if (enc0 == 0 && enc1 == 0) {
      Serial.println("✓ readEncoder() correctly returns 0 for all indices when NO_ENCODERS is defined");
    } else {
      Serial.println("✗ readEncoder() should return 0 for all indices when NO_ENCODERS is defined");
    }
  #else
    Serial.print("  Encoder 0 value: ");
    Serial.println(enc0);
    Serial.print("  Encoder 1 value: ");
    Serial.println(enc1);
    Serial.println("✓ readEncoder() returns values when encoders are enabled");
  #endif
  
  // Test reading from invalid encoder index
  long encInvalid = readEncoder(99);
  if (encInvalid == 0) {
    Serial.println("✓ readEncoder() correctly returns 0 for invalid encoder index");
  } else {
    Serial.println("✗ readEncoder() should return 0 for invalid encoder index");
  }
}

void testEncoderReset() {
  Serial.println("Testing encoder reset...");
  
  // Test individual encoder reset
  resetEncoder(0);
  resetEncoder(1);
  
  // Test reset all encoders
  resetEncoders();
  
  #ifdef NO_ENCODERS
    Serial.println("✓ Encoder reset functions execute without error when NO_ENCODERS is defined");
  #else
    // After reset, encoder values should be 0 (or close to 0)
    long enc0_after = readEncoder(0);
    long enc1_after = readEncoder(1);
    
    Serial.print("  Encoder 0 after reset: ");
    Serial.println(enc0_after);
    Serial.print("  Encoder 1 after reset: ");
    Serial.println(enc1_after);
    Serial.println("✓ Encoder reset functions execute when encoders are enabled");
  #endif
}

void testEncoderDirection() {
  Serial.println("Testing encoder direction setting...");
  
  // Test setting encoder direction
  setEncoderDirection(0, 1);
  setEncoderDirection(1, -1);
  setEncoderDirection(0, 0); // Invalid direction
  
  #ifdef NO_ENCODERS
    Serial.println("✓ setEncoderDirection() executes without error when NO_ENCODERS is defined");
  #else
    Serial.println("✓ setEncoderDirection() executes when encoders are enabled");
  #endif
}

void testEncoderIndexCompatibility() {
  Serial.println("Testing encoder index compatibility...");
  
  // Test that both DRIVE/STEER and LEFT/RIGHT indexing work
  long driveValue = readEncoder(DRIVE);
  long leftValue = readEncoder(LEFT);
  long steerValue = readEncoder(STEER);
  long rightValue = readEncoder(RIGHT);
  
  #ifdef NO_ENCODERS
    if (driveValue == 0 && leftValue == 0 && steerValue == 0 && rightValue == 0) {
      Serial.println("✓ Both DRIVE/STEER and LEFT/RIGHT indexing return 0 when NO_ENCODERS is defined");
    } else {
      Serial.println("✗ All encoder indices should return 0 when NO_ENCODERS is defined");
    }
  #else
    Serial.print("  DRIVE/LEFT value: ");
    Serial.println(driveValue);
    Serial.print("  STEER/RIGHT value: ");
    Serial.println(steerValue);
    
    // For most encoder implementations, DRIVE should equal LEFT and STEER should equal RIGHT
    if (driveValue == leftValue && steerValue == rightValue) {
      Serial.println("✓ DRIVE/STEER and LEFT/RIGHT indexing are compatible");
    } else {
      Serial.println("! DRIVE/STEER and LEFT/RIGHT indexing may use different mappings");
    }
  #endif
}

void runEncoderTests() {
  Serial.println("=== Encoder Abstraction Layer Tests ===");
  Serial.println();
  
  testEncoderAvailability();
  Serial.println();
  
  testEncoderReading();
  Serial.println();
  
  testEncoderReset();
  Serial.println();
  
  testEncoderDirection();
  Serial.println();
  
  testEncoderIndexCompatibility();
  Serial.println();
  
  Serial.println("=== Encoder Tests Complete ===");
  Serial.println();
}

#endif // USE_BASE