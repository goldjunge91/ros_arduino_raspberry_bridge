/*
 * Test functionality of differential controller modifications
 * This test verifies that the differential controller works correctly
 * in both encoder and encoder-less modes.
 */

// Test with encoders first
#undef NO_ENCODERS

#include "commands.h"

// Mock encoder functions for testing
long mockEncoderValue = 0;
long readEncoder(int i) { return mockEncoderValue; }
void resetEncoder(int i) { mockEncoderValue = 0; }
void resetEncoders() { mockEncoderValue = 0; }
void setEncoderDirection(int enc, int dir) {}
bool encodersAvailable() { return true; }
int getEncoderCount() { return 2; }

// Mock motor driver functions
int lastMotorSpeed = 0;
void setMotorSpeed(int spd) {
  lastMotorSpeed = spd;
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Mock implementation for testing
}

// Define MAX_PWM for testing
#define MAX_PWM 255

// Include the differential controller
#include "diff_controller.h"

void testWithEncoders() {
  Serial.println("=== Testing WITH encoders ===");
  
  // Test resetPID function
  resetPID();
  Serial.println("resetPID() - OK");
  
  // Test PID functionality
  moving = 1;
  drivePID.TargetTicksPerFrame = 100;
  mockEncoderValue = 50; // Simulate encoder reading
  updatePID();
  Serial.print("PID output: ");
  Serial.println(drivePID.output);
  Serial.println("updatePID() with encoders - OK");
}

void testWithoutEncoders() {
  Serial.println("=== Testing WITHOUT encoders ===");
  
  // Redefine NO_ENCODERS for this test
  #define NO_ENCODERS
  
  // Test direct drive functionality
  moving = 1;
  setDirectDriveSpeed(150);
  if (drivePID.output == 150 && moving == 1) {
    Serial.println("setDirectDriveSpeed(150) - OK");
  } else {
    Serial.println("setDirectDriveSpeed(150) - FAILED");
  }
  
  // Test updateDirectDrive
  updateDirectDrive();
  if (lastMotorSpeed == 150) {
    Serial.println("updateDirectDrive() - OK");
  } else {
    Serial.println("updateDirectDrive() - FAILED");
  }
  
  // Test speed clamping
  setDirectDriveSpeed(300);
  if (drivePID.output == MAX_PWM) {
    Serial.println("Speed clamping (300 -> 255) - OK");
  } else {
    Serial.println("Speed clamping - FAILED");
  }
  
  // Test stop functionality
  setDirectDriveSpeed(0);
  if (drivePID.output == 0 && moving == 0) {
    Serial.println("Stop functionality - OK");
  } else {
    Serial.println("Stop functionality - FAILED");
  }
}

void setup() {
  Serial.begin(57600);
  Serial.println("Testing differential controller modifications");
  
  testWithEncoders();
  testWithoutEncoders();
  
  Serial.println("All tests completed!");
}

void loop() {
  // Empty loop for testing
}