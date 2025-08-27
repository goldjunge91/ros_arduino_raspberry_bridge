/*
 * Test compilation of differential controller with NO_ENCODERS defined
 * This test verifies that the differential controller compiles and works
 * correctly when encoders are not available.
 */

// Define NO_ENCODERS to test encoder-less operation
#define NO_ENCODERS

// Include necessary headers
#include "commands.h"

// Mock encoder functions for NO_ENCODERS mode
#ifdef NO_ENCODERS
long readEncoder(int i) { return 0; }
void resetEncoder(int i) {}
void resetEncoders() {}
void setEncoderDirection(int enc, int dir) {}
bool encodersAvailable() { return false; }
int getEncoderCount() { return 0; }
#endif

// Mock motor driver functions
void setMotorSpeed(int spd) {
  // Mock implementation for testing
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Mock implementation for testing
}

// Define MAX_PWM for testing
#define MAX_PWM 255

// Include the differential controller
#include "diff_controller.h"

void setup() {
  Serial.begin(57600);
  Serial.println("Testing differential controller with NO_ENCODERS");
  
  // Test resetPID function
  resetPID();
  Serial.println("resetPID() - OK");
  
  // Test updatePID function (should call updateDirectDrive)
  moving = 1;
  drivePID.output = 100;
  updatePID();
  Serial.println("updatePID() with moving=1 - OK");
  
  // Test updatePID function with moving=0
  moving = 0;
  updatePID();
  Serial.println("updatePID() with moving=0 - OK");
  
  // Test setDirectDriveSpeed function
  setDirectDriveSpeed(150);
  if (drivePID.output == 150 && moving == 1) {
    Serial.println("setDirectDriveSpeed(150) - OK");
  } else {
    Serial.println("setDirectDriveSpeed(150) - FAILED");
  }
  
  // Test speed clamping
  setDirectDriveSpeed(300);
  if (drivePID.output == MAX_PWM) {
    Serial.println("setDirectDriveSpeed(300) clamping - OK");
  } else {
    Serial.println("setDirectDriveSpeed(300) clamping - FAILED");
  }
  
  // Test negative speed clamping
  setDirectDriveSpeed(-300);
  if (drivePID.output == -MAX_PWM) {
    Serial.println("setDirectDriveSpeed(-300) clamping - OK");
  } else {
    Serial.println("setDirectDriveSpeed(-300) clamping - FAILED");
  }
  
  // Test zero speed
  setDirectDriveSpeed(0);
  if (drivePID.output == 0 && moving == 0) {
    Serial.println("setDirectDriveSpeed(0) - OK");
  } else {
    Serial.println("setDirectDriveSpeed(0) - FAILED");
  }
  
  Serial.println("All tests completed!");
}

void loop() {
  // Empty loop for testing
}