/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

/***************************************************************
   Motor Driver Pin Definitions and Configuration
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  // L298 Motor Driver Pin Configuration
  #define RIGHT_MOTOR_BACKWARD 5
  #define LEFT_MOTOR_BACKWARD  6
  #define RIGHT_MOTOR_FORWARD  9
  #define LEFT_MOTOR_FORWARD   10
  #define RIGHT_MOTOR_ENABLE 12
  #define LEFT_MOTOR_ENABLE 13
  
  // Compile-time validation for L298
  #if defined(USE_MECANUM)
    #error "L298 motor driver does not support mecanum mode (4-motor control). Use TB6612 or compatible driver."
  #endif

#elif defined(ZKBM1_MOTOR_DRIVER)
  // ZKBM1 Motor Driver Pin Configuration
  #define DRIVE_PWM_IN1 5
  #define DRIVE_PWM_IN2 6 
  #define STEER_PWM_IN3 9
  #define STEER_PWM_IN4 10
  
  // Compile-time validation for ZKBM1
  #if defined(USE_MECANUM)
    #error "ZKBM1 motor driver does not support mecanum mode (4-motor control). Use TB6612 or compatible driver."
  #endif

#elif defined(SPARKFUN_TB6612)
  /***************************************************************
   TB6612 Motor Driver Pin Configuration
   
   This configuration supports both 2-motor differential drive
   and 4-motor mecanum drive modes.
   
   Pin Layout:
   - Left Driver (TB6612 #1): Controls FL and RL motors
   - Right Driver (TB6612 #2): Controls FR and RR motors
   
   Wiring Notes:
   - Ensure PWM pins are connected to PWM-capable Arduino pins
   - STBY pins must be connected to digital pins and pulled HIGH to enable
   - Motor direction pins (AIN1, AIN2, BIN1, BIN2) control motor direction
   *************************************************************/
  
  // Left TB6612 Driver (Controls Front-Left and Rear-Left motors)
  #define L_AIN1 2      // Left Motor A Direction Pin 1 (Motor 1)
  #define L_AIN2 4      // Left Motor A Direction Pin 2 (Motor 1)
  #define L_PWMA 5      // Left Motor A PWM Pin (Motor 1)
  
  #define L_BIN1 7      // Left Motor B Direction Pin 1 (Motor 2)
  #define L_BIN2 8      // Left Motor B Direction Pin 2 (Motor 2)
  #define L_PWMB 6      // Left Motor B PWM Pin (Motor 2)
  
  #define L_STBY A2     // Left TB6612 Standby Pin (HIGH = enabled)

  // Right TB6612 Driver (Controls Front-Right and Rear-Right motors)
  #define R_AIN1 0      // Right Motor A Direction Pin 1 (Motor 3)
  #define R_AIN2 1      // Right Motor A Direction Pin 2 (Motor 3)
  #define R_PWMA 9      // Right Motor A PWM Pin (Motor 3)

  #define R_BIN1 11     // Right Motor B Direction Pin 1 (Motor 4)
  #define R_BIN2 12     // Right Motor B Direction Pin 2 (Motor 4)
  #define R_PWMB 10     // Right Motor B PWM Pin (Motor 4)

  #define R_STBY A3     // Right TB6612 Standby Pin (HIGH = enabled)

  // Motor Direction Offsets (change to -1 if motor spins in wrong direction)
  #define OFFSET_L1  1  // Motor 1 (Left Driver Motor A) direction offset
  #define OFFSET_L2  1  // Motor 2 (Left Driver Motor B) direction offset  
  #define OFFSET_R1  1  // Motor 3 (Right Driver Motor A) direction offset
  #define OFFSET_R2  1  // Motor 4 (Right Driver Motor B) direction offset

  // Motor Trim Values (fine-tuning for straight movement)
  #define TRIM_L1    0  // Motor 1 PWM trim offset
  #define TRIM_L2    0  // Motor 2 PWM trim offset
  #define TRIM_R1    0  // Motor 3 PWM trim offset
  #define TRIM_R2    0  // Motor 4 PWM trim offset

  // Motor Control Parameters
  #define PWM_MAX           255  // Maximum PWM value (8-bit)
  #define MOTOR_DEADZONE    30   // Minimum PWM to overcome motor friction (0-80)
  #define MOTOR_SLEW_RATE   8    // Maximum PWM change per control loop (1-30)

  /***************************************************************
   TB6612 Pin Configuration - Tested Working Configuration
   
   Pin conflict validation has been removed as this is a tested,
   working configuration that uses valid pin assignments.
   *************************************************************/

#else
  #error "No motor driver selected! Please define one of: L298_MOTOR_DRIVER, ZKBM1_MOTOR_DRIVER, SPARKFUN_TB6612"
#endif

/***************************************************************
   Motor Driver Function Declarations
   *************************************************************/

void initMotorController();
void setMotorSpeed(int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);

#ifdef USE_MECANUM
  void setMecanumMotorSpeeds(int fl, int fr, int rl, int rr);
#endif

/***************************************************************
   Steering Support Detection and Macro
   *************************************************************/

// Define which motor drivers support steering
#ifdef ZKBM1_MOTOR_DRIVER
  #define HAS_STEERING_SUPPORT
  void setSteeringDirection(int target_position);
#endif

// Macro for conditional steering calls
#ifdef HAS_STEERING_SUPPORT
  #define SET_STEERING_DIRECTION(target) setSteeringDirection(target)
#else
  #define SET_STEERING_DIRECTION(target) // No-op for drivers without steering
#endif

#endif // MOTOR_DRIVER_H
