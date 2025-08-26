/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

   #ifdef USE_BASE
   
   #ifdef POLOLU_VNH5019
     /* Include the Pololu library */
     #include "DualVNH5019MotorShield.h"
   
     /* Create the motor driver object */
     DualVNH5019MotorShield drive;
     
     /* Wrap the motor driver initialization */
     void initMotorController() {
       drive.init();
     }
   
     /* Wrap the drive motor set speed function */
     void setMotorSpeed(int i, int spd) {
       if (i == LEFT) drive.setM1Speed(spd);
       else drive.setM2Speed(spd);
     }
   
     // A convenience function for setting both motor speeds
     void setMotorSpeeds(int leftSpeed, int rightSpeed) {
       setMotorSpeed(LEFT, leftSpeed);
       setMotorSpeed(RIGHT, rightSpeed);
     }
     

   #elif defined POLOLU_MC33926
     /* Include the Pololu library */
     #include "DualMC33926MotorShield.h"
   
     /* Create the motor driver object */
     DualMC33926MotorShield drive;
     
     /* Wrap the motor driver initialization */
     void initMotorController() {
       drive.init();
     }
   
     /* Wrap the drive motor set speed function */
     void setMotorSpeed(int i, int spd) {
       if (i == LEFT) drive.setM1Speed(spd);
       else drive.setM2Speed(spd);
     }
   
     // A convenience function for setting both motor speeds
     void setMotorSpeeds(int leftSpeed, int rightSpeed) {
       setMotorSpeed(LEFT, leftSpeed);
       setMotorSpeed(RIGHT, rightSpeed);
     }
     

   #elif defined L298_MOTOR_DRIVER
     void initMotorController() {
       digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
       digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
     }
     
     void setMotorSpeed(int i, int spd) {
       unsigned char reverse = 0;
     
       if (spd < 0)
       {
         spd = -spd;
         reverse = 1;
       }
       if (spd > 255)
         spd = 255;
       
       if (i == LEFT) { 
         if      (reverse == 0) { analogWrite(LEFT_MOTOR_FORWARD, spd); analogWrite(LEFT_MOTOR_BACKWARD, 0); }
         else if (reverse == 1) { analogWrite(LEFT_MOTOR_BACKWARD, spd); analogWrite(LEFT_MOTOR_FORWARD, 0); }
       }
       else /*if (i == RIGHT) //no need for condition*/ {
         if      (reverse == 0) { analogWrite(RIGHT_MOTOR_FORWARD, spd); analogWrite(RIGHT_MOTOR_BACKWARD, 0); }
         else if (reverse == 1) { analogWrite(RIGHT_MOTOR_BACKWARD, spd); analogWrite(RIGHT_MOTOR_FORWARD, 0); }
       }
     }
     
     void setMotorSpeeds(int leftSpeed, int rightSpeed) {
       setMotorSpeed(LEFT, leftSpeed);
       setMotorSpeed(RIGHT, rightSpeed);
     }
     

   #elif defined ZKBM1_MOTOR_DRIVER
   
      void initMotorController() {
        pinMode(DRIVE_PWM_IN1, OUTPUT);
        pinMode(DRIVE_PWM_IN2, OUTPUT);
        pinMode(STEER_PWM_IN3, OUTPUT);
        pinMode(STEER_PWM_IN4, OUTPUT);
      }

      void setMotorSpeed(int spd) {
        bool reverse = false;
    
        if (spd < 0)
        {
          spd = -spd;
          reverse = true;
        }
        if (spd > 255)
          spd = 255;

        // Inform encoder driver of direction
        // Pass 0 for stop condition to trigger inertia-aware direction handling
        if (spd == 0) {
          updateEncoderDirection(DRIVE, 0); // Signal stop condition
        } else {
          updateEncoderDirection(DRIVE, reverse ? -1 : 1);
        }

        if (!reverse) {
          analogWrite(DRIVE_PWM_IN1, spd);
          analogWrite(DRIVE_PWM_IN2, 0);
        } else {
          analogWrite(DRIVE_PWM_IN1, 0);
          analogWrite(DRIVE_PWM_IN2, spd);
        }
      }

      void setMotorSpeeds(int leftSpeed, int rightSpeed) {
        // ZKBM1 is a single drive motor system, so we average the speeds
        // or use just the left speed for drive motor control
        int driveSpeed = leftSpeed; // Use left speed as primary drive
        setMotorSpeed(driveSpeed);
      }

      void setSteeringDirection(int target_position) {
        long current_position = readEncoder(STEER);
        long error = target_position - current_position;

        const long tolerance = 8; // Encoder counts tolerance

        if (abs(error) <= tolerance) {
          // Stop steering motor if within tolerance
          analogWrite(STEER_PWM_IN3, 0);
          analogWrite(STEER_PWM_IN4, 0);
          return;
        }

        if (error > 0) {
          // Turn steering right
          updateEncoderDirection(STEER, 1);
          analogWrite(STEER_PWM_IN3, 255);
          analogWrite(STEER_PWM_IN4, 0);
        } else {
          // Turn steering left
          updateEncoderDirection(STEER, -1);
          analogWrite(STEER_PWM_IN3, 0);
          analogWrite(STEER_PWM_IN4, 255);
        }
      }

   
  #elif defined SPARKFUN_TB6612
    /***************************************************************
     TB6612 Motor Driver Implementation
     
     This implementation supports both differential drive (2-motor groups)
     and mecanum drive (4 individual motors) configurations.
     *************************************************************/
    
    // SparkFun TB6612 Library - if not available, we'll use direct pin control
    #ifdef SPARKFUN_TB6612_LIBRARY_AVAILABLE
      #include <SparkFun_TB6612.h>
      #define USE_SPARKFUN_LIBRARY
    #else
      // Direct pin control implementation (no external library required)
      #define USE_DIRECT_PIN_CONTROL
    #endif

    /***************************************************************
     TB6612 Direct Pin Control Implementation
     
     This implementation uses direct pin control without requiring
     the SparkFun TB6612 library, making it more portable.
     *************************************************************/
    
    void initMotorController() {
      // Set all control pins as outputs
      pinMode(L_AIN1, OUTPUT);
      pinMode(L_AIN2, OUTPUT);
      pinMode(L_PWMA, OUTPUT);
      pinMode(L_BIN1, OUTPUT);
      pinMode(L_BIN2, OUTPUT);
      pinMode(L_PWMB, OUTPUT);
      pinMode(L_STBY, OUTPUT);
      
      pinMode(R_AIN1, OUTPUT);
      pinMode(R_AIN2, OUTPUT);
      pinMode(R_PWMA, OUTPUT);
      pinMode(R_BIN1, OUTPUT);
      pinMode(R_BIN2, OUTPUT);
      pinMode(R_PWMB, OUTPUT);
      pinMode(R_STBY, OUTPUT);
      
      // Enable both TB6612 drivers (standby HIGH = enabled)
      digitalWrite(L_STBY, HIGH);
      digitalWrite(R_STBY, HIGH);
    }

    /***************************************************************
     Individual Motor Control Functions
     *************************************************************/
    
    void driveMotor(int ain1, int ain2, int pwm, int speed, int offset, int trim) {
      // Apply direction offset
      speed *= offset;
      
      // Apply trim adjustment
      speed += trim;
      
      // Clamp speed to valid range
      speed = constrain(speed, -PWM_MAX, PWM_MAX);
      
      // Apply deadzone compensation
      if (speed > 0 && speed < MOTOR_DEADZONE) {
        speed = MOTOR_DEADZONE;
      } else if (speed < 0 && speed > -MOTOR_DEADZONE) {
        speed = -MOTOR_DEADZONE;
      }
      
      if (speed > 0) {
        // Forward direction
        digitalWrite(ain1, HIGH);
        digitalWrite(ain2, LOW);
        analogWrite(pwm, speed);
      } else if (speed < 0) {
        // Reverse direction
        digitalWrite(ain1, LOW);
        digitalWrite(ain2, HIGH);
        analogWrite(pwm, -speed);
      } else {
        // Stop motor (brake mode)
        digitalWrite(ain1, LOW);
        digitalWrite(ain2, LOW);
        analogWrite(pwm, 0);
      }
    }

    #ifdef USE_MECANUM
      /***************************************************************
       Mecanum Drive Mode (4 Individual Motors)
       *************************************************************/
      
      void setMecanumMotorSpeeds(int fl, int fr, int rl, int rr) {
        // Drive each motor individually with trim compensation
        driveMotor(L_AIN1, L_AIN2, L_PWMA, fl, OFFSET_L1, TRIM_L1);  // Motor 1 (Front-Left)
        driveMotor(L_BIN1, L_BIN2, L_PWMB, rl, OFFSET_L2, TRIM_L2);  // Motor 2 (Rear-Left)
        driveMotor(R_AIN1, R_AIN2, R_PWMA, fr, OFFSET_R1, TRIM_R1);  // Motor 3 (Front-Right)
        driveMotor(R_BIN1, R_BIN2, R_PWMB, rr, OFFSET_R2, TRIM_R2);  // Motor 4 (Rear-Right)
      }

      void setMotorSpeeds(int leftSpeed, int rightSpeed) {
        // In mecanum mode, treat left/right as front motor speeds
        setMecanumMotorSpeeds(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
      }

      void setMotorSpeed(int spd) {
        // Single speed applies to all motors
        setMecanumMotorSpeeds(spd, spd, spd, spd);
      }

    #else
      /***************************************************************
       Differential Drive Mode (2 Motor Groups)
       *************************************************************/
      
      void setMotorSpeeds(int leftSpeed, int rightSpeed) {
        // Drive left motor group (both motors on left TB6612) with trim compensation
        driveMotor(L_AIN1, L_AIN2, L_PWMA, leftSpeed, OFFSET_L1, TRIM_L1);  // Motor 1
        driveMotor(L_BIN1, L_BIN2, L_PWMB, leftSpeed, OFFSET_L2, TRIM_L2);  // Motor 2
        
        // Drive right motor group (both motors on right TB6612) with trim compensation
        driveMotor(R_AIN1, R_AIN2, R_PWMA, rightSpeed, OFFSET_R1, TRIM_R1);  // Motor 3
        driveMotor(R_BIN1, R_BIN2, R_PWMB, rightSpeed, OFFSET_R2, TRIM_R2);  // Motor 4
      }

      void setMotorSpeed(int spd) {
        // Single speed applies to both sides (straight movement)
        setMotorSpeeds(spd, spd);
      }
      
      void setMecanumMotorSpeeds(int fl, int fr, int rl, int rr) {
        // In differential mode, convert individual wheel commands to left/right groups
        int leftSpeed = (fl + rl) / 2;   // Average of left side motors
        int rightSpeed = (fr + rr) / 2;  // Average of right side motors
        setMotorSpeeds(leftSpeed, rightSpeed);
      }
    #endif
    
 
   #else
     #error A motor driver must be selected!
   #endif
  #endif
