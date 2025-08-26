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
   #else
     #error A motor driver must be selected!
   #endif
   
   #endif