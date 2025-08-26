/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 5
  #define LEFT_MOTOR_BACKWARD  6
  #define RIGHT_MOTOR_FORWARD  9
  #define LEFT_MOTOR_FORWARD   10
  #define RIGHT_MOTOR_ENABLE 12
  #define LEFT_MOTOR_ENABLE 13
#elif defined(ZKBM1_MOTOR_DRIVER)
  #define DRIVE_PWM_IN1 5
  #define DRIVE_PWM_IN2 6 
  #define STEER_PWM_IN3 9
  #define STEER_PWM_IN4 10
#endif

void initMotorController();
void setMotorSpeed(int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
