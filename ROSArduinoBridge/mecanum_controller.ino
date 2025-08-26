/***************************************************************
   Mecanum Controller Implementation
   
   Implementation of mecanum wheel kinematics and PID control
   for omnidirectional robot movement.
   *************************************************************/

#ifdef USE_MECANUM

/***************************************************************
   Global Variable Definitions
   *************************************************************/

// PID control structures for each wheel (FL, FR, RL, RR)
MecanumWheelPID wheelPID[4];

// Mecanum kinematics parameters
MecanumParams mecanumParams;

// PID parameters for mecanum wheels
int MecanumKp = 20;
int MecanumKd = 12;
int MecanumKi = 0;
int MecanumKo = 50;

// Movement state
unsigned char mecanumMoving = 0;

// Current motor speeds for open-loop operation
int currentMecanumSpeeds[4] = {0, 0, 0, 0}; // FL, FR, RL, RR

/***************************************************************
   Mecanum PID Control Functions
   *************************************************************/

/*
 * Initialize mecanum PID controllers
 * Resets all PID variables to prevent startup spikes
 */
void resetMecanumPID() {
  for (int i = 0; i < 4; i++) {
    wheelPID[i].TargetTicksPerFrame = 0.0;
    
    #ifndef NO_ENCODERS
      wheelPID[i].Encoder = readEncoder(i);
      wheelPID[i].PrevEnc = wheelPID[i].Encoder;
    #else
      wheelPID[i].Encoder = 0;
      wheelPID[i].PrevEnc = 0;
    #endif
    
    wheelPID[i].output = 0;
    wheelPID[i].PrevInput = 0;
    wheelPID[i].ITerm = 0;
  }
}

/*
 * Individual wheel PID calculation
 * Based on the existing doPID function but adapted for mecanum wheels
 */
void doMecanumPID(MecanumWheelPID * p, int wheelIndex) {
  long Perror;
  long output;
  int input;

  #ifndef NO_ENCODERS
    // Map mecanum wheel indices to available encoders
    // For 2-encoder systems: FL+RL use LEFT encoder, FR+RR use RIGHT encoder
    // For 4-encoder systems: direct mapping (when available)
    int encoderIndex;
    if (getEncoderCount() >= 4) {
      // Direct mapping for 4-encoder systems
      encoderIndex = wheelIndex;
    } else {
      // Map to 2-encoder system: left wheels (0,2) -> LEFT, right wheels (1,3) -> RIGHT
      encoderIndex = (wheelIndex == 0 || wheelIndex == 2) ? LEFT : RIGHT;
    }
    
    p->Encoder = readEncoder(encoderIndex);
    input = p->Encoder - p->PrevEnc;
  #else
    // In encoder-less mode, assume perfect tracking
    input = (int)p->TargetTicksPerFrame;
    p->Encoder += input;
  #endif

  Perror = p->TargetTicksPerFrame - input;

  // PID calculation with derivative kick avoidance
  output = (MecanumKp * Perror - MecanumKd * (input - p->PrevInput) + p->ITerm) / MecanumKo;
  p->PrevEnc = p->Encoder;

  output += p->output;
  
  // Clamp output to PWM limits and handle integral windup
  if (output >= MAX_PWM) {
    output = MAX_PWM;
  } else if (output <= -MAX_PWM) {
    output = -MAX_PWM;
  } else {
    // Only accumulate integral term if output is not saturated
    p->ITerm += MecanumKi * Perror;
  }

  p->output = output;
  p->PrevInput = input;
}

/*
 * Main mecanum PID update function
 * Updates all four wheel PID controllers and sets motor speeds
 */
void updateMecanumPID() {
  #ifdef NO_ENCODERS
    // In open-loop mode, use direct motor control
    updateDirectMecanum();
    return;
  #endif

  // If not moving, reset PID once to prevent startup spikes
  if (!mecanumMoving) {
    if (wheelPID[0].PrevInput != 0 || wheelPID[1].PrevInput != 0 || 
        wheelPID[2].PrevInput != 0 || wheelPID[3].PrevInput != 0) {
      resetMecanumPID();
    }
    return;
  }

  // Update PID for each wheel
  for (int i = 0; i < 4; i++) {
    doMecanumPID(&wheelPID[i], i);
  }

  // Set motor speeds based on PID outputs
  setMecanumMotorSpeeds(
    (int)wheelPID[0].output,  // Front Left
    (int)wheelPID[1].output,  // Front Right
    (int)wheelPID[2].output,  // Rear Left
    (int)wheelPID[3].output   // Rear Right
  );
}

/***************************************************************
   Mecanum Kinematics Functions
   *************************************************************/

/*
 * Convert twist commands to individual wheel speeds
 * Implements standard mecanum wheel kinematics for open-loop control
 */
void mecanumTwistToWheels(float vx, float vy, float wz, int* wheelSpeeds) {
  // Simplified mecanum kinematics for open-loop operation
  // Input velocities are treated as normalized values (-1.0 to 1.0)
  // and converted directly to PWM values
  
  // Standard mecanum wheel equations for 45-degree rollers
  // Simplified without complex robot dimension calculations
  float robotRadius = 1.0; // Normalized robot radius for rotation
  
  // Calculate wheel velocities using standard mecanum kinematics
  float fl = vx - vy - wz * robotRadius;  // Front Left
  float fr = vx + vy + wz * robotRadius;  // Front Right  
  float rl = vx + vy - wz * robotRadius;  // Rear Left
  float rr = vx - vy + wz * robotRadius;  // Rear Right
  
  // Convert to PWM values (scale by MAX_PWM for full range)
  wheelSpeeds[0] = (int)(fl * MAX_PWM);  // Front Left
  wheelSpeeds[1] = (int)(fr * MAX_PWM);  // Front Right
  wheelSpeeds[2] = (int)(rl * MAX_PWM);  // Rear Left
  wheelSpeeds[3] = (int)(rr * MAX_PWM);  // Rear Right
  
  // Scale speeds proportionally if any exceed limits
  scaleMecanumSpeeds(wheelSpeeds);
}

/*
 * Scale wheel speeds proportionally to stay within PWM limits
 */
void scaleMecanumSpeeds(int* wheelSpeeds) {
  // Find the maximum absolute speed
  int maxSpeed = 0;
  for (int i = 0; i < 4; i++) {
    int absSpeed = abs(wheelSpeeds[i]);
    if (absSpeed > maxSpeed) {
      maxSpeed = absSpeed;
    }
  }
  
  // If maximum speed exceeds PWM limit, scale all speeds down
  if (maxSpeed > MAX_PWM) {
    float scaleFactor = (float)MAX_PWM / (float)maxSpeed;
    for (int i = 0; i < 4; i++) {
      wheelSpeeds[i] = (int)(wheelSpeeds[i] * scaleFactor);
    }
  }
}

/***************************************************************
   Utility Functions
   *************************************************************/

/*
 * Set target speeds for all mecanum wheels (PID mode)
 */
void setMecanumTargetSpeeds(double fl, double fr, double rl, double rr) {
  wheelPID[0].TargetTicksPerFrame = fl;  // Front Left
  wheelPID[1].TargetTicksPerFrame = fr;  // Front Right
  wheelPID[2].TargetTicksPerFrame = rl;  // Rear Left
  wheelPID[3].TargetTicksPerFrame = rr;  // Rear Right
  
  // Set moving flag if any wheel has a non-zero target
  mecanumMoving = (fl != 0 || fr != 0 || rl != 0 || rr != 0) ? 1 : 0;
}

/*
 * Set direct PWM speeds for all mecanum wheels (open-loop mode)
 */
void setMecanumDirectSpeeds(int fl, int fr, int rl, int rr) {
  // Store current speeds
  currentMecanumSpeeds[0] = fl;  // Front Left
  currentMecanumSpeeds[1] = fr;  // Front Right
  currentMecanumSpeeds[2] = rl;  // Rear Left
  currentMecanumSpeeds[3] = rr;  // Rear Right
  
  // Set moving flag if any wheel has a non-zero speed
  mecanumMoving = (fl != 0 || fr != 0 || rl != 0 || rr != 0) ? 1 : 0;
  
  // Apply speeds directly to motors
  setMecanumMotorSpeeds(fl, fr, rl, rr);
}

/*
 * Convert wheel speeds from m/s to ticks per frame
 */
double wheelSpeedToTicksPerFrame(float wheelSpeed_ms) {
  // Convert m/s to ticks per frame
  // This assumes a certain encoder resolution and control loop frequency
  double ticksPerSecond = wheelSpeed_ms * TICKS_PER_METER;
  double ticksPerFrame = ticksPerSecond / PID_RATE;
  return ticksPerFrame;
}

/*
 * Direct mecanum motor control (encoder-less operation)
 */
void updateDirectMecanum() {
  #ifdef NO_ENCODERS
    // In open-loop mode, motor speeds are set directly by commands
    // No PID processing needed - just maintain the last commanded speeds
    
    // If not moving, ensure motors are stopped
    if (!mecanumMoving) {
      setMecanumMotorSpeeds(0, 0, 0, 0);
      return;
    }
    
    // In direct mode, the motor speeds are already set by the command processing
    // This function mainly handles the auto-stop functionality
  #endif
}

/*
 * Initialize mecanum parameters with default values
 */
void initMecanumParams() {
  mecanumParams.wheelRadius = DEFAULT_WHEEL_RADIUS;
  mecanumParams.wheelBase = DEFAULT_WHEEL_BASE;
  mecanumParams.trackWidth = DEFAULT_TRACK_WIDTH;
  mecanumParams.maxLinearVel = DEFAULT_MAX_LINEAR_VEL;
  mecanumParams.maxAngularVel = DEFAULT_MAX_ANGULAR_VEL;
}

#endif // USE_MECANUM