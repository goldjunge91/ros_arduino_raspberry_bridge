/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  int PrevInput;                // last input
  //int PrevErr;                   // last error

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //int Ierror;
  int ITerm;                    //integrated term

  long output;                    // last motor setting
}
SetPointInfo;

SetPointInfo drivePID;

/* PID Parameters */
int Kp = 20;
int Kd = 12;
int Ki = 0;
int Ko = 50;

unsigned char moving = 0; // is the base in motion?

#ifdef NO_ENCODERS
// Forward declarations for encoder-less operation functions
void updateDirectDrive();
void setDirectDriveSpeed(int speed);
#endif

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(){
   drivePID.TargetTicksPerFrame = 0.0;
   #ifndef NO_ENCODERS
   drivePID.Encoder = readEncoder(DRIVE);
   drivePID.PrevEnc = drivePID.Encoder;
   #else
   drivePID.Encoder = 0;
   drivePID.PrevEnc = 0;
   #endif
   drivePID.output = 0;
   drivePID.PrevInput = 0;
   drivePID.ITerm = 0;
}

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
  #ifndef NO_ENCODERS
  long Perror;
  long output;
  int input;

  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;


  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;
  output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
    p->ITerm += Ki * Perror;

  p->output = output;
  p->PrevInput = input;
  #else
  // When encoders are not available, PID control is disabled
  // This function should not be called in NO_ENCODERS mode
  p->output = 0;
  #endif
}

/* Read the encoder values and call the PID routine */
void updatePID() {
  #ifndef NO_ENCODERS
  /* Read the encoders */
  drivePID.Encoder = readEncoder(DRIVE);
  
  /* If we're not moving there is nothing more to do */
  if (!moving){
    /*
    * Reset PIDs once, to prevent startup spikes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    * PrevInput is considered a good proxy to detect
    * whether reset has already happened
    */
    if (drivePID.PrevInput != 0) resetPID();
    return;
  }

  /* Compute PID update for each motor */
  doPID(&drivePID);

  /* Set the motor speed accordingly */
  setMotorSpeed(drivePID.output);
  #else
  // When encoders are not available, use direct drive mode
  updateDirectDrive();
  #endif
}

#ifdef NO_ENCODERS
/*
* Direct drive update function for encoder-less operation
* This function maintains motor commands without PID feedback control
* It handles auto-stop functionality and direct PWM motor control
*/
void updateDirectDrive() {
  /* If we're not moving there is nothing more to do */
  if (!moving){
    /* Ensure motors are stopped */
    if (drivePID.output != 0) {
      drivePID.output = 0;
      setMotorSpeed(0);
    }
    return;
  }

  /* In direct drive mode, the output is set directly by motor commands */
  /* The drivePID.output holds the last commanded motor speed */
  setMotorSpeed(drivePID.output);
}

/*
* Set direct motor speed for encoder-less operation
* This function bypasses PID control and sets motor speed directly
* Used when NO_ENCODERS is defined and direct motor control is needed
*/
void setDirectDriveSpeed(int speed) {
  /* Clamp speed to valid PWM range */
  if (speed > MAX_PWM) speed = MAX_PWM;
  else if (speed < -MAX_PWM) speed = -MAX_PWM;
  
  /* Store the commanded speed in the PID structure for consistency */
  drivePID.output = speed;
  
  /* Set moving flag based on speed */
  moving = (speed != 0) ? 1 : 0;
  
  /* Apply the motor speed immediately */
  setMotorSpeed(speed);
}
#endif

