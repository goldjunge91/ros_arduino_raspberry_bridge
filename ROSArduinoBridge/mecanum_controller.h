/***************************************************************
   Mecanum Controller - Omnidirectional Drive Control
   
   This module provides mecanum wheel kinematics calculations and
   PID control for 4-wheel omnidirectional robots. It supports both
   encoder-based PID control and direct PWM control modes.
   
   Motor Layout:
   FL (0) ---- FR (1)
   |            |
   |            |
   RL (2) ---- RR (3)
   
   Wheel Numbering:
   0 = Front Left (FL)
   1 = Front Right (FR) 
   2 = Rear Left (RL)
   3 = Rear Right (RR)
   *************************************************************/

#ifndef MECANUM_CONTROLLER_H
#define MECANUM_CONTROLLER_H

/***************************************************************
   Mecanum Wheel PID Structure
   
   Individual PID control structure for each mecanum wheel.
   Based on the existing SetPointInfo structure but adapted
   for 4-wheel independent control.
   *************************************************************/
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count
  int PrevInput;                 // last input (for derivative kick avoidance)
  int ITerm;                     // integrated term
  long output;                   // last motor PWM setting
} MecanumWheelPID;

/***************************************************************
   Mecanum Kinematics Parameters
   
   Physical parameters for mecanum wheel calculations.
   These can be adjusted based on robot dimensions.
   *************************************************************/
typedef struct {
  float wheelRadius;      // Wheel radius in meters (default: 0.05m = 50mm)
  float wheelBase;        // Distance between left and right wheels in meters
  float trackWidth;       // Distance between front and rear wheels in meters
  float maxLinearVel;     // Maximum linear velocity in m/s
  float maxAngularVel;    // Maximum angular velocity in rad/s
} MecanumParams;

/***************************************************************
   Global Variables
   *************************************************************/

// PID control structures for each wheel
extern MecanumWheelPID wheelPID[4];

// Mecanum kinematics parameters (initialized with default values)
extern MecanumParams mecanumParams;

// PID parameters (shared across all wheels)
extern int MecanumKp;
extern int MecanumKd; 
extern int MecanumKi;
extern int MecanumKo;

// Movement state
extern unsigned char mecanumMoving;

/***************************************************************
   Function Declarations
   *************************************************************/

/*
 * Initialize mecanum PID controllers
 * Resets all PID variables to prevent startup spikes
 */
void resetMecanumPID();

/*
 * Main mecanum PID update function
 * Reads encoders and updates all wheel PID controllers
 * Should be called at regular intervals (30Hz recommended)
 */
void updateMecanumPID();

/*
 * Individual wheel PID calculation
 * Computes PID output for a single wheel
 * 
 * @param p Pointer to MecanumWheelPID structure for the wheel
 * @param wheelIndex Index of the wheel (0-3) for encoder reading
 */
void doMecanumPID(MecanumWheelPID * p, int wheelIndex);

/*
 * Convert twist commands to individual wheel speeds
 * Implements mecanum wheel kinematics to convert desired robot
 * velocity (vx, vy, wz) into individual wheel speeds
 * 
 * @param vx Linear velocity in x direction (forward/backward) in m/s
 * @param vy Linear velocity in y direction (left/right) in m/s  
 * @param wz Angular velocity around z axis (rotation) in rad/s
 * @param wheelSpeeds Output array of 4 wheel speeds in PWM units (-255 to 255)
 */
void mecanumTwistToWheels(float vx, float vy, float wz, int* wheelSpeeds);

/*
 * Direct mecanum motor control (encoder-less operation)
 * Updates motor speeds directly without PID control
 * Used when NO_ENCODERS is defined
 */
void updateDirectMecanum();

/*
 * Set target speeds for all mecanum wheels (PID mode)
 * Sets the target ticks per frame for each wheel's PID controller
 * 
 * @param fl Front left wheel target speed (ticks per frame)
 * @param fr Front right wheel target speed (ticks per frame)
 * @param rl Rear left wheel target speed (ticks per frame)
 * @param rr Rear right wheel target speed (ticks per frame)
 */
void setMecanumTargetSpeeds(double fl, double fr, double rl, double rr);

/*
 * Set direct PWM speeds for all mecanum wheels (open-loop mode)
 * Sets motor speeds directly without PID control
 * 
 * @param fl Front left wheel PWM speed (-255 to 255)
 * @param fr Front right wheel PWM speed (-255 to 255)
 * @param rl Rear left wheel PWM speed (-255 to 255)
 * @param rr Rear right wheel PWM speed (-255 to 255)
 */
void setMecanumDirectSpeeds(int fl, int fr, int rl, int rr);

/*
 * Convert wheel speeds from m/s to ticks per frame
 * Utility function for converting physical velocities to encoder units
 * 
 * @param wheelSpeed_ms Wheel speed in meters per second
 * @return Equivalent speed in ticks per frame
 */
double wheelSpeedToTicksPerFrame(float wheelSpeed_ms);

/*
 * Scale wheel speeds proportionally to stay within PWM limits
 * If any wheel speed exceeds MAX_PWM, all speeds are scaled down
 * proportionally to maintain the desired motion direction
 * 
 * @param wheelSpeeds Array of 4 wheel speeds to be scaled in-place
 */
void scaleMecanumSpeeds(int* wheelSpeeds);

/*
 * Initialize mecanum parameters with default values
 * Sets up default robot dimensions and velocity limits
 * Can be called during setup or when parameters need to be reset
 */
void initMecanumParams();

/***************************************************************
   Mecanum Wheel Kinematics Constants
   
   These constants define the kinematic relationships for mecanum
   wheels. The standard mecanum wheel configuration uses these
   coefficients to convert robot velocities to wheel velocities.
   *************************************************************/

// Mecanum wheel kinematic coefficients
// For standard mecanum wheel arrangement (45-degree rollers)
#define MECANUM_FL_VX_COEFF   1.0    // Front left X coefficient
#define MECANUM_FL_VY_COEFF  -1.0    // Front left Y coefficient  
#define MECANUM_FL_WZ_COEFF  -1.0    // Front left rotation coefficient

#define MECANUM_FR_VX_COEFF   1.0    // Front right X coefficient
#define MECANUM_FR_VY_COEFF   1.0    // Front right Y coefficient
#define MECANUM_FR_WZ_COEFF   1.0    // Front right rotation coefficient

#define MECANUM_RL_VX_COEFF   1.0    // Rear left X coefficient
#define MECANUM_RL_VY_COEFF   1.0    // Rear left Y coefficient
#define MECANUM_RL_WZ_COEFF  -1.0    // Rear left rotation coefficient

#define MECANUM_RR_VX_COEFF   1.0    // Rear right X coefficient
#define MECANUM_RR_VY_COEFF  -1.0    // Rear right Y coefficient
#define MECANUM_RR_WZ_COEFF   1.0    // Rear right rotation coefficient

/***************************************************************
   Default Mecanum Parameters
   
   These default values can be overridden by calling initMecanumParams()
   or by directly modifying the mecanumParams structure.
   *************************************************************/

#define DEFAULT_WHEEL_RADIUS     0.05    // 50mm wheels
#define DEFAULT_WHEEL_BASE       0.30    // 300mm between left/right wheels
#define DEFAULT_TRACK_WIDTH      0.25    // 250mm between front/rear wheels  
#define DEFAULT_MAX_LINEAR_VEL   1.0     // 1 m/s maximum linear velocity
#define DEFAULT_MAX_ANGULAR_VEL  2.0     // 2 rad/s maximum angular velocity

/***************************************************************
   Velocity Scaling Constants
   
   Constants for converting between different velocity units
   *************************************************************/

#define VEL_SCALE_FACTOR        100.0    // Scale factor for twist command parsing
#define PWM_TO_VELOCITY_RATIO   0.01     // Approximate PWM to m/s conversion
#define TICKS_PER_METER         1000     // Encoder ticks per meter (adjust for your setup)

#endif // MECANUM_CONTROLLER_H