/* *************************************************************
   Encoder driver function definitions - by James Nugen
   Enhanced with encoder abstraction layer for flexible configuration
   ************************************************************ */

// Encoder index constants for compatibility
#ifndef LEFT
  #define LEFT 0
#endif
#ifndef RIGHT  
  #define RIGHT 1
#endif

// Encoder availability check functions
bool encodersAvailable();
int getEncoderCount();

// Core encoder interface functions
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
void setEncoderDirection(int enc, int dir);

// Conditional compilation for encoder hardware
#ifdef NO_ENCODERS
  // When NO_ENCODERS is defined, all encoder functions return safe values
  // This allows the firmware to compile and run without encoder hardware
#else
  // Encoder hardware configuration
  #ifdef ARDUINO_ENC_COUNTER
    //below can be changed, but should be PORTD pins; 
    //otherwise additional changes in the code are required
    #define LEFT_ENC_PIN_A PD2  //pin 2
    #define LEFT_ENC_PIN_B PD3  //pin 3
    
    //below can be changed, but should be PORTC pins
    #define RIGHT_ENC_PIN_A PC4  //pin A4
    #define RIGHT_ENC_PIN_B PC5   //pin A5
  #elif defined(ARDUINO_HC89_COUNTER)
    #define DRIVE_ENC_PIN PD2
    #define STEER_ENC_PIN PD3
  #endif
#endif

