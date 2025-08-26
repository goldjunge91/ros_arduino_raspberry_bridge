/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
// Driving motors encoder drivers: 
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
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
void setEncoderDirection(int enc, int dir);

