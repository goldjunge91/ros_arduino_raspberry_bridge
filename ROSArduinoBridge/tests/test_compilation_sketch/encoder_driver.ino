/* *************************************************************
   Encoder definitions with abstraction layer
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   Enhanced with encoder abstraction layer for flexible configuration
   ************************************************************ */
   
#ifdef USE_BASE

// Encoder abstraction layer implementation
#ifdef NO_ENCODERS
  // When NO_ENCODERS is defined, provide stub functions that return safe values
  
  bool encodersAvailable() {
    return false;
  }
  
  int getEncoderCount() {
    return 0;
  }
  
  long readEncoder(int i) {
    // Return 0 for any encoder index when encoders are disabled
    return 0L;
  }
  
  void resetEncoder(int i) {
    // No-op when encoders are disabled
  }
  
  void resetEncoders() {
    // No-op when encoders are disabled
  }
  
  void setEncoderDirection(int enc, int dir) {
    // No-op when encoders are disabled
  }



#else
  // Encoders are enabled - provide full functionality
  
  bool encodersAvailable() {
    return true;
  }
  
  #ifdef ROBOGAIA
    /* The Robogaia Mega Encoder shield */
    #include "MegaEncoderCounter.h"

    /* Create the encoder shield object */
    MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
    
    int getEncoderCount() {
      return 2; // Robogaia supports 2 encoders (LEFT/RIGHT or DRIVE/STEER)
    }
    
    /* Wrap the encoder reading function */
    long readEncoder(int i) {
      // Support both LEFT/RIGHT (0/1) and DRIVE/STEER (0/1) indexing
      if (i == LEFT || i == DRIVE) return encoders.YAxisGetCount();
      else if (i == RIGHT || i == STEER) return encoders.XAxisGetCount();
      else return 0L; // Invalid encoder index
    }

    /* Wrap the encoder reset function */
    void resetEncoder(int i) {
      if (i == LEFT || i == DRIVE) encoders.YAxisReset();
      else if (i == RIGHT || i == STEER) encoders.XAxisReset();
    }
    
    void setEncoderDirection(int enc, int dir) {
      // Robogaia encoder direction is typically handled in hardware
      // This is a no-op for this encoder type
    }
  #elif defined(ARDUINO_ENC_COUNTER)
    volatile long left_enc_pos = 0L;
    volatile long right_enc_pos = 0L;
    static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table
    
    int getEncoderCount() {
      return 2; // Arduino encoder counter supports 2 encoders
    }
      
    /* Interrupt routine for LEFT encoder, taking care of actual counting */
    ISR (PCINT2_vect){
      static uint8_t enc_last=0;
          
    enc_last <<=2; //shift previous state two places
    enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits
    
      left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
    }
    
    /* Interrupt routine for RIGHT encoder, taking care of actual counting */
    ISR (PCINT1_vect){
          static uint8_t enc_last=0;
              
    enc_last <<=2; //shift previous state two places
    enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits
    
      right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
    }
    
    /* Wrap the encoder reading function */
    long readEncoder(int i) {
      // Support both LEFT/RIGHT (0/1) and DRIVE/STEER (0/1) indexing
      if (i == LEFT || i == DRIVE) return left_enc_pos;
      else if (i == RIGHT || i == STEER) return right_enc_pos;
      else return 0L; // Invalid encoder index
    }

    /* Wrap the encoder reset function */
    void resetEncoder(int i) {
      if (i == LEFT || i == DRIVE){
        left_enc_pos = 0L;
      } else if (i == RIGHT || i == STEER) { 
        right_enc_pos = 0L;
      }
    }
    
    void setEncoderDirection(int enc, int dir) {
      // Arduino encoder counter direction is handled by wiring
      // This is a no-op for this encoder type
    }
  #elif defined(ARDUINO_HC89_COUNTER)
    volatile long enc_count[2] = {0L, 0L}; // DRIVE = 0, STEER = 1
    volatile int enc_direction[2] = {1, 1}; // +1 = forward/right, -1 = reverse/left
    
    // Inertia detection variables
    volatile long last_encoder_pos[2] = {0L, 0L}; // Previous encoder positions
    volatile unsigned long last_direction_change[2] = {0L, 0L}; // Timestamp of last direction change
    volatile int motor_command_direction[2] = {1, 1}; // Last commanded motor direction
    const unsigned long INERTIA_DELAY = 500; // 500ms delay before allowing direction change
    
    int getEncoderCount() {
      return 2; // HC89 counter supports 2 encoders (DRIVE/STEER)
    }
    
    void initEncoders() {
      pinMode(DRIVE_ENC_PIN, INPUT_PULLUP);
      pinMode(STEER_ENC_PIN, INPUT_PULLUP);

      attachInterrupt(digitalPinToInterrupt(DRIVE_ENC_PIN), driveISR, FALLING); // or RISING, CHANGE
      attachInterrupt(digitalPinToInterrupt(STEER_ENC_PIN), steerISR, FALLING);
    }

    void updateEncoderDirection(int enc, int dir) {
      if (enc == DRIVE || enc == STEER) {
        // Store the commanded direction
        motor_command_direction[enc] = dir;
        
        // Only change encoder direction if:
        // 1. The motor is actually moving (dir != 0)
        // 2. OR enough time has passed since the last direction change (inertia delay)
        // 3. OR the direction change is significant (different from current)
        unsigned long current_time = millis();
        
        if (dir != 0) {
          // Motor is actively commanded - update direction immediately
          enc_direction[enc] = dir;
          last_direction_change[enc] = current_time;
        } else {
          // Motor stopped - only change direction after inertia delay
          // and if the actual wheel movement suggests a real direction change
          if ((current_time - last_direction_change[enc]) > INERTIA_DELAY) {
            // Check if wheel is still moving in the previous direction
            long current_pos = readEncoder(enc);
            long delta = current_pos - last_encoder_pos[enc];
            
            // If wheel is still moving significantly in the previous direction,
            // maintain that direction. Otherwise, allow direction change.
            if (abs(delta) < 5) { // Small threshold to detect stopped wheels
              // Wheel has stopped, safe to change direction
              enc_direction[enc] = motor_command_direction[enc];
            }
            // If wheel is still moving, keep the current direction
          }
        }
      }
    }

    void driveISR() {
      // Update last position for inertia detection
      last_encoder_pos[DRIVE] = enc_count[DRIVE];
      
      // Add the count in the current direction
      enc_count[DRIVE] += enc_direction[DRIVE];
    }

    void steerISR() {
      // Update last position for inertia detection
      last_encoder_pos[STEER] = enc_count[STEER];
      
      // Add the count in the current direction
      enc_count[STEER] += enc_direction[STEER];
    }

    long readEncoder(int i) {
      // Support both DRIVE/STEER and LEFT/RIGHT indexing
      if (i == DRIVE || i == LEFT) return enc_count[DRIVE];
      else if (i == STEER || i == RIGHT) return enc_count[STEER];
      else return 0L; // Invalid encoder index
    }

    void resetEncoder(int i) {
      if (i == DRIVE || i == LEFT) {
        enc_count[DRIVE] = 0L;
        last_encoder_pos[DRIVE] = 0L;
        last_direction_change[DRIVE] = 0L;
        motor_command_direction[DRIVE] = 1; // Reset to default forward
      } else if (i == STEER || i == RIGHT) {
        enc_count[STEER] = 0L;
        last_encoder_pos[STEER] = 0L;
        last_direction_change[STEER] = 0L;
        motor_command_direction[STEER] = 1; // Reset to default right
      }
    }
    
    // Function to manually set encoder direction (useful for debugging)
    void setEncoderDirection(int enc, int dir) {
      if ((enc == DRIVE || enc == LEFT) || (enc == STEER || enc == RIGHT)) {
        if (dir == 1 || dir == -1) {
          int actualEnc = (enc == LEFT) ? DRIVE : ((enc == RIGHT) ? STEER : enc);
          enc_direction[actualEnc] = dir;
          motor_command_direction[actualEnc] = dir;
          last_direction_change[actualEnc] = millis();
        }
      }
    }
  #else
    #error An encoder driver must be selected when NO_ENCODERS is not defined!
  #endif

  void resetEncoders() {
    // Reset all available encoders
    for (int i = 0; i < getEncoderCount(); i++) {
      resetEncoder(i);
    }
  }

#endif // End of encoder abstraction layer

#endif
