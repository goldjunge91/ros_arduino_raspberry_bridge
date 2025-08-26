/* *************************************************************
   Encoder abstraction layer test runner header
   
   Include this header to run encoder abstraction tests
   ************************************************************ */

#ifndef ENCODER_TEST_RUNNER_H
#define ENCODER_TEST_RUNNER_H

#ifdef USE_BASE
  // Function to run all encoder abstraction tests
  void runEncoderTests();
  
  // Individual test functions (can be called separately if needed)
  void testEncoderAvailability();
  void testEncoderReading();
  void testEncoderReset();
  void testEncoderDirection();
  void testEncoderIndexCompatibility();
#endif

#endif // ENCODER_TEST_RUNNER_H