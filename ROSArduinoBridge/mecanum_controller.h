#pragma once
#ifdef USE_MECANUM

// ---- Open-loop (keine Encoder) ----
#ifdef NO_ENCODERS

inline void mecanumOpenLoopPWM(int fl, int fr, int rl, int rr) {
  setMecanumMotorSpeeds(fl, fr, rl, rr);
}

#else
// ---- Closed-loop (Encoder vorhanden) ----
// TODO: vier PID-Strukturen + doPID4(); Konvertierung Twist->Soll-RPM->Ticks/Frame
// Platzhalter, damit der Build läuft:
inline void mecanumDoPID() {
  // Implementieren, sobald 4 Encoder verfügbar sind.
}
#endif

#endif // USE_MECANUM
