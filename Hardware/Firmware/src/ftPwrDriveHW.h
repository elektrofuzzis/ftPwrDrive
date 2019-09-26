#ifndef ftPwrDriveHW_h
#define ftPwrDriveHW_h

// Hardware defintions for ftPwrDrive
//
// 13.04.19 / V1.00
//
// (C) 2019 Christian Bergschneider & Stefan Fuss

#define MS1    8  // D8
#define MS2    6  // D6
#define MS3    4  // D4

#define MREFI  A5 // A5

#define SERVO1 13 // D13
#define SERVO2 11 // D11
#define SERVO3 10 // D10
#define SERVO4 5  // D5

#define M1STEP 1  // D1
#define M2STEP 12 // D12
#define M3STEP 7  // D7
#define M4STEP 0  // D0

#define ES1    A3  // A3
#define ES2    A2  // A2
#define ES3    A1  // A1
#define ES4    A0  // A0
#define EMS    A4  // A4

#define LED    9   // D9

#define M1EN   0   // QA
#define M2EN   1   // QB
#define M3EN   2   // QC
#define M4EN   3   // QD

#define M1DIR  4   // QE
#define M2DIR  5   // QF
#define M3DIR  6   // QG
#define M4DIR  7   // QH

#define MaxServo   4
#define MaxStepper 4

const int ENABLE[MaxStepper]    = { M1EN, M2EN, M3EN, M4EN };
const int DIRECTION[MaxStepper] = { M1DIR, M2DIR, M3DIR, M4DIR };
const int STEP[MaxStepper]      = { M1STEP, M2STEP, M3STEP, M4STEP };
const int ES[MaxStepper]        = { ES1, ES2, ES3, ES4 };
const int SERVO[MaxServo]       = { SERVO1, SERVO2, SERVO3, SERVO4 };

#endif
