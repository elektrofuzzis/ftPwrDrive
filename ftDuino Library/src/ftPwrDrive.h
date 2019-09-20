////////////////////////////////////////////////////
//
// ftPwrDrive Arduino Interface
//
// 20.09.2019 V0.93 / latest version
//
// (C) 2019 Christian Bergschneider & Stefan Fuss
//
///////////////////////////////////////////////////

#ifndef ftPwrDrive_h
#define ftPwrDrive_h

#include <Arduino.h>

// some constant to make live easier:
 
// microstep modes
static const uint8_t FTPWRDRIVE_FULLSTEP = 0, FTPWRDRIVE_HALFSTEP = 4, FTPWRDRIVE_QUARTERSTEP = 2, FTPWRDRIVE_EIGTHSTEP = 6, FTPWRDRIVE_SIXTEENTHSTEP = 7;
    
// servo numbers
static const uint8_t FTPWRDRIVE_S1 = 0, FTPWRDRIVE_S2 = 1, FTPWRDRIVE_S3 = 2, FTPWRDRIVE_S4 = 3;

// number of servos
static const uint8_t FTPWRDRIVE_SERVOS = 4;

// enumeration of servo numbers
static const uint8_t FTPWRDRIVE_S[ FTPWRDRIVE_SERVOS ] = { FTPWRDRIVE_S1, FTPWRDRIVE_S2, FTPWRDRIVE_S3, FTPWRDRIVE_S4 };

// motor numbers
static const uint8_t FTPWRDRIVE_M1 = 1, FTPWRDRIVE_M2 = 2, FTPWRDRIVE_M3 = 4, FTPWRDRIVE_M4 = 8;

// number of motors
static const uint8_t FTPWRDRIVE_MOTORS = 4;
    
// enumeration of motor numbers
static const uint8_t FTPWRDRIVE_M[ FTPWRDRIVE_MOTORS ] = { FTPWRDRIVE_M1, FTPWRDRIVE_M2, FTPWRDRIVE_M3, FTPWRDRIVE_M4 };

// flags, i.e. used in getState
static const uint8_t FTPWRDRIVE_ISMOVING = 1, FTPWRDRIVE_ENDSTOP = 2, FTPWRDRIVE_EMERCENCYSTOP = 4, FTPWRDRIVE_HOMING = 8;

class ftPwrDrive {
  public:
    
    ftPwrDrive( uint8_t myI2CAddress );
      // constructor
      
    void Watchdog( long w );
      // set wartchog timer
      
    void setMicrostepMode( uint8_t mode );
      // set microstep mode - FILLSTEP, HALFSTEP, QUARTERSTEP, EIGTHSTEP, SIXTEENTHSTEP
      
    uint8_t getMicrostepMode( void );
      // get microstep mode - FILLSTEP, HALFSTEP, QUARTERSTEP, EIGTHSTEP, SIXTEENTHSTEP
      
    void setRelDistance( uint8_t motor, long distance );
      // set a distance to go, relative to actual position
      
    void setAbsDistance( uint8_t motor, long distance );
      // set a absolute distance to go
      
    void setAbsDistanceAll( long d1, long d2, long d3, long d4 );
      // set a absolute distance to go for all motors
      
    long getStepsToGo( uint8_t motor );
      // number of needed steps to go to distance
      
    void setMaxSpeed( uint8_t motor, long speed );
      // set a max speed
      
    long getMaxSpeed( uint8_t motor);
      // get max speed
      
    void startMoving( uint8_t motor, boolean disableOnStop = true );
      // start motor moving, disableOnStop disables the motor driver at the end of the movement
      
    void startMovingAll( uint8_t maskMotor, uint8_t maskDisableOnStop = FTPWRDRIVE_M1|FTPWRDRIVE_M2|FTPWRDRIVE_M3|FTPWRDRIVE_M4 );
      // same as StartMoving, but using uint8_t masks
      
    boolean isMoving( uint8_t motor );
      // check, if a motor is moving
      
    uint8_t isMovingAll( );
      // return value is uint8_tmask, flag 1 is motor#1, flag2 is motor#2, ...

    void wait( uint8_t motor_mask, uint16_t interval = 100 );
      // wait until all motors in motor_mask completed their work
      
    uint8_t getState( uint8_t motor );
      // 8754321  - flag 1 motor is running, flag 2 endstop, flag 3 EMS, flag 4 homing

    boolean endStopActive( uint8_t motor );
      // check, if end stop is pressed

    boolean emergencyStopActive( void );
      // check, if emergeny stop is pressed
      
    void setPosition( uint8_t motor, long position );
      // set position
      
    void setPositionAll( long p1, long p2, long p3, long p4 );
      // set position of all motors
      
    long getPosition( uint8_t motor );
      // get position
      
    void getPositionAll( long &p1, long &p2, long &p3, long &p4 );
      // get position of all motors

    // don't use acceleration yet - just stub implementation
      
    void setAcceleration( uint8_t motor, long acceleration );
      // set acceleration
      
    void setAccelerationAll( long a1, long a2, long a3, long a4 );
      // set accelerationof all motors
      
    long getAcceleration( uint8_t motor );
      // get acceleration

    void getAccelerationAll( long &a1, long &a2, long &a3, long &a4 );
      // get acceleration of all motors

    void setServo( uint8_t servo, long position );
      // set servo position

    long getServo( uint8_t servo );
      // get servo position

    void setServoAll( long p1, long p2, long p3, long p4 );
      // set all servos positions

    void getServoAll( long &p1, long &p2, long &p3, long &p4 );
      // get all servo positions
      
    void setServoOffset( uint8_t servo, long offset );
      // set servo offset

    long getServoOffset( uint8_t servo );
      // get servo offset

    void setServoOffsetAll( long o1, long o2, long o3, long o4 );
      // set servo offset all

    void getServoOffsetAll( long &o1, long &o2, long &o3, long &o4 );
      // get all servo offset

    void setServoOnOff( uint8_t servo, boolean on );
      // set servo pin On or Off without PWM

    void homing( uint8_t motor, long maxDistance, boolean disableOnStop = true );
      // homing of motor using end stop

    boolean isHoming( uint8_t motor );
      // check, homing is active

  private:
    uint8_t i2cAddress = 32;
};


#endif
