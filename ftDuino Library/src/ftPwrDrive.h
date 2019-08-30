////////////////////////////////////////////////////
//
// ftPwrDrive Arduino Interface
//
// 25.08.2019 V0.9 / latest version
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
    
// motor numbers
static const uint8_t FTPWRDRIVE_M1 = 1, FTPWRDRIVE_M2 = 2, FTPWRDRIVE_M3 = 4, FTPWRDRIVE_M4 = 8;

// number of motors
static const uint8_t FTPWRDRIVE_MOTORS = 4;
    
// enumeration of motor numbers
static const uint8_t FTPWRDRIVE_M[ FTPWRDRIVE_MOTORS ] = { FTPWRDRIVE_M1, FTPWRDRIVE_M2, FTPWRDRIVE_M3, FTPWRDRIVE_M4 };

// flags, i.e. used in getState
static const uint8_t FTPWRDRIVE_ISMOVING = 1, FTPWRDRIVE_ENDSTOP = 2, FTPWRDRIVE_EMERCENCYSTOP = 3;

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
      // return value is uint8_tmask, flag 1 is motor#1, flag2 is motoris #2, ...
      
    uint8_t getState( uint8_t motor );
      // 8754321  - flag 1 motor is running, flag 2 endstop, flag 3 EMS, flag 4 position overrun
      
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

  private:
    uint8_t i2cAddress = 32;
};


#endif
