////////////////////////////////////////////////////
//
// ftPwrDrive Arduino Interface
//
// 25.08.2019 V0.9 / latest version
//
// (C) 2019 Christian Bergschneider & Stefan Fuss
//
///////////////////////////////////////////////////

#include "ftPwrDrive.h"
#include "i2cBuffer.h"
#include <Wire.h>

// ftPwrDrive Commands
#define CMD_SETWATCHDOG         0  // void setWatchdog( long interval )                                  set watchdog timer

#define CMD_SETMICROSTEPMODE    1  // void setMicrostepMode( mode )                                      set microstep mode - FILLSTEP, HALFSTEP, QUARTERSTEP, EIGTHSTEP, SIXTEENTHSTEP
#define CMD_GETMICROSTEPMODE    2  // uint8_t getMicroStepMode( void )                                   get microstep mode - FILLSTEP, HALFSTEP, QUARTERSTEP, EIGTHSTEP, SIXTEENTHSTEP

#define CMD_SETRELDISTANCE      3  // void setRelDistance( uint8_t motor, long distance )                set a distance to go, relative to actual position
#define CMD_SETABSDISTANCE      5  // void setAbsDistance( uint8_t motor, long distance )                set a absolute distance to go
#define CMD_GETSTEPSTOGO        7  // long getSetpsToGo( uint8_t motor )                                 number of needed steps to go to distance

#define CMD_SETMAXSPEED         8  // void setMaxSpeed( uint8_t axis, long maxSpeed )                    set max speed
#define CMD_GETMAXSPEED         9  // long getMaxSpeed( uint8_t axis );                                  get max speed

#define CMD_STARTMOVING        10  // void startMoving( uint8_t motor, boolean disableOnStop )           start motor moving, disableOnStop disables the motor driver at the end of the movement
#define CMD_STARTMOVINGALL     11  // void startMovingAll( uint8_t maskMotor, uint8_t maskDisableOnStop )   same as StartMoving, but using uint8_t masks
#define CMD_ISMOVING           12  // boolean isMoving( uint8_t motor )                                  check, if a motor is moving
#define CMD_ISMOVINGALL        13  // uint8_t isMovingAll(  )                                            return value is uint8_tmask, flag 1 is motor#1, flag2 is motor #2, ...

#define CMD_GETSTATE           14  // uint8_t getState( uint8_t motor )                                  8754321  - flag 1 motor is running, flag 2 endstop, flag 3 EMS, flag 4 position overrun

#define CMD_SETPOSITION        15  // void setPosition( uint8_t motor, long position )                   set position
#define CMD_SETPOSITIONALL     16  // void setPositionAll( long p1, long p2, long p3, long p4 )          set position of all motors
#define CMD_GETPOSITION        17  // long getPosition( uint8_t motor )                                  get position
#define CMD_GETPOSITIONALL     18  // (long,long,long,long) getPositionAll( void )                       get position of all motors

#define CMD_SETACCELERATION    19  // void setAcceleration( uint8_t motor, long acceleration )           set acceleration
#define CMD_GETACCELERATION    20  // long getAcceleration( uint8_t motor )                              get acceleration
#define CMD_SETACCELERATIONALL 21  // void setAccelerationAll( long acc1, long acc2, long acc3, long acc4 ) set acceleration of all motors
#define CMD_GETACCELERATIONALL 22  // (long,long,long,long) getAccelerationAll( void )                   get acceleration of all motors

i2cBuffer i2c;

ftPwrDrive::ftPwrDrive( uint8_t myI2CAddress ) { 
  // // constructor
  i2cAddress = myI2CAddress;
  Wire.begin();
}

void ftPwrDrive::Watchdog( long wtime ) {
  // set wartchog timer
  i2c.sendData( i2cAddress, CMD_SETWATCHDOG, wtime );
}

void ftPwrDrive::setMicrostepMode( uint8_t mode ) {
  // set microstep mode - FILLSTEP, HALFSTEP, QUARTERSTEP, EIGTHSTEP, SIXTEENTHSTEP
  i2c.sendData( i2cAddress, (uint8_t)CMD_SETMICROSTEPMODE, mode );
}

uint8_t ftPwrDrive::getMicrostepMode( void ) {
  // get microstep mode - FILLSTEP, HALFSTEP, QUARTERSTEP, EIGTHSTEP, SIXTEENTHSTEP
  return i2c.receiveuint8_t( i2cAddress, CMD_GETMICROSTEPMODE );
}

void ftPwrDrive::setRelDistance( uint8_t motor, long distance ) {
  // set a distance to go, relative to actual position
  i2c.sendData( i2cAddress, CMD_SETRELDISTANCE, motor, distance );
}

void ftPwrDrive::setAbsDistance( uint8_t motor, long distance ) {
  // set a absolute distance to go
  i2c.sendData( i2cAddress, CMD_SETABSDISTANCE, motor, distance );
}

long ftPwrDrive::getStepsToGo( uint8_t motor ) {
  // number of needed steps to go to distance
  return i2c.receiveLong( i2cAddress, CMD_GETSTEPSTOGO, motor );
}

void ftPwrDrive::setMaxSpeed( uint8_t motor, long speed) {
  // set max speed
  i2c.sendData( i2cAddress, CMD_SETMAXSPEED, motor, speed );
}

long ftPwrDrive::getMaxSpeed( uint8_t motor ) {
  // get max speed
  return i2c.receiveLong( i2cAddress, CMD_GETMAXSPEED, motor );
}

void ftPwrDrive::startMoving( uint8_t motor, boolean disableOnStop = true ) {
  // start motor moving, disableOnStop disables the motor driver at the end of the movement
  i2c.sendData( i2cAddress, CMD_STARTMOVING, motor, disableOnStop );
}

void ftPwrDrive::startMovingAll( uint8_t maskMotor, uint8_t maskDisableOnStop = FTPWRDRIVE_M1|FTPWRDRIVE_M2|FTPWRDRIVE_M3|FTPWRDRIVE_M4  ) {
  // same as StartMoving, but using uint8_t masks
  i2c.sendData( i2cAddress, CMD_STARTMOVINGALL, maskMotor, maskDisableOnStop );
}

boolean ftPwrDrive::isMoving( uint8_t motor ) {
  // check, if a motor is moving
  return i2c.receiveuint8_t( i2cAddress, CMD_ISMOVING, motor );
}
  
uint8_t ftPwrDrive::isMovingAll( void ) {
  // return value is uint8_tmask, flag 1 is motoris#1, flag2 is motor #2, 
  return i2c.receiveuint8_t( i2cAddress, CMD_ISMOVINGALL );
}

uint8_t ftPwrDrive::getState( uint8_t motor ) {
  // 8754321  - flag 1 motor is running, flag 2 endstop, flag 3 EMS, flag 4 position overrun
  return i2c.receiveuint8_t( i2cAddress, CMD_GETSTATE );
}

void ftPwrDrive::setPosition( uint8_t motor, long position ) {
  // set position
  i2c.sendData( i2cAddress, CMD_SETPOSITION, motor, position );
}

void ftPwrDrive::setPositionAll( long p1, long p2, long p3, long p4 ) {
  // set position of all motors
  i2c.sendData( i2cAddress, CMD_SETPOSITIONALL, p1, p2, p3, p4 );
}

long ftPwrDrive::getPosition( uint8_t motor ) {
  // get position
  return i2c.receiveLong( i2cAddress, CMD_GETPOSITION, motor );
}

void ftPwrDrive::getPositionAll( long &p1, long &p2, long &p3, long &p4 ) {
  // get position of all motors
  i2c.receive4Long( i2cAddress, CMD_GETPOSITIONALL, p1, p2, p3, p4 );
}

void ftPwrDrive::setAcceleration( uint8_t motor, long acceleration ) {
  // set acceleration
  i2c.sendData( i2cAddress, CMD_SETACCELERATION, motor, acceleration );
}

void ftPwrDrive::setAccelerationAll( long a1, long a2, long a3, long a4 ) {
  // set acceleration of all motors
  i2c.sendData( i2cAddress, CMD_SETACCELERATIONALL, a1, a2, a3, a4 );
}

long ftPwrDrive::getAcceleration( uint8_t motor ) {
  // get acceleration
  return i2c.receiveLong( i2cAddress, CMD_GETACCELERATION, motor );
}

void ftPwrDrive::getAccelerationAll( long &a1, long &a2, long &a3, long &a4 ) {
   // get acceleration of all motors

  i2c.receive4Long( i2cAddress, CMD_GETACCELERATIONALL, a1, a2, a3, a4 );
}
   
