////////////////////////////////////////////////////////////////////////////////
//
// ftPwrDrive Firmware
//
// 10.02.2022 V0.99
//
// (C) 2019-2022 Christian Bergschneider & Stefan Fuss
//
////////////////////////////////////////////////////////////////////////////////

// Bugfixes & Features:
//
// #0007 Optimized homing
//
// #0008 some bugfixes to check, if there is not need to start moving
//
// #0009 bugfix checking "inSync"-Option in stepperTimer 
//
// #0010 Optimized homing using ft switches
//
// #0011 bugfix change max power settings

#include <Arduino.h>

#include <SPI.h>
#include <TimerOne.h>
#include <TimerThree.h>
#include <Wire.h>
#include <EEPROM.h>
#include "HC595.h"
#include "ftPwrDriveHW.h"

// ********* some useful definitions *********

#define myVersion 0.99

// micro stepping modes
#define FULLSTEP      0
#define HALFSTEP      4
#define QUARTERSTEP   2
#define EIGTHSTEP     6
#define SIXTEENTHSTEP 7

// ftPwrDrive Commands
#define CMD_SETWATCHDOG         0  // void setWatchdog( long interval )                                  set watchdog timer

#define CMD_SETMICROSTEPMODE    1  // void setMicrostepMode( mode )                                      set microstep mode - FULLSTEP, HALFSTEP, QUARTERSTEP, EIGTHSTEP, SIXTEENTHSTEP
#define CMD_GETMICROSTEPMODE    2  // uint8_t getMicroStepMode( void )                                   get microstep mode - FULLSTEP, HALFSTEP, QUARTERSTEP, EIGTHSTEP, SIXTEENTHSTEP

#define CMD_SETRELDISTANCE      3  // void setRelDistance( uint8_t motor, long distance )                set a distance to go, relative to actual position
#define CMD_SETABSDISTANCE      5  // void setAbsDistance( uint8_t motor, long distance )                set a absolute distance to go
#define CMD_GETSTEPSTOGO        7  // long getSetpsToGo( uint8_t motor )                                 number of needed steps to go to distance

#define CMD_SETMAXSPEED         8  // void setMaxSpeed( uint8_t axis, long maxSpeed )                    set max speed
#define CMD_GETMAXSPEED         9  // long getMaxSpeed( uint8_t axis );                                  get max speed

#define CMD_STARTMOVING        10  // void startMoving( uint8_t motor, boolean disableOnStop )           start motor moving, disableOnStop disables the motor driver at the end of the movement
#define CMD_STARTMOVINGALL     11  // void startMovingAll( uint8_t maskMotor, uint8_t maskDisableOnStop )   same as StartMoving, but using uint8_t masks
#define CMD_ISMOVING           12  // boolean isMoving( uint8_t motor )                                  check, if a motor is moving
#define CMD_ISMOVINGALL        13  // uint8_t isMovingAll(  )                                            return value is uint8_tmask, flag 1 is motor#1, flag2 is motor #2, ...

#define CMD_GETSTATE           14  // uint8_t getState( uint8_t motor )                                  8754321  - flag 1 motor is running, flag 2 endstop, flag 3 EMS, flag 4 position overrun, flag 5 homing

#define CMD_SETPOSITION        15  // void setPosition( uint8_t motor, long position )                   set position
#define CMD_SETPOSITIONALL     16  // void setPositionAll( long p1, long p2, long p3, long p4 )          set position of all motors
#define CMD_GETPOSITION        17  // long getPosition( uint8_t motor )                                  get position
#define CMD_GETPOSITIONALL     18  // (long,long,long,long) getPositionAll( void )                       get position of all motors

#define CMD_SETACCELERATION    19  // void setAcceleration( uint8_t motor, long acceleration )           set acceleration
#define CMD_GETACCELERATION    20  // long getAcceleration( uint8_t motor )                              get acceleration
#define CMD_SETACCELERATIONALL 21  // void setAccelerationAll( long acc1, long acc2, long acc3, long acc4 ) set acceleration of all motors
#define CMD_GETACCELERATIONALL 22  // (long,long,long,long) getAccelerationAll( void )                   get acceleration of all motors

#define CMD_SETSERVO           23  // void setServo( uint8_t servo, long position )                      set servo position
#define CMD_GETSERVO           24  // long getServo( uint8_t servo )                                     get servo position
#define CMD_SETSERVOALL        25  // void setServoAll( long p1, long p2, long p3, long p4 )             set all servos positions
#define CMD_GETSERVOALL        26  // (long, long, long, long) getServoAll( void )                       get all servo positions
#define CMD_SETSERVOOFFSET     27  // void setServoOffset( uint8_t servo, long Offset )                  set servo offset
#define CMD_GETSERVOOFFSET     28  // long getServoOffset( void )                                        get servo offset
#define CMD_SETSERVOOFFSETALL  29  // void setServoOffsetAll( long o1, long o2, longnt o3, long o4 )     set servo offset all
#define CMD_GETSERVOOFFSETALL  30  // (long, long, long, long) getServoOffsetAll( void )                 get all servo offset
#define CMD_SETSERVOONOFF      31  // void setServoOnOff( uint8_t servo, boolean OnOff )                 set servo pin On or Off without PWM

#define CMD_HOMING             32  // void homing( uint8_t motor, long maxDistance, boolean disableOnStop) homing of motor "motor": run max. maxDistance or until endstop is reached. 

#define CMD_STOPMOVING         33  // void stopMoving( uint8_t motor )                                  stop motor moving
#define CMD_STOPMOVINGALL      34  // void stopMovingAll( uint8_t maskMotor )                           same as StartMoving, but using uint8_t masks

#define CMD_SETINSYNC          35  // void setInSync( unit8_t motor1, uint8_t motor2, boolean OnOff)    set two motors running in sync

#define CMD_HOMINGOFFSET       36  // void homingOffset( unit8_t motor1, ulong offset )                 set offset to run during homing, after endstop is free again 

#define stepperInterval        100  // 10kHz
#define servoInterval           25  // 40kHz

#define HOMING_OFF    0
#define HOMING_PHASE1 1
#define HOMING_PHASE2 2
#define HOMING_PHASE3 3

// type to control one stepper
struct t_stepper {
  long    cycle = 0; 
  long    cycleCounter = 0;
  long    stepsToGo = 0;              // distance in steps to go
  int8_t  cw = 1;                     // running counterwise 1, contra clockwise -1
  int8_t  cwEndStop = 0;              // cw-mode before getting an end stop trigger event,  0 = unknown state
  long    position = 0;               // absolute position
  long    maxSpeed = 0;
  long    acceleration = 0;
  boolean disableOnStop = true;
  boolean isMoving = false;
  uint8_t homing = HOMING_OFF;
  long    homingOffset = 0;          // steps to go during homing, if endstop is released in phase 2
  boolean endStop = false;
  boolean ignoreEndStop = false;     // semaphore to ignore an end stop trigger, to move out of an triggered event
  uint8_t inSyncWith;                // the motor, I'm running in sync. if none, my own number
  
};

#define SERVOINTERNALOFFSET 60

// type to control all servos
struct t_servo {
  volatile long duty = SERVOINTERNALOFFSET;
  long dutyCounter;
  long position = 0;
  long offset = 0;
};

boolean emergencyStop = false;
uint8_t microstepMode = FULLSTEP;

#define maxCmdSize 16

struct t_CmdBlock {
  boolean newCmd = false;
  uint8_t Cmd[maxCmdSize];
};

// Servos
t_servo Servo[MaxServo];

long servoCycle = 20000 / servoInterval;
long servoCycleCounter = 0;
boolean timer1Started = false;

// Steppers
t_stepper Stepper[MaxStepper];

// received command data
t_CmdBlock CmdBlock;

// returnBuffer & Size
uint8_t returnBuffer[16];
uint8_t returnBytes = 0;

// watchdog:
//  -1 watchdog deactivated
//  >0 time in millis when the watchdog should stop the system
long    watchdog = -1;

// i²c-address
int myI2CBusAddress = 0x20;

// maxMotorCurrent
float maxMotorCurrent = 0.7;

// mySerialNumber
int mySerialNumber = 0;

// Mode: Maintenance/Normal
#define MAINTENANCE      1
#define NORMAL           0
uint8_t mode = NORMAL;

// ************ motor current control ***************
boolean      boardV3 = false;

float getCurrent( uint8_t digits) {
  // gets the motors current setting
  // works only in V2 mode

  int a;
  float v,c;

  if (boardV3) {
    return maxMotorCurrent;
  }

  // get reading
  a = analogRead( MREFI );
  // convert to volts
  v = ( (float) a ) / 1024 * 2.56;   
  // calculate motor current
  c = v / ( 8 * 0.068 );

  // return c with 2 digits precision
  return ((float) ((int)(c*digits))) / digits; 
  
}

bool checkCurrent(float givenCurrent) {
  // checks, in actualReading fits to a given value

  if (boardV3) {
    return true;
  }

  // getCurrent
  float c = getCurrent(100);

  // check deviation
  if (abs( givenCurrent - c ) <= 0.05 ) {
    // in tolerance
    return true;
    
  } else {
    // first reading out of tolaerance, try a second one
    delay(50);
    c = getCurrent(100);
    return (abs( givenCurrent - c ) <= 0.05 );

  }

}

void setCurrent(float current) {

  if (boardV3) {
    return;
  }

  if ( getCurrent( 10 ) != current ) {

    Serial.println( "Please turn the trim resistor to set the motor current." );
  
    while ( getCurrent( 10 ) != current ) {
      Serial.print("max current: "); Serial.print(  getCurrent( 100 ) );
      Serial.println( "A" );
      delay( 100 );
    }
    
  }
  
}

void writeDAC( uint8_t dac, uint8_t value ) {

  uint16_t cmd = ((uint16_t)dac) << 14 | 0x1000 | ((uint16_t) value) << 4;

  digitalWrite( MREFI, LOW );
  SPI.transfer16( cmd );
  digitalWrite( MREFI, HIGH );
}

void writeDACC( uint8_t dac, float current ) {

  // convert current to reference volts
  float   v   = current * ( 8 * 0.068 );

  uint8_t x =  (uint8_t)( v / 2.0 * 256 );

  writeDAC( dac, x );
}

// ********** init hardware & maintenance mode **********

void initializeHardware( void ) {
  // do all initialisation stuff

  int i;

  // *** SPI ***
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE2);

  // Board Type with DAC?

  analogReference(INTERNAL);
  pinMode(MREFI, INPUT);
  boardV3 = (analogRead( MREFI ) > 1000);

  if ( boardV3) {
  
    pinMode( MREFI, OUTPUT );
  
    for (i = 0; i<4; i++ ) {
      writeDACC(i, maxMotorCurrent );
    }

  };

  // *** all stepper stuff ***

  // all drivers off
  write595( 31 ); 

  // Servos
  for (i=0; i<MaxServo; i++ ) {
    pinMode( SERVO[i], OUTPUT );
    digitalWrite( SERVO[i], LOW );
  }

  // Steppers and end stops
  for (i=0; i<MaxStepper; i++ ) {
    pinMode( STEP[i], OUTPUT );
    pinMode( ES[i], INPUT_PULLUP);
  }
  
  // emergency end stop
  pinMode( EMS, INPUT_PULLUP);

  // microsteps 1/1
  pinMode( MS1, OUTPUT );
  pinMode( MS2, OUTPUT );
  pinMode( MS3, OUTPUT );
  microsteps( FULLSTEP );
    
  // *** LED stuff: always on ***
  pinMode( LED, OUTPUT );
  digitalWrite( LED, HIGH );

  // *** reference voltage ***  
  pinMode( MREFI, INPUT );
  analogReference( INTERNAL );

  // *** I2C ***
  Wire.begin(myI2CBusAddress);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
    
}

boolean readEEPROM( void ) {
  // reads the EEPROM, false if EEPROM was not initialized

  uint8_t eHeader     = EEPROM.read(0);
  uint8_t eVersion    = EEPROM.read(1);
  uint8_t eSerialLow  = EEPROM.read(2);
  uint8_t eSerialHigh = EEPROM.read(3);
  uint8_t eI2CAddress = EEPROM.read(4);
  uint8_t eCurrent    = EEPROM.read(5);
  uint8_t eCRC        = EEPROM.read(6);

  uint8_t c1 = eHeader + eSerialLow + eSerialHigh + eVersion + eI2CAddress + eCurrent;
  uint8_t c2 = c1 % 10;

  if ( ( eHeader == 42 ) && ( eVersion == 1 ) && ( c2 == eCRC ) ) {
    // this block was writte by ftPwrDrive with correct CRC 
    myI2CBusAddress = eI2CAddress;
    maxMotorCurrent = ((float) eCurrent ) / 10;
    mySerialNumber  = (unsigned int) eSerialLow + ((unsigned int) eSerialHigh ) * 256;
    return true;
  } else {
    return false;
  }

}

void writeEEPROM( void ) {
  // writes the EEPROM

  uint8_t eHeader     = 42;
  uint8_t eVersion    = 1;
  uint8_t eSerialLow  = mySerialNumber & 0xFF;
  uint8_t eSerialHigh = mySerialNumber >> 8;
  uint8_t eI2CAddress = myI2CBusAddress;
  uint8_t eCurrent    = (uint8_t) (maxMotorCurrent * 10 );

  uint8_t c1 = eHeader + eSerialLow + eSerialHigh + eVersion + eI2CAddress + eCurrent;
  uint8_t c2 = c1 % 10;

  EEPROM.write( 0, eHeader );
  EEPROM.write( 1, eVersion );
  EEPROM.write( 2, eSerialLow );
  EEPROM.write( 3, eSerialHigh );
  EEPROM.write( 4, eI2CAddress );
  EEPROM.write( 5, eCurrent );
  EEPROM.write( 6, c2 );
  
}

/*

float maxCurrent( int digits ) {
  // reads the reference voltage

  int a;
  float v,c;

  // get the analog value
  a = analogRead( MREFI );

  // cast analog vaue to volts
  v = ( (float) a ) / 1024 * 2.56;        

  // cast volts to motor current
  c = v / ( 8 * 0.068 );

  // return c with 2 digits precision
  return ((float) ((int)(c*digits))) / digits;  
}
*/

void BannerText( void ) {

  Serial.println( "  __ _   _____                _____       _" );
  Serial.println( " / _| | |  __ \\              |  __ \\     (_)" );
  Serial.println( "| |_| |_| |__) |_      ___ __| |  | |_ __ ___   _____" );
  Serial.println( "|  _| __|  ___/\\ \\ /\\ / / '__| |  | | '__| \\ \\ / / _ \\" );
  Serial.println( "| | | |_| |     \\ V  V /| |  | |__| | |  | |\\ V /  __/" );
  Serial.println( "|_|  \\__|_|      \\_/\\_/ |_|  |_____/|_|  |_| \\_/ \\___|" );
  Serial.println( "" );
  Serial.println( "(C) 2019 Christian Bergschneider & Stefan Fuss" );
  Serial.println( "" );

  Serial.print  ( "Board type:         "); if (boardV3) { Serial.println("V3"); } else { Serial.println("V2"); }
  Serial.print  ( "FW-Version          "); Serial.println( myVersion );

  Serial.print  ( "Serial number:      ");  Serial.println( mySerialNumber );
  Serial.print  ( "I2C-Address:        " ); Serial.println( myI2CBusAddress );

  // need some time to get a first correct value
  float c = getCurrent(100);
  delay(50);
  c = getCurrent(100);
  
  Serial.print  ( "max. motor current: " ); Serial.print( maxMotorCurrent ); Serial.print( "A / " ); Serial.print( c ); Serial.println( "A" );

  if ( abs( maxMotorCurrent - c ) > 0.05 ) {
    mode = MAINTENANCE;
    activateErrorLED();
    while (!Serial);
    Serial.println( "System error: motor current doesn't fit given value." );
  }

  if (mode==NORMAL) {
    Serial.println( "" );
    Serial.println( "Operating mode normal." );
    Serial.println( "Press any key to enter maintenance mode.");
  }
  
}

  
void setup() {
  // put your setup code here, to run once:

  if (!readEEPROM()) {
    mode = MAINTENANCE;
  }
  
  initializeHardware();

  #ifdef __debug__
    while (!Serial);
  #endif

  // initialize serial
  Serial.begin( 9600 );
  Serial.setTimeout( 0 );

  // initialize Stepper
  for (int i=0; i<MaxStepper; i++) {
    Stepper[i].inSyncWith = i;
  }

  BannerText();

}

String readln( char prompt[] ) {
  // read keyboard via serial until \n

  char result[10];
  int i = 0;
  char lastChar = 0;

  // prompt
  Serial.print( prompt );

  // read until EOL
  while (lastChar != '\n') {
    
    // if a char is available
    if ( Serial.available() ) {
      
      // read and print it
      lastChar = Serial.read();
      Serial.print( lastChar );

      // if there is a buffer overflow, just ignore the new char
      // otherwise add it to the buffer
      if (i<9) {
        result[i] = lastChar;
        i++;
      }
    }
  }
  
  // terminate the string
  result[i] = 0;

  // and return it
  return result;
  
}

void setMaxCurrent( void ) {
  
  float  c = 0;
  String result;
  float  newMaxMotorCurrent = 0;
  char   buffer[20], x[10];

  dtostrf( maxMotorCurrent, 3, 1, x);
  sprintf( buffer, "(%s)>", x );
  Serial.println( "Please select your motor's max. current:" );
  Serial.println( "(1) 0.1A     (3) 0.3 A     (5) 0.5 A     (7) 0.7 A     (9) 0.9 A" );
  Serial.println( "(2) 0.2A     (4) 0.4 A     (6) 0.6 A     (8) 0.8 A     (0) 1.0 A" );

  while (newMaxMotorCurrent==0) {
    result = readln( buffer );

    if ( result[0] == '\n' ) {
      // default
      newMaxMotorCurrent = maxMotorCurrent;
      
    } else if ((result[0] < '0') || (result[0] > '9')) {
      // out of range
      Serial.println( "Error: please enter a valid value." );
      
    } else {
      // new value choosen
      newMaxMotorCurrent = result.toInt();
      if ( newMaxMotorCurrent == 0 ) newMaxMotorCurrent = 10;
      newMaxMotorCurrent /= 10;
    }
  }

  if ( ( getCurrent( 10 ) != newMaxMotorCurrent ) && (!boardV3) ) {

    Serial.println( "Please turn the trim resistor to set the motor current." );

    while (c != newMaxMotorCurrent ) {
      c = getCurrent( 100 );
      Serial.print("max current: "); Serial.print( c );
      Serial.println( "A" );
      delay( 100 );
    }
    
  }

  maxMotorCurrent = newMaxMotorCurrent;
  Serial.print( "New max current is " ); Serial.println( maxMotorCurrent ); 
  
}

void setI2CAddress( void ) {

  String cmd;
  char   buffer[20];
  int    newAddr = 0;

  // set the I2C address first
  Serial.println( "Please enter the devices I2C-address (16..127).");
  while ((newAddr<16) || (newAddr>127)) {
    // read keyboard
    sprintf( buffer, "(%d)>", myI2CBusAddress );
    cmd = readln( buffer );
    
    if (cmd[0] == '\n')  {
      // default value?
      newAddr = myI2CBusAddress;
 
    } else {
      // get the new number
      newAddr = cmd.toInt();
    }
    
    if ((newAddr<16) || (newAddr>127)) {
      // if new value not in 16..127 throw an error
      Serial.println( "Error: please enter a valid value." );
    }
  }

  // set new address
  myI2CBusAddress = newAddr;
  sprintf( buffer, "New I2C address is %d.", myI2CBusAddress );
  Serial.println( buffer );
  
}

void setSerialNumber( void ) {
  // sets the serial number

  String sn;
  unsigned int newSerialNumber = 0;
  
  Serial.println( "EEPROM not initialized. Please enter the serial number (1..65535)" );

  while ( newSerialNumber == 0 ) {
    // read serial number
    sn = readln( ">" );
    newSerialNumber = sn.toInt();
  }

  mySerialNumber = newSerialNumber;
  
}

void maintenanceLoop( void ) {

  // all drivers off
  write595( 31 ); 

  while (!Serial);
  BannerText();

  // some user information
  Serial.println( "-----------------------------------------------------------------------" );
  Serial.println( "Starting maintenance mode. Motor drivers and I2C interface are offline.");
  Serial.println( "" );

  if (mySerialNumber==0) {
    // no valid EEPROM data found
    setSerialNumber();
  }

  setI2CAddress();
  Serial.println();
  setMaxCurrent();

  Serial.print( "Saving config data" );
  writeEEPROM();
  for (int i=0; i<3; i++) {
    delay(1000); Serial.print(".");
  }
  Serial.println("");
  Serial.print("Please reboot the device manually.");

  while (true) {
    delay( 100000 );
  }
 
}

// ********** Servo & LED Timer **********

void servoTimer( void ) {

  // interrupt to control the servos

  int i;

  // common clycle counter for all servos
  servoCycleCounter--;

  if ( servoCycleCounter <= 0 ) {
    // servo cycle ended?

    // start new cycle;
    servoCycleCounter = servoCycle;

    // start all duties
    for (i=0;i<MaxServo;i++) {
      Servo[i].dutyCounter = Servo[i].duty;
      
      // start only, if a duty value is set
      if (Servo[i].dutyCounter>0) {
        digitalWrite( SERVO[i], HIGH );
      }
      
    }
    
  } else {
    
    // check all duties if they are ending
    for (i=0;i<MaxServo;i++) {

      // something to count?
      if (Servo[i].dutyCounter>0) {
        
         Servo[i].dutyCounter--;

         // end counter?
         if ( Servo[i].dutyCounter <= 0 ) {
           // stop duty
           digitalWrite( SERVO[i], LOW );
         }
      }
    }
    
  }
  
}

// ********** stepper timer: due to timing problemns, this function is triggers via main loop instead of timer events **********

void StepperTimer( void ) {

  // interrupt to control the steppers
   
  int i;
  boolean endStop;

  emergencyStop = !digitalRead( EMS );

  // check all steppers
  for (i=0; i<MaxStepper; i++ ) {

     // check endStop
     endStop = !digitalRead( ES[i] );

     // run all HOMING stuff only, if a end stop trigger event is found
     if ( endStop != Stepper[i].endStop ) {
      
        if ( endStop && (!Stepper[i].endStop) && ( Stepper[i].homing == HOMING_PHASE1 ) ) {
           // end stop triggert during homing
           Stepper[i].cw = -Stepper[i].cw;                     // change direction
           write595( DIRECTION[i], ( Stepper[i].cw == -1 ) );  // change direction
           Stepper[i].homing = HOMING_PHASE2;                  // run until end stop is released
        }

        if ( (!endStop) && (Stepper[i].endStop) && ( Stepper[i].homing == HOMING_PHASE2 ) ) {
           // end stop released during homing
           Stepper[i].stepsToGo = Stepper[i].homingOffset;    // set steps to go additionally
           Stepper[i].homing    = HOMING_PHASE3;              // start 3rd pahse and run the additional steps
        } 

        // store the new state
        Stepper[i].endStop = endStop;
     }
     

     // check, if the stepper is running
     if ( Stepper[i].isMoving > 0 ) {

       // check on ES or EMS - only if homing is off and ignoreEndStop isn't set
       if ( ( ( Stepper[i].endStop ) && 
              ( Stepper[i].homing != HOMING_PHASE2 ) && 
              ( Stepper[i].homing != HOMING_PHASE3 ) && 
              ( Stepper[i].ignoreEndStop == false ) 
            ) || emergencyStop ) {

          // stop all motors which are in sync (I'm is in sync with myself)
          for (int j=0; j<MaxStepper; j ++ ) {

            if ( Stepper[j].inSyncWith == i ) {
         
              // stop moving
              Stepper[j].isMoving = false;
              Stepper[j].stepsToGo = 0;

              // store cw mode before stopping
              Stepper[j].cwEndStop = Stepper[j].cw;

              // set disableOnStop
              write595( ENABLE[j], Stepper[j].disableOnStop );
              
            }
            
          }
          
       } else {

         // reset ignoreEndStop
         if ( Stepper[i].ignoreEndStop ) {
          Stepper[i].ignoreEndStop = !Stepper[i].endStop;
         }

         // check if cycle ended
         Stepper[i].cycleCounter--;
         if ( Stepper[i].cycleCounter <= 0 ) {

           // invoke a step and start a new cycle
           Stepper[i].cycleCounter = Stepper[i].cycle;
           digitalWrite( STEP[i], HIGH );
           digitalWrite( STEP[i], LOW );
           Stepper[i].stepsToGo--;
           Stepper[i].position += Stepper[i].cw;

           // check if motion has to stop
           if ( Stepper[i].stepsToGo <= 0 ) {
             Stepper[i].isMoving = false;                       // stop moving
             write595( ENABLE[i], Stepper[i].disableOnStop );   // disable motor current if needed
             Stepper[i].homing    = HOMING_OFF;                 // if it was homing, it's all done now
           }
         
         }

       }
       
     }
     
  }
  
}

// ********** I2C commands **********

void setWatchdog( long w ) {
  // activates the watchdog timer

  // store the time, when the watchdog timer should stop the system
  watchdog = millis() + w;
  
}

void setRelDistance( uint8_t motor, long relDistance, boolean force = false ) {

  // cmd is only accepted on the primary motor
  if ( ( Stepper[motor].inSyncWith != motor ) && !force ) {
    return;
  }
  
  // check if another motor is inSnycWith this motor
  for (int i=0; i<MaxStepper; i++ ) {
    if ( ( Stepper[i].inSyncWith == motor ) && ( i != motor ) ) {
      // run same command
      setRelDistance( i, relDistance );
    }
  }

  // check to run clockwise or counterclockwise
  if ( relDistance < 0 ) {
    // ccw
    Stepper[motor].cw = -1;
    write595( DIRECTION[motor], 1 );
  } else {
    // cw
    Stepper[motor].cw = 1;
    write595( DIRECTION[motor], 0 );
  }

  // set distance 
  Stepper[motor].stepsToGo = abs(relDistance);

}

void setAbsDistance( uint8_t motor, long absDistance ) {
  // set a absolute distance to go to

  setRelDistance( motor, absDistance - Stepper[motor].position );

}

void setMaxSpeed( uint8_t motor, long speed, boolean force = false ) {
  // set cyle times, speed is steps/second

  // cmd is only accepted on the primary motor
  if ( ( Stepper[motor].inSyncWith != motor ) && !force ) {
    return;
  }
  
  // check if another motor is inSnycWith this motor
  for (int i=0; i<MaxStepper; i++ ) {
    if ( ( Stepper[i].inSyncWith == motor ) && ( i != motor ) ) {
      // run same command
      setMaxSpeed( i, speed, true );
    }
  }

  Stepper[motor].maxSpeed = speed;
  Stepper[motor].cycle = (long)( (1 / (float) speed ) * 1000000 / stepperInterval);

}

void startMoving( uint8_t motor, boolean disableOnStop, boolean force = false ) {
  // start a motor

  // cmd is only accepted on the primary motor
  if ( ( Stepper[motor].inSyncWith != motor ) && !force ) {
    return;
  }
 
  // check if another motor is inSnycWith this motor
  for (int i=0; i<MaxStepper; i++ ) {
    if ( ( Stepper[i].inSyncWith == motor ) && ( i != motor ) ) {
      // run same command
      startMoving( i, disableOnStop, true );
    }
  }

  // don't start if emercencyStop is activated
  if ( emergencyStop ) {
    return;
  }

  // if endStop is triggered, test to move in "the other direction" 
  if ( Stepper[motor].endStop ) {  
    // endStop is triggered
    
    if ( Stepper[motor].cwEndStop == 0 ) { 
      // last direction is unknown, so don't start moving
      return;
    }
    
    if ( Stepper[motor].cwEndStop == Stepper[motor].cw ) { 
      // last direction is the same as the new direction, so don't start moving
      return;
    }
    
  }

  // now, the stepper could start
  Stepper[motor].disableOnStop = disableOnStop;
  write595( ENABLE[motor], 0 );      // set enable
  Stepper[motor].isMoving = true;    // start interrupt working

}

void startMovingAll( uint8_t motorMask, uint8_t disableOnStopMask ) {
  // start multiple motors
  
  int i;
  int mask = 1;
  
  for (i=0; i<MaxStepper; i++ ) {

    if ( motorMask & mask ) {
      startMoving( i, ( disableOnStopMask & mask ) );
    }

    mask = mask * 2;

  }

}

void stopMoving( uint8_t motor, boolean force = false ) {
  // stop a motor immediately

 // cmd is only accepted on the primary motor
  if ( ( Stepper[motor].inSyncWith != motor ) && !force ) {
    return;
  }
  
  // check if another motor is inSnycWith this motor
  for (int i=0; i<MaxStepper; i++ ) {
    if ( ( Stepper[i].inSyncWith == motor ) && ( i != motor ) ) {
      // run same command
      stopMoving( i, true );
    }
  }
  
  Stepper[motor].isMoving = false;
  Stepper[motor].stepsToGo = 0;
  write595( ENABLE[motor], Stepper[motor].disableOnStop );

}

void stopMovingAll( uint8_t motorMask ) {
  // stop multiple motors immediately
  
  int i;
  int mask = 1;
  
  for (i=0; i<MaxStepper; i++ ) {

    if ( motorMask & mask ) {
      stopMoving( i );
    }

    mask = mask * 2;

  }

}

void microsteps( uint8_t myMicrostepMode ) {
  // set microstep mode

  microstepMode = myMicrostepMode;
  digitalWrite( MS1, (microstepMode >> 2) & 1 );
  digitalWrite( MS2, (microstepMode >> 1) & 1 );
  digitalWrite( MS3, microstepMode & 1 );
  
}

uint8_t isMovingAll( void ) {
  // check, who is moving

  uint8_t result = 0;
  int  i;
  
  for (i=MaxStepper-1; i>=0; i--) {
    result = ( result * 2 ) | Stepper[i].isMoving;
  }

  return result;
}

uint8_t getStatusMotor( uint8_t motor ) {
  // 8754321  - flag 1 axis is running, flag 2 endstop, flag 3 EMS, flag 4 homing

  boolean isHoming = !( Stepper[motor].homing == HOMING_OFF );
      
  uint8_t x = Stepper[motor].isMoving    |
              Stepper[motor].endStop * 2 |
              emergencyStop          * 4 |
              isHoming               * 8;

  return x;
         
}

void setPosition( uint8_t motor, long position, boolean force = false ) {
  // sets motor position

  // cmd is only accepted on the primary motor
  if ( ( Stepper[motor].inSyncWith != motor ) && !force ) {
    return;
  }
  
  // check if another motor is inSnycWith this motor
  for (int i=0; i<MaxStepper; i++ ) {
    if ( ( Stepper[i].inSyncWith == motor ) && ( i != motor ) ) {
      // run same command
      setPosition( i, position, true );
    }
  }

  Stepper[motor].position = position;
  
}

void getPositionAll() {
  // returns position values of all Steppers

  int i;
  for (i=0;i<MaxStepper;i++) {
    returnBytes = ReturnLong( returnBytes, Stepper[ i ].position );
  }
  
}

void setAcceleration( uint8_t motor, long acceleration, boolean force = false) {
  // sets acceleration

  // cmd is only accepted on the primary motor
  if ( ( Stepper[motor].inSyncWith != motor ) && !force ) {
    return;
  }
 
  // check if another motor is inSnycWith this motor
  for (int i=0; i<MaxStepper; i++ ) {
    if ( ( Stepper[i].inSyncWith == motor ) && ( i != motor ) ) {
      // run same command
      setAcceleration( i, acceleration, true);
    }
  }
  
  Stepper[motor].acceleration = acceleration;
}

void getAccelerationAll() {
  // returns acceleration values of all Steppers

  int i;
  for (i=0;i<MaxStepper;i++) {
    returnBytes = ReturnLong( returnBytes, Stepper[ i ].acceleration );
  }
  
}

void setServo( uint8_t s, long position ) {
  // set servo position
  
  // if not already startet, start servo Timer
  if ( !timer1Started ) {
    timer1Started = true;
    Timer1.initialize( servoInterval );
    Timer1.attachInterrupt( servoTimer );
  }

  Servo[s].position = position;

  noInterrupts();
  Servo[s].duty     = Servo[s].position + Servo[s].offset + SERVOINTERNALOFFSET;
  interrupts();
  
}

void setServoOffset( uint8_t s, long offset ) {
  // set servo offset

  Servo[s].offset = offset;
  
  noInterrupts();
  Servo[s].duty   = Servo[s].position + Servo[s].offset + SERVOINTERNALOFFSET;
  interrupts();

}


void getServoAll() {
  // returns position values of all Servos

  int i;
  for (i=0;i<MaxServo;i++) {
    returnBytes = ReturnLong( returnBytes, Servo[i].position );
  }
  
}

void getServoOffsetAll() {
  // returns offset values of all Servos

  int i;
  for (i=0;i<MaxServo;i++) {
    returnBytes = ReturnLong( returnBytes, Servo[i].offset );
  }
  
}

void setServoOnOff( uint8_t s, boolean on ) {
  // sets servo pin continous on/off

  noInterrupts();
  Servo[s].duty = -1; // take PWM offline
  interrupts();

  digitalWrite( SERVO[s], on );

}

void homing( uint8_t motor, long maxDistance, boolean disableOnStop ) {
  // run a homing cycle:
  //   1. run maxDistance and stop if maxDistance is reached or end stop found.
  //   2. if end stop found, run with 1/10 speed until end stop is released

  // cmd is only accepted on the primary motor
  if ( Stepper[motor].inSyncWith != motor ) {
    return;
  }

  Stepper[motor].homing = HOMING_PHASE1;
  setRelDistance( motor, maxDistance );
  startMoving( motor, disableOnStop );

}

void homingOffset( uint8_t motor, long Offset) {
  // Apply homing offset to run in HOMING_PHASE3
  Stepper[motor].homingOffset = abs( Offset );
} 

void setInSync( uint8_t motor1, uint8_t motor2, boolean on ) {
  // set two motors to run in sync

  for (int i=0; i<MaxStepper; i++ ) {

   // check if a motor is already syncronized with m1 oder m2: stop motor and "unsync" it
   if ( ( Stepper[i].inSyncWith = motor1 ) || ( Stepper[i].inSyncWith = motor2 ) ) {
    stopMoving( i );
    Stepper[i].inSyncWith = i;
   }

  }

  // if "on" connect the two motors
  if ( on ) {
    Stepper[motor2].inSyncWith = motor1;
    Stepper[motor2].maxSpeed = Stepper[motor1].maxSpeed;
    Stepper[motor2].acceleration = Stepper[motor1].acceleration;
  }

}

long Cmd2Long( uint8_t startFrom ) {
  // gets a long out of the cmdBuffer, starting at position startFrom

  return ((long)  CmdBlock.Cmd[startFrom] ) |
         (((long) CmdBlock.Cmd[startFrom+1] ) << 8 ) |
         (((long) CmdBlock.Cmd[startFrom+2] ) << 16 ) |
         (((long) CmdBlock.Cmd[startFrom+3] ) << 24 );
         
}

int Cmd2Int( uint8_t startFrom ) {
  // gets a int out of the cmdBuffer, starting at position startFrom

  return ((int)  CmdBlock.Cmd[startFrom] ) |
         (((int) CmdBlock.Cmd[startFrom+1] ) << 8 );
         
}

uint8_t ReturnLong( uint8_t startFrom, long value ) {
  // sets a long in the returnBuffer
  // returns startFrom + 4

  for (int i=0; i<4;i++ ) {
    returnBuffer[startFrom] = value & 0xFF;
    value = value >> 8;
    startFrom++;
  }

  return startFrom;
}

uint8_t returnInt( uint8_t startFrom, int value ) {
  // sets a int in the returnBuffer
  // returns startFrom + 2

  for (int i=0; i<2;i++ ) {
    returnBuffer[startFrom] = value & 0xFF;
    value = value >> 8;
    startFrom++;
  }

  return startFrom;
}

uint8_t interface2motor( uint8_t interface ) {
  // translates the motor codes from the external interface to the internal enummeration

  if ( interface & 1 ) { return 0; }
  else if ( interface & 2 ) { return 1; }
  else if ( interface & 4 ) { return 2; }
  else if ( interface & 8 ) { return 3; }
  else { return 0; }
}

uint8_t interface2servo( uint8_t interface ) {
  // translates the servo codes from the external interface to the internal enummeration
  
  return interface & 0x03;
  
}

void cmdInterpreter( void ) {
  // interpretation off external cmds

  // new data available?
  if ( CmdBlock.newCmd ) {

    returnBytes = 0;

    // translate extral bit oriented motor types to internal enummeration
    uint8_t motor = interface2motor( CmdBlock.Cmd[1] );
    uint8_t s     = interface2servo( CmdBlock.Cmd[1] );

    switch ( CmdBlock.Cmd[0] ) {
      
      case CMD_SETWATCHDOG:
        setWatchdog( Cmd2Long( 1 ) );
        break;
        
      case CMD_SETMICROSTEPMODE:
        microsteps( CmdBlock.Cmd[1] );
        break;
        
      case CMD_GETMICROSTEPMODE:
        returnBuffer[ returnBytes++ ] = microstepMode;
        break;
        
      case CMD_SETRELDISTANCE:
        setRelDistance( motor, Cmd2Long( 2 ) );
        break;
        
      case CMD_SETABSDISTANCE:
        setAbsDistance(motor, Cmd2Long( 2 ) );
        break;
        
      case CMD_GETSTEPSTOGO:
        returnBytes = ReturnLong( returnBytes, Stepper[ motor ].stepsToGo );
        break;
        
      case CMD_SETMAXSPEED:
        setMaxSpeed( motor, Cmd2Long( 2 ) );
        break;
        
      case CMD_GETMAXSPEED:
        returnBytes = ReturnLong( returnBytes, Stepper[ motor ].maxSpeed );
        break;
        
      case CMD_STARTMOVING:
        // Motor, disableOnStop
        startMoving( motor, CmdBlock.Cmd[2] );
        break;
        
      case CMD_STARTMOVINGALL:
        // Mask motor, mask disableOnStop
        startMovingAll( CmdBlock.Cmd[1], CmdBlock.Cmd[2] );
        break;

      case CMD_ISMOVING:
        // Motor
        returnBuffer[ returnBytes++ ] = Stepper[ motor ].isMoving ;
        break;

      case CMD_ISMOVINGALL:
        returnBuffer[ returnBytes++ ] = isMovingAll( );
        break;

      case CMD_GETSTATE:
        // Motor
        returnBuffer[ returnBytes++ ] = getStatusMotor( motor );
        break;

      case CMD_SETPOSITION:
        // Motor, Position
        setPosition( motor, Cmd2Long(2) );
        break;

      case CMD_GETPOSITION:
        // Motor
        returnBytes = ReturnLong( returnBytes, Stepper[ motor ].position );
        break;

      case CMD_SETPOSITIONALL:
        // P1, P2, P3, P4
        setPosition( 0, Cmd2Long(1) );
        setPosition( 1, Cmd2Long(5) );
        setPosition( 2, Cmd2Long(9) );
        setPosition( 3, Cmd2Long(13) );
        break;

      case CMD_GETPOSITIONALL:
        getPositionAll();
        break;

      case CMD_SETACCELERATION:
        // motor, acceleration
        setAcceleration( motor, Cmd2Long(2) );
        break;

      case CMD_GETACCELERATION:
        returnBytes = ReturnLong( returnBytes, Stepper[ motor ].acceleration );
        break;

      case CMD_SETACCELERATIONALL:
        // acc1, acc2, acc3, acc4
        setAcceleration( 0, Cmd2Long(1) );
        setAcceleration( 1, Cmd2Long(5) );
        setAcceleration( 2, Cmd2Long(9) );
        setAcceleration( 3, Cmd2Long(13) );
        break;

      case CMD_GETACCELERATIONALL:
        getAccelerationAll();
        break;

      case CMD_SETSERVO:
        // servo, position
        setServo( s, Cmd2Int(2) );
        break;

      case CMD_GETSERVO:
        returnBytes = ReturnLong( returnBytes, Servo[s].position );
        break;

      case CMD_SETSERVOALL:
        // p1, p2, p3, p4
        setServo( 0, Cmd2Long(2) );
        setServo( 1, Cmd2Long(2) );
        setServo( 2, Cmd2Long(2) );
        setServo( 3, Cmd2Long(2) );
        break;

      case CMD_GETSERVOALL:
        getServoAll();
        break;
        
      case CMD_SETSERVOOFFSET:
        // servo, offset
        setServoOffset( s, Cmd2Int(2) );
        break;

      case CMD_GETSERVOOFFSET:
        returnBytes = ReturnLong( returnBytes, Servo[s].offset );
        break;

      case CMD_SETSERVOOFFSETALL:
        // o1, o2, o3, o4
        setServoOffset( 0, Cmd2Long(2) );
        setServoOffset( 1, Cmd2Long(2) );
        setServoOffset( 2, Cmd2Long(2) );
        setServoOffset( 3, Cmd2Long(2) );
        break;

      case CMD_GETSERVOOFFSETALL:
        getServoOffsetAll();
        break;

      case CMD_SETSERVOONOFF:
        setServoOnOff( s, CmdBlock.Cmd[2] );
        break;

      case CMD_HOMING:
        // motor, maxDistance, disableOnStop
        homing( motor, Cmd2Long(2), CmdBlock.Cmd[6] );
        break;

      case CMD_STOPMOVING:
        // Motor
        stopMoving( motor );
        break;
        
      case CMD_STOPMOVINGALL:
        // Mask motor
        stopMovingAll( CmdBlock.Cmd[1] );
        break;

      case CMD_SETINSYNC:
        // motor1, motor2, on
        setInSync( motor, interface2motor( CmdBlock.Cmd[2] ), CmdBlock.Cmd[3] );
        break;

      case CMD_HOMINGOFFSET:
        // motor, offset
        homingOffset( motor, Cmd2Long(2) );
        break;
    }

  }

}

void receiveEvent(int uint8_tsReceived){
  // is called when I2C data is received
  int i;

  // store received data into CommandBuffer
  for (i=0; i<uint8_tsReceived; i++) {
    CmdBlock.Cmd[i] = Wire.read();
  }

  // fill all
  for (; i<maxCmdSize; i++) {
    CmdBlock.Cmd[i] = 0;
  }

  // New data available!!!
  CmdBlock.newCmd = true;

  cmdInterpreter();
  
}

void requestEvent() {
  // i2C-Interrupt to send data to master
  
  Wire.write( returnBuffer, returnBytes );
  
}

void activateErrorLED( void ) {
  Timer3.initialize( 100000 );
  Timer3.attachInterrupt( errorLEDTimer );
}

void errorLEDTimer( void ) {
  // blink 
  digitalWrite( LED, !digitalRead( LED ) );
}


unsigned long lastStep = 0;   // time of last times stepper timer

void loop() {

  unsigned long now;          // now, to calculate if a step could happen

  // check on keyboard event to change to maintenance mode
  if (Serial.available()){
    // set maintenance mode
    mode = MAINTENANCE;
    activateErrorLED();
    // clear keyboard buffer
    while (Serial.available()) {
      Serial.read();
    }
  }

  // check if reference voltage is in range
  if ( abs( maxMotorCurrent - getCurrent(100) ) > 0.05 ) {
    mode = MAINTENANCE;
    activateErrorLED();
  }

  if (mode == MAINTENANCE) {
    maintenanceLoop();
  }

  // Stepper Timer
  now = micros();
  if ( ( now < lastStep ) || ( now > lastStep + stepperInterval ) ) {
    lastStep = now;
    StepperTimer();
  }

  // Watchdog Timer
  // check only, if watchdog is active
  if ( watchdog > 0 ) {
    // check if watchdog-Time is reached
    if ( millis() > watchdog ) {
      // watchdog is reached, stop all motors and stop watchdog
      stopMovingAll( 0x0F );
      watchdog = -1;
    }

  }

}
