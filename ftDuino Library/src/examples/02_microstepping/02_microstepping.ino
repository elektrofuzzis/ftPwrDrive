
////////////////////////////////////////////////////////////////
//
// ftPwrDrive - Example Application for Arduino/ftDuino boards
//
// Working with microstepping
//
// 22.08.2019 / V1.00
//
// (C) 2019 Christian Bergschneider & Stefan Fuss
//
////////////////////////////////////////////////////////////////

// just include the ftPwrDrive header files
#include <ftPwrDrive.h>

// we need some serial output, so we include wire.h, too
#include <Wire.h>

// create an instance of ftPwrDrive and set the I2C address
ftPwrDrive Drive = ftPwrDrive(32);

void print_StepsToGo( void ) {
  // print the distance all motors have to go

  char buffer[200];

  sprintf( buffer, 
           "stepsToGo: M1: %5ld M2: %5ld M3: %5ld M4: %5ld | getPosition: M1: %5ld M2: %5ld M3: %5ld M4: %5ld",
           Drive.getStepsToGo( FTPWRDRIVE_M1 ), Drive.getStepsToGo( FTPWRDRIVE_M2 ), Drive.getStepsToGo( FTPWRDRIVE_M3 ), Drive.getStepsToGo( FTPWRDRIVE_M4 ),
           Drive.getPosition( FTPWRDRIVE_M1 ), Drive.getPosition( FTPWRDRIVE_M2 ), Drive.getPosition( FTPWRDRIVE_M3 ), Drive.getPosition( FTPWRDRIVE_M4 )
         );
  
  Serial.print( buffer );
  Serial.println( );
  
}

void run_M1( void ) {

  int i;
  
  // Run all motors 1000 steps
  Serial.println( "M1 is now moving 1000 steps: ");

  // start positions...
  print_StepsToGo();
  
  Drive.setRelDistance( FTPWRDRIVE_M1, 1000 );
  Drive.startMoving( FTPWRDRIVE_M1 );
    
  while (Drive.isMoving( FTPWRDRIVE_M1 )) {
      
      print_StepsToGo();
      delay(100);
      
  }
    

  print_StepsToGo();

}


void setup() {

  // wait for the serial monitor
  while (!Serial);
  Serial.begin(9600);

  // Wire.begin();

  // here we are
  Serial.println( "ftPwrDrive example application.");
  Serial.println( "running motors each by each and parallel." );

  // don't know how often this program is already started... Reset motor positon
  Drive.setPositionAll( 0, 0, 0, 0 );

  // set normal speed
  Drive.setMaxSpeed( FTPWRDRIVE_M1, 500);
  Drive.setMaxSpeed( FTPWRDRIVE_M2, 500);
  Drive.setMaxSpeed( FTPWRDRIVE_M3, 500);
  Drive.setMaxSpeed( FTPWRDRIVE_M4, 500);

  // test stepping types
  
  Serial.println( "Working in fullstep mode:" );
  Drive.setMicrostepMode( FTPWRDRIVE_FULLSTEP );
  run_M1();

  Serial.println( "Working in halfstep mode:" );
  Drive.setMicrostepMode( FTPWRDRIVE_HALFSTEP );
  run_M1();

  Serial.println( "Working in quarterstep mode:" );
  Drive.setMicrostepMode( FTPWRDRIVE_QUARTERSTEP );
  run_M1();

  Serial.println( "Working in eigthstep mode:" );
  Drive.setMicrostepMode( FTPWRDRIVE_EIGTHSTEP );
  run_M1();

  Serial.println( "Working in sixteenthstep mode:" );
  Drive.setMicrostepMode( FTPWRDRIVE_SIXTEENTHSTEP );
  run_M1();

}

void loop() {
  // wait forever, we run already everything in setup
  delay(1000);
}
