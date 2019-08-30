
////////////////////////////////////////////////////////////////
//
// ftPwrDrive - Example Application for Arduino/ftDuino boards
//
// Working with motor constants and motor array
//
// 22.08.2019 / V1.00
//
// (C) 2019 Christian Bergschneider & Stefan Fuss
//
////////////////////////////////////////////////////////////////

#include <Wire.h>
// just include the ftPwrDrive header files
#include <ftPwrDrive.h>

// we need some serial output, so we include wire.h, too


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

void run_sequential( void ) {

  int i;
  
  // Run all motors 1000 steps
  Serial.println();
  Serial.println( "All steppers are moving 1000 steps one after another: ");

  // start positions...
  print_StepsToGo();
  
  // every motor
  for (i=0; i<FTPWRDRIVE_MOTORS; i++) {
 
    // never use Drive.setRelDistance( i, 1000 ); because i is not equal to FTPWRDRIVE_M[i] !
    
    Drive.setRelDistance( FTPWRDRIVE_M[i], 1000 );
    Drive.startMoving( FTPWRDRIVE_M[i] );
    
    while (Drive.isMoving( FTPWRDRIVE_M[i] )) {
      
      print_StepsToGo();
      delay(100);
      
    }
    
  }

  print_StepsToGo();

}

void run_parallel( void ) {

  int i;
  
  // Run all motors 100 steps parallel
  Serial.println();
  Serial.println( "All steppers are moving 1000 steps parallel, M3 starts 100ms late: ");

  for (i=0; i<FTPWRDRIVE_MOTORS; i++ ) {
    Drive.setRelDistance( FTPWRDRIVE_M[i], 1000 );
  }

  // start positions...
  Serial.print( Drive.isMovingAll(), BIN ); Serial.print( " " );
  print_StepsToGo();

  // Start M1, M2, M4
  Drive.startMovingAll( FTPWRDRIVE_M1 | FTPWRDRIVE_M2 | FTPWRDRIVE_M4 );

  delay(100);

  // run loop since any montor is running
  while ( Drive.isMovingAll() ) {
    
    delay(100);

    Serial.print( Drive.isMovingAll(), BIN ); Serial.print( " " );
    print_StepsToGo();

    if (!Drive.isMoving( FTPWRDRIVE_M3 ) ) {
      Drive.startMoving( FTPWRDRIVE_M3 );
      delay(100);
    }
    
  }
 
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

  // set stepping type
  Drive.setMicrostepMode( FTPWRDRIVE_FULLSTEP );

  // set normal speed
  Drive.setMaxSpeed( FTPWRDRIVE_M1, 500);
  Drive.setMaxSpeed( FTPWRDRIVE_M2, 500);
  Drive.setMaxSpeed( FTPWRDRIVE_M3, 500);
  Drive.setMaxSpeed( FTPWRDRIVE_M4, 500);

  // run motors by each
  run_sequential();

  // run motors in parallel
  run_parallel();

}

void loop() {
  // wait forever, we run already everything in setup
  delay(1000);
}
