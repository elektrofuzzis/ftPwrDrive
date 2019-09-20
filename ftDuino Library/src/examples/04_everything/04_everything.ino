
////////////////////////////////////////////////////////////////
//
// ftPwrDrive - Example Application for Arduino/ftDuino boards
//
// Working with everything
//
// 05.09.2019 / V1.00
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


void setup() {

  // wait for the serial monitor
  while (!Serial);
  Serial.begin(9600);

  // Wire.begin();

  // here we are
  Serial.println( "ftPwrDrive example application.");
  Serial.println( "running all stuff." );

}

int i=0, j=0;
boolean S[4] = { false, false, false, false };

void loop() {

  // the loop turns the servo pins alternating on/off, displays the state of the end stop and emergency stop

  // get the next servo
  i++;
  if (i>3) {
    i=0;
  }

  // change state of choosen servo
  S[i] = !S[i];
  Drive.setServoOnOff( i, S[i] );

  // check if motor i is running and start 100 steps if needed
  if ( !Drive.isMoving( FTPWRDRIVE_M[j] ) ) {
    j++;
    if (j>3) {
      j=0;
    }
    Drive.setRelDistance( FTPWRDRIVE_M[j], 100 );
    Drive.startMoving( FTPWRDRIVE_M[j] );
  }

  // display all endStops
  char buffer[200];
  sprintf( buffer,
           "EndStop1:%d EndStop2:%d EndStop3:%d EndStop4:%d EmergencyStop:%d",
           Drive.endStopActive( FTPWRDRIVE_M1 ),
           Drive.endStopActive( FTPWRDRIVE_M2 ),
           Drive.endStopActive( FTPWRDRIVE_M3 ),
           Drive.endStopActive( FTPWRDRIVE_M4 ),
           Drive.emergencyStopActive( ));
  Serial.println( buffer );

  delay(250);

}
