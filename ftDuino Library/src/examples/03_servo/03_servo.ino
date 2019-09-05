
////////////////////////////////////////////////////////////////
//
// ftPwrDrive - Example Application for Arduino/ftDuino boards
//
// Working with servocs
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
  Serial.println( "running servos." );

}

void loop() {
  // wait forever, we run already everything in setup

  for ( int i=-30; i<=30; i++ ) {
    Drive.setServo( 0, i );
    delay( 200 );
    Drive.setServo( 1, i );
    delay( 200 );
    Drive.setServo( 2, i );
    delay( 200 );
    Drive.setServo( 3, i );
    delay( 200 );
  }
    
 // delay(1000);
}
