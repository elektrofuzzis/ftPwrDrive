#include <Arduino.h>
#include <SPI.h>
#include "HC595.h"

uint8_t Register595 = 0;

void write595( uint8_t b ) {
  // write 1 uint8_t via SPI
  digitalWrite( SS, LOW );  // CS low
  SPI.transfer( b );        // write uint8_t
  digitalWrite( SS, HIGH ); // CS low->high write b to output
  Register595 = b;         // remember me

}

void write595( uint8_t bit, uint8_t value ) {
  // Set/Reset a bit and write data to device
  uint8_t  mask = 1<<bit;
  uint8_t imask = ~mask;

  write595( ( Register595 & imask ) | ( ( value & 1 ) << bit ) );
}
