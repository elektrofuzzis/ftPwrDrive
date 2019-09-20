#ifndef HC595_h
#define HC595_h

#include <Arduino.h>

void write595( uint8_t b );
  // write 1 uint8_t via SPI
  
void write595( uint8_t bit, uint8_t value );
  // Set/Reset a bit and write data to device

#endif
