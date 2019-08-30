#ifndef i2cBuffer_h
#define i2cBuffer_h

#include <Arduino.h>

class i2cBuffer {
  public:
    uint8_t data[32];
    uint8_t len = 0;
    void push( uint8_t v );
      // writes a uint8_t into the buffer
    void push( long v );
      // writes a long into the buffer
    long pop( uint8_t pos );
      // reads a long out of the buffer
    void sendData( uint8_t address, uint8_t cmd );
      // send a command 
    void sendData( uint8_t address, uint8_t cmd, long v1 );
      // send a command with a long value
    void sendData( uint8_t address, uint8_t cmd, uint8_t v1 );
      // send a command with a uint8_t 
    void sendData( uint8_t address, uint8_t cmd, uint8_t v1, long v2 );
      // send a command with a uint8_t and a long value
    void sendData( uint8_t address, uint8_t cmd, long v1, long v2, long v3, long v4 );
      // send a command with 4 long values
    uint8_t receiveuint8_t( uint8_t address, uint8_t cmd );
      // receive a uint8_t value 
    uint8_t receiveuint8_t( uint8_t address, uint8_t cmd, uint8_t v1 );
      // receive a uint8_t value 
    long receiveLong( uint8_t address, uint8_t cmd, uint8_t v1 );
      // receive a long value 
    void receive4Long( uint8_t address, uint8_t cmd, long &v1, long &v2, long &v3, long &v4 );
      // receive 4 long values
    void sendBuffer( uint8_t address );
      // send data
    void receiveBuffer( uint8_t address, uint8_t quantity );
};

#endif
