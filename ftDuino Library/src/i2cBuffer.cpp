#include <Wire.h>
#include "i2cBuffer.h"

void i2cBuffer::sendBuffer( uint8_t address  ) {
  // send data

/*
  Serial.print("sendBuffer "); Serial.print( address ); Serial.print(" ");
  for (int i=0; i<len; i++ ) {
    Serial.print( data[i], HEX );
    Serial.print(" ");
  }
  Serial.println();
*/  
  Wire.beginTransmission( address );
  Wire.write( data, len );
  Wire.endTransmission();
}

void i2cBuffer::receiveBuffer( uint8_t address, uint8_t quantity ) {
  // receive data

//  Serial.print( "receiveBuffer( " ); Serial.print( address ), Serial.print(","); Serial.print( quantity ); Serial.println(")");

  len = 0;

  // request quantity uint8_ts
  Wire.requestFrom( address, quantity);

  uint8_t x;
  // receive data
  while (Wire.available()) { 
    x = Wire.read();
    data[len++] = x;
//    Serial.print( x, HEX ); 
//    Serial.print( " " );
  }
//  Serial.println();
  
  // fillup buffer
  for (uint8_t i=len;i<16;i++) {
    data[i] = 0;
  }

}

void i2cBuffer::push( uint8_t v ) {
  // writes a uint8_t into the buffer
  data[len++] = v;
}

void i2cBuffer::push( long v ) {
  // writes a long into the buffer
  memcpy( &data[len], &v, sizeof(v) );
  len += sizeof(v);
}

long i2cBuffer::pop( uint8_t pos ) {
  // reads a long out of the buffer
  long v;
  memcpy( &v, &data[pos], sizeof(v) );
  return v;
}

void i2cBuffer::sendData( uint8_t address, uint8_t cmd ) {
  // send a command
  len = 0;
  push( cmd );
  sendBuffer( address );
}

void i2cBuffer::sendData( uint8_t address, uint8_t cmd, uint8_t v1 ) {
  // send a command with a long value
  len = 0;
  push( cmd );
  push( v1 );
  sendBuffer( address );
}

void i2cBuffer::sendData( uint8_t address, uint8_t cmd, long v1 ) {
  // send a command with a long value
  len = 0;
  push( cmd );
  push( v1 );
  sendBuffer( address );
}

void i2cBuffer::sendData( uint8_t address, uint8_t cmd, uint8_t v1, long v2 ) {
  // send a command with a uint8_t and a long value
  len = 0;
  push( cmd );
  push( v1 );
  push( v2 );
  sendBuffer( address );
}

void i2cBuffer::sendData( uint8_t address, uint8_t cmd,long v1, long v2, long v3, long v4 ) {
  // send a command with 4 long values
  
  len = 0;
  push( cmd );
  push( v1 );
  push( v2 );
  push( v3 );
  push( v4 );

  sendBuffer( address );
}

uint8_t i2cBuffer::receiveuint8_t( uint8_t address, uint8_t cmd ) {
  // receive a uint8_t value 
  sendData( address, cmd );
  receiveBuffer( address, 1 );
  return data[0];
}

uint8_t i2cBuffer::receiveuint8_t( uint8_t address, uint8_t cmd, uint8_t v1 ) {
  // receive a uint8_t value 
  sendData( address, cmd, v1 );
  receiveBuffer( address, 1 );
  return data[0];
}

long i2cBuffer::receiveLong( uint8_t address, uint8_t cmd, uint8_t v1 ) {
  // receive a long value 
  sendData( address, cmd, v1 );
  receiveBuffer( address, 4 );
  return pop( 0 );
}

void i2cBuffer::receive4Long( uint8_t address, uint8_t cmd, long &v1, long &v2, long &v3, long &v4 ) {
  // receive 4 long values
  sendData( address, cmd );
  receiveBuffer( address, 16 );
  v1 = pop( 0 );
  v2 = pop( 4 );
  v3 = pop( 8 );
  v4 = pop( 12 );
} 
