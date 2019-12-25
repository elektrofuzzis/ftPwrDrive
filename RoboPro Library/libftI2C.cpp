//////////////////////////////////////////////////////////////////////////////////////////
//
// ftI2C Library
//
// Send and Receive complex I2C Cmds with less bugs with fischertechnik TX/TXT Controller
//
// Version 1.00
//
// (C) 2019 Christian Bergschneider & Stefan Fuss - elektrofuzzis
//
// compile to libftI2C.so and copy it as User ROBOPRO to /opt/knobloch
//
/////////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <string.h>
#include <KeLibTxtDl.h>          // TXT Lib
#include <FtShmem.h>             // TXT Transfer Area

#define VERSION   1
#define MAXBUFFER 255

uint8_t  buffer[MAXBUFFER];
uint8_t  bufferPtr  = -1;
uint16_t i2cAddress = 0x20; 
uint16_t i2cSpeed   = I2C_SPEED_400_KHZ;

extern "C" {

  // Return value:
  //  0: success, continue with waiting for pFinishVar becoming 1
  //  1: not finished
  //  2: busy (entity locked by other process)
  // -1: error
  // Other positive values can be used for other waiting codes
  // Other negative values can be used for other error codes

  int init(short* t)
  {
	bufferPtr  = 0;
	i2cAddress = 0x20;
	i2cSpeed   = I2C_SPEED_400_KHZ;
	*t         = VERSION;
    return 0;
  }
  
  int setI2CAddress(short address)
  // sets the I2CAddress
  {
	  i2cAddress = address;
	  
	  return 0;
  }

  int getI2CAddress(short *address)
  // gets the I2CAddress
  {
	  *address = i2cAddress;
	  
	  return 0;
  }
    
  int setI2CSpeed(short speed)
  // sets the I2CSpeed
  {
	  i2cSpeed = speed;
	  
	  return 0;
  }

  int getI2CSpeed(short *speed)
  // gets the I2CSpped
  {
	  *speed = i2cSpeed;
	  
	  return 0;
  }
  
  int setBufferPointer(short ptr)
  // sets the bufferPtr
  {
	  bufferPtr = ptr;
	  
	  return 0;
  }
  
  int getBufferPointer(short *ptr)
  // gets the BufferPointer
  {
	  *ptr = bufferPtr;
	  
	  return 0;
  }
  
  int pushLong(double v)
  // push a long value - ROBOPro must use double
  {   
	  long longvalue = (long) v;
	  
	  memcpy( &buffer[bufferPtr], &longvalue, sizeof(long) );
	  bufferPtr = bufferPtr + sizeof(long);

	  return 0;
  }

  int pushByte(short v)
  // push a byte value 
  {   
	  // cast to byte
	  uint8_t bytevalue = (uint8_t) v;
	  
	  // copy to buffer
	  memcpy( &buffer[bufferPtr], &bytevalue, sizeof(uint8_t) );
	  bufferPtr = bufferPtr + sizeof(uint8_t);

	  return 0;
  }
  
  int pushShort(short v)
  // push a short value
  {
	  // copy to buffer
	  memcpy( &buffer[bufferPtr], &v, sizeof(short) );
	  bufferPtr = bufferPtr + sizeof(short);

	  return 0;
  }
  
  int popLong(double* v)
  // pop a long value
  {   
	  long longvalue;
	  
	  memcpy( &longvalue, &buffer[bufferPtr], sizeof(long) );
	  *v = (double) longvalue;
	  bufferPtr = bufferPtr + sizeof(long);
	  
	  return 0;
  }

  int popByte(short* v)
  // pop a byte value
  {
	  uint8_t bytevalue;
	  
	  // copy 1 byte from array to bytevalue
	  memcpy( &bytevalue, &buffer[bufferPtr], sizeof(uint8_t) );
	  bufferPtr = bufferPtr + sizeof(uint8_t);
	  
	  // cast bytevalue to short parameter
	  *v = (short) bytevalue;
	  
	  return 0;
  }
  
  int popShort(short* v)
  // pop a short value
  {
	  memcpy( v, &buffer[bufferPtr], sizeof(short) );
	  bufferPtr = bufferPtr + sizeof(short);
	  
	  return 0;
  }

  int I2CSendBuffer(short ignore)
  {
	  return KeLibI2cTransfer(i2cAddress, bufferPtr, buffer, 0, 0, i2cSpeed);  
  }

  int I2CReceiveBuffer(short bytes)
  {
	  bufferPtr = 0;
	  return KeLibI2cTransfer(i2cAddress, 0, 0, bytes, buffer, i2cSpeed);
  }

} // extern "C"