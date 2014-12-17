/*
  TLV5618.cpp
  
  Arduino library for Texas Instruments TLV5618 2-channel 12-bit SPI DAC
  Partially based on the Wiblocks library http://wiblocks.luciani.org/src/lib/DAC/classDAC__TLV5618.html
  
  2012-09-29 @machinesalem,  (cc) https://creativecommons.org/licenses/by/3.0/
*/

#include "SPI.h"
#include "TLV5618.h"


TLV5618::TLV5618( uint8_t cs_pin )
{
  _cs_pin = cs_pin;
  _control = TLV5618_SPEED_SLOW | TLV5618_POWER_NORM;
};

TLV5618::TLV5618( uint8_t cs_pin, uint8_t control )
{
  _cs_pin = cs_pin;
  _control = control;
};

    
void TLV5618::begin()
{
  /* Initialize SPI */
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV8);  // 2 MHz clock (you could try other speeds)
  
  /* !Chip select (low to enable) */
  pinMode(_cs_pin, OUTPUT);
  digitalWrite(_cs_pin,  1);
};


/* Write using one of the TLV5618_CMD_xxx */
void TLV5618::write_data( uint8_t cmd, uint16_t value )
{
  uint8_t data1, data2;
  cmd &= TLV5618_CMD_MASK;
  cmd |= _control;
  data1 = (uint8_t)(((value & 0xF00)>>8) | cmd );
  data2 = (uint8_t )(value & 0xFF);
  
  digitalWrite(_cs_pin, 0);
  delayMicroseconds(10); 
  SPI.transfer(data1);
  SPI.transfer(data2);
  delayMicroseconds(10); 
  digitalWrite(_cs_pin,  1);
};


/* Convenient method to write channels A and B at the same time */
void TLV5618::write( uint16_t valueA, uint16_t valueB )
{
  write_data( TLV5618_CMD_WRITE_BUFFER, valueA );
  write_data( TLV5618_CMD_WRITE_A_UPDATE_B, valueB );
};

