#include "defs.h"

int orBits(int bit, ...)
{
  if (bit == -1) return 0;
  int result = 1<<bit;
  va_list vl;
  va_start(vl,bit);
  while ((bit = va_arg(vl, int)) != -1) {
    result |= 1<<bit;
  }
  return result;
}

#define cbi(sfr, bit) (*sfr &= ~_BV(bit))
#define sbi(sfr, bit) (*sfr |= _BV(bit))
#define vbi(sfr, bit) (*sfr & _BV(bit))

void AtmelPort::input(bool pullup) const
{
  cbi(ddr, pno); // Input direction
  cbi(port, pno); // First select tristate
  if (pullup) sbi(port, pno); // Then enable pull up
}

bool AtmelPort::read() const
{
  return vbi(pin, pno)!=0;
}

void AtmelPort::output(bool high) const
{
  sbi(ddr, pno); // Input direction
  cbi(port, pno); // First select output low
  if (high) sbi(port, pno); // Then enable pull up
}

void AtmelPort::write(bool high) const
{
  if (high) {
    sbi(port, pno); // Set output bit
  } else {
    cbi(port, pno); // Clear output bit
  }
}

void crc16Update(uint16_t& crc, uint8_t a)
{
  byte i;
  crc ^= a;
  for (i = 0; i < 8; ++i)
  {
    if (crc & 1)
      crc = (crc >> 1) ^ 0xA001;
    else
      crc = (crc >> 1);
  }
}

uint16_t calcCRC16(const byte* data, byte length)
{
  // append CRC
  uint16_t u16CRC = 0xFFFF;
  while (length-- > 0) {
    crc16Update(u16CRC, *data++);
  }
  return u16CRC;
}


