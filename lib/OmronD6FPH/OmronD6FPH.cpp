#include "OmronD6FPH.h"

OmronD6FPH::OmronD6FPH(void){
  // Constructor
}

void  OmronD6FPH::initialize(uint8_t address)
{
  _address = address;
  Wire.beginTransmission(address);
  Wire.write(0x0B); 
  Wire.write(0x00);
  int x = Wire.endTransmission();
}

int OmronD6FPH::flow(int rangeMode)
{
  Wire.beginTransmission(_address);
  Wire.write(0x00);   
  Wire.write(0xD0);  // reg 0 - address register high byte
  Wire.write(0x51);  // reg 1 - address register low byte
  Wire.write(0x18);  // reg 2  - serial control register - indicate # bytes among others (page 7 bottom)
  Wire.write(0x06);  // reg 3 - value to be written to SENS control register
  int x = Wire.endTransmission();

  delay(33);

  Wire.beginTransmission(_address);
  Wire.write(0x00);   
  Wire.write(0xD0);
  Wire.write(0x51);
  Wire.write(0x2C);
  Wire.write(0x07);
  x = Wire.endTransmission();

  Wire.requestFrom(_address, 2); 
  uint8_t hibyte = Wire.read();
  uint8_t lobyte = Wire.read();
  long raw = (hibyte << 8) | lobyte;
  //long raw = word( hibyte, lobyte); 
  int rd_flow = (raw - 1024) * rangeMode * 10 / 60000L;

  return rd_flow;
}

int OmronD6FPH::temperature()
{
  Wire.beginTransmission(_address);
  Wire.write(0x00);   
  Wire.write(0xD0);  // reg 0 - address register high byte
  Wire.write(0x51);  // reg 1 - address register low byte
  Wire.write(0x18);  // reg 2  - serial control register - indicate # bytes among others (page 7 bottom)
  Wire.write(0x06);  // reg 3 - value to be written to SENS control register
  int x = Wire.endTransmission();

  delay(33);

  Wire.beginTransmission(_address);
  Wire.write(0x00);   
  Wire.write(0xD0);
  Wire.write(0x61);
  Wire.write(0x2C);
  Wire.write(0x07);
  x = Wire.endTransmission();

  Wire.requestFrom(_address, 2); 
  uint8_t hibyte = Wire.read();
  uint8_t lobyte = Wire.read();
  long raw = (hibyte << 8) | lobyte;
  int temp = (raw - 10214) * 1000L / 3739L;
  return temp;
}