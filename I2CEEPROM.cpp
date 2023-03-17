/*
 * \brief I2C EEPROMs data writer/reader (implementation)
 *
 * \author Quentin Comte-Gaz <quentin@comte-gaz.com>
 * \date 15 January 2023
 * \license MIT License (contact me if too restrictive)
 * \copyright Copyright (c) 2023 Quentin Comte-Gaz
 * \version 1.3
 */

#include "I2CEEPROM.h"
#include <Wire.h>

I2CEEPROM::I2CEEPROM(int i2c_device_address, enum address_mode addressing_mode, bool initialize_wire)
{
  _i2c_device_address = i2c_device_address;
  _addressing_mode = addressing_mode;

  // Set the address mask (as defined in enum) to properly set I2C address when writing/reading
  if (addressing_mode == ADDRESS_MODE_16BIT)
  {
    _i2c_address_mask = 0xFF;
  }
  else
  {
    _i2c_address_mask = (0xFF << addressing_mode);
  }
  
  // Compatibility -- defaults to true to automatically initialize the Wire library, but it may be unwanted by some users.
  if (initialize_wire)
  {
    Wire.begin();
  }
}

void I2CEEPROM::write(unsigned int address, byte data) const
{
  // Generate the proper I2C address based on device mask and memory address
  uint8_t dev_address = generate_I2C_address(address);

#ifdef CAT24CXX_ACK_POLLING
  if(cat24cxx_ack_poll(dev_address)) return;
#endif

  Wire.beginTransmission(dev_address);

  // In classic device mode, send the 8 MSB of the memory address.
  // This is already part of the device addr in CAT24CXX
  if (_addressing_mode == ADDRESS_MODE_16BIT)
  {
    Wire.write((int)(address >> 8));    // First part of the address (MSB)
    Wire.write((int)(address & 0xFF));  // Second part of the address (LSB)
  }
  else
  {
    Wire.write((byte)(address & 0xFF));
  }

  Wire.write(data);                     // Write byte
  Wire.endTransmission();

#ifndef CAT24CXX_ACK_POLLING
  // Writing in I2C EEPROM takes ~5ms (even if I2C writing already done) -- Run only if ACK polling isn't enabled
  delay(5);
#endif
}

byte I2CEEPROM::read(unsigned int address) const
{
  // Generate the proper I2C address based on device mask and memory address
  uint8_t dev_address = generate_I2C_address(address);
  byte read_data = 0xFF;

#ifdef CAT24CXX_ACK_POLLING
  if(cat24cxx_ack_poll(dev_address)) return 0xFF;
#endif

  Wire.beginTransmission(dev_address);

  // In classic device mode, send the 8 MSB of the memory address.
  // This is already part of the device addr in CAT24CXX
  if (_addressing_mode == ADDRESS_MODE_16BIT)
  {
    Wire.write((int)(address >> 8));   // MSB
    Wire.write((int)(address & 0xFF)); // LSB
  }
  else
  {
    Wire.write((byte)(address & 0xFF));
  }

  Wire.endTransmission();

  // Request 1 byte from device
  Wire.requestFrom((uint8_t)dev_address, (uint8_t)1);

  // Give it a couple of opportunities to read
  int loopbreak = 16;
  while(!Wire.available())
  {
      if(!loopbreak--) return 0xFF;
      delayMicroseconds(1);
  }
  
  read_data = Wire.read();

  return read_data;
}

void I2CEEPROM::update(unsigned int address, byte data) const
{
  if (read(address) != data)
  {
    write(address, data);
  }
  else
  {
    // Data to write is the same as data already written
    // => No need to write !
  }
}

uint8_t I2CEEPROM::generate_I2C_address(uint16_t address) const 
{
  uint8_t dev_address = _i2c_device_address;

  if (_addressing_mode != ADDRESS_MODE_16BIT)
  {
    // Set the N MSB of the address to the N LSB of the high-byte of the address
    dev_address = (_i2c_device_address & _i2c_address_mask) | ((address >> 8) & (~_i2c_address_mask));
  }

  return dev_address;
}

#ifdef CAT24CXX_ACK_POLLING
uint8_t I2CEEPROM::cat24cxx_ack_poll(uint8_t dev_address) const
{
  // At 400kHz, one byte should take 1/50,000 sec = 20 us, so 5 ms = 5/0.02 = 250 attempts =>  Factor of 400000/250 = 1600.
  int ackpollmax = CAT24CXX_ACK_POLLING / 1500; // max number of attempts at ack polling, run slightly more times to be sure.
  uint8_t rv;
  do {
      Wire.beginTransmission(dev_address);
      rv = Wire.endTransmission();
      if(!ackpollmax--) return -1;
  } while(rv == 2); // From Arduino docs -- 2: received NACK on transmit of address.
           
  return 0;
}
#endif
