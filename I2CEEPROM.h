/*
 * \brief I2C EEPROMs data writer/reader library
 *
 * \author Quentin Comte-Gaz <quentin@comte-gaz.com>
 * \date 29 June 2016
 * \license MIT License (contact me if too restrictive)
 * \copyright Copyright (c) 2016 Quentin Comte-Gaz
 * \version 1.1
 *
 * \history
 *  - v1.0 Main design of the library
 *  - v1.1 Add Arduino < 1.0 compatibility
 */

#ifndef I2CEEPROM_h
#define I2CEEPROM_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// Enum to set the number of bits used in I2C address for the memory address for CAT24CXX devices.
// The "Classic" device sends the address and then 16-bits for the address.
// The CAT24CXX devices include the 'n' MSB of the MEMORY address as the 'n' LSB of the I2C address.
//    Where n is 3 for the 16 kb, 2 for the 8 kb, 1 for the 4 kb, and 0 for the 2kb
enum address_mode {
    ADDRESS_MODE_16BIT,
    ADDRESS_MODE_8BIT = 0,
    ADDRESS_MODE_9BIT = 1, 
    ADDRESS_MODE_10BIT = 2,
    ADDRESS_MODE_11BIT = 3
};

// Map device names to Addressing mode to make it easier to use
#define EEPROM_DEVICE_CLASSIC  ADDRESS_MODE_16BIT
#define EEPROM_DEVICE_CAT24C02 ADDRESS_MODE_8BIT
#define EEPROM_DEVICE_CAT24C04 ADDRESS_MODE_9BIT
#define EEPROM_DEVICE_CAT24C08 ADDRESS_MODE_10BIT
#define EEPROM_DEVICE_CAT24C16 ADDRESS_MODE_11BIT

class I2CEEPROM
{
  public:
    /*!
     * \brief I2CEEPROM Initialize I2C EEPROM instance
     * \param device_address (int) I2C address  of the EEPROM device
     * \param addressing_mode (enum) Selects the device in use (defaults to "Classic", non CAT24CXX)
     * \param initialize_wire (boolean) Initializes the Wire library (defaults to true)
     */
    I2CEEPROM(int i2c_device_address = 0x50, enum address_mode addressing_mode = EEPROM_DEVICE_CLASSIC, boolean initialize_wire = true);

    /*!
     * \brief write Write one byte \p data in EEPROM device at EEPROM internal address \p address
     * \param address (unsigned int) EEPROM internal address (most of the time, first address is 0x00)
     * \param data (byte) Byte to write at EEPROM internal address \p address
     */
    void write(unsigned int address, byte data) const;

    /*!
     * \brief read Read one byte in EEPROM device at EEPROM internal address \p address
     * \param address (unsigned int) EEPROM internal address (most of the time, first address is 0x00)
     * \return (byte) Read Byte at EEPROM internal address \p address (returns 0xFF if an error occurred)
     */
    byte read(unsigned int address) const;

  private:
    /*!
     * \brief read Combine I2C address with high byte of address for 512-2048 byte devices, return regular address for 16-bit addressed devices.
     * \param address (uint16_T) EEPROM internal address
     * \return (uint8_t) Read Byte at EEPROM internal address \p address (returns 0xFF if an error occurred)
     */
    uint8_t generate_I2C_address(uint16_t address) const;
  #ifdef CAT24CXX_ACK_POLLING
    /*!
     * \brief Perform CAT24Cxx device "ACK polling" to ensure availability of the device.
     * \param dev_address (uint8_t) CAT24Cxx I2C "device address" as returned from generate_I2C_address
     * \return (uint8_t) Return 0 if ack polling completed successfully, returns non-zero if it is unable to communicate with device.
     */
    uint8_t cat24cxx_ack_poll(uint8_t dev_address) const;
  #endif

  private:
    int _i2c_device_address;
    int _addressing_mode;
    int _i2c_address_mask;
};

#endif //I2CEEPROM_h

