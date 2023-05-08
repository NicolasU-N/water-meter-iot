#ifndef EEPROM_24LC256_H
#define EEPROM_24LC256_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "driver/i2c.h"

#define EEPROM_ADDR 0x50    /*!< Slave address of the MPU9250 sensor */
#define I2C_MAX_DATA_LEN 16 /*!< I2C master doesn't need buffer */
#define EEPROM_SIZE 32768

esp_err_t eeprom_write_byte(uint16_t address, uint8_t data);
esp_err_t eeprom_write_string(uint16_t addr, const char *data);
void eeprom_read_string(uint16_t address, char *buffer, int buffer_size);
esp_err_t eeprom_write_long(uint16_t address, long value);
esp_err_t eeprom_read_long(uint16_t address, long *value);
esp_err_t eeprom_update_long(uint16_t address, long value);
esp_err_t eeprom_read_float(uint16_t address, float *value);
esp_err_t eeprom_write_float(uint16_t address, float value);
esp_err_t eeprom_update_float(uint16_t address, float value);
esp_err_t eeprom_erase_all(void);

#endif
