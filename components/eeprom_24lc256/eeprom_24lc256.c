#include "eeprom_24lc256.h"

/**
 * @brief Write a byte to the specified address of EEPROM
 *
 * @param address Address to write
 * @param data Byte to write
 *
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t eeprom_write_byte(uint16_t address, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (EEPROM_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, address >> 8, true);
    i2c_master_write_byte(cmd, address & 0xFF, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

void eeprom_read_string(uint16_t address, char *buffer, int buffer_size)
{
    int i = 0;
    while (i < buffer_size)
    {
        int remaining_bytes = buffer_size - i;
        int bytes_to_read = remaining_bytes > I2C_MAX_DATA_LEN ? I2C_MAX_DATA_LEN : remaining_bytes;
        uint8_t write_data[2] = {address >> 8, address & 0xFF};
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, EEPROM_ADDR << 1 | I2C_MASTER_WRITE, true);
        i2c_master_write(cmd, write_data, 2, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, EEPROM_ADDR << 1 | I2C_MASTER_READ, true);
        i2c_master_read(cmd, (uint8_t *)&buffer[i], bytes_to_read, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (ret != ESP_OK)
        {
            printf("Failed to read string from EEPROM, err = %d\n", ret);
            break;
        }
        address += bytes_to_read;
        i += bytes_to_read;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    buffer[i] = '\0';
}

/**
 * @brief write a string to the specified address of EEPROM
 *
 * @param addr
 * @param data
 * @return esp_err_t
 */
esp_err_t eeprom_write_string(uint16_t addr, const char *data)
{
    uint8_t len = strlen(data);
    uint8_t *buffer = (uint8_t *)calloc(len + 1, sizeof(uint8_t));
    buffer[0] = len;
    memcpy(buffer + 1, data, len);
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, EEPROM_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (addr >> 8) & 0xff, true);
    i2c_master_write_byte(cmd, addr & 0xff, true);
    i2c_master_write(cmd, buffer, len + 1, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    free(buffer);
    return ret;
}

/**
 * @brief write a long value to the specified address of EEPROM
 *
 * @param address
 * @param value
 * @return esp_err_t
 */
esp_err_t eeprom_write_long(uint16_t address, long value)
{
    esp_err_t ret;
    uint8_t data[4];

    data[0] = (value >> 24) & 0xFF;
    data[1] = (value >> 16) & 0xFF;
    data[2] = (value >> 8) & 0xFF;
    data[3] = value & 0xFF;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, EEPROM_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, address >> 8, true);
    i2c_master_write_byte(cmd, address & 0xFF, true);
    i2c_master_write(cmd, data, 4, true);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief read a long value from the specified address of EEPROM
 *
 * @param address
 * @param value
 * @return esp_err_t
 */
esp_err_t eeprom_read_long(uint16_t address, long *value)
{
    esp_err_t ret;
    uint8_t data[4];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, EEPROM_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, address >> 8, true);
    i2c_master_write_byte(cmd, address & 0xFF, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, EEPROM_ADDR << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 4, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK)
    {
        *value = ((long)data[0] << 24) | ((long)data[1] << 16) | ((long)data[2] << 8) | (long)data[3];
    }

    return ret;
}

/**
 * @brief update a long value in the specified address of EEPROM
 *
 * @param address
 * @param value
 * @return esp_err_t
 */
esp_err_t eeprom_update_long(uint16_t address, long value)
{
    long current_value;
    esp_err_t ret = eeprom_read_long(address, &current_value);

    if (ret == ESP_OK && current_value != value)
    {
        ret = eeprom_write_long(address, value);
    }

    return ret;
}

/**
 * @brief read a float value from the specified address of EEPROM
 *
 * @param address
 * @param value
 * @return esp_err_t
 */
esp_err_t eeprom_read_float(uint16_t address, float *value)
{
    esp_err_t ret;
    uint8_t data[4];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, EEPROM_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, address >> 8, true);
    i2c_master_write_byte(cmd, address & 0xFF, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, EEPROM_ADDR << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 4, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK)
    {
        *value = *((float *)data);
    }

    return ret;
}

/**
 * @brief write a float value to the specified address of EEPROM
 *
 * @param i2c_port
 * @param address
 * @param value
 * @return esp_err_t
 */
esp_err_t eeprom_write_float(uint16_t address, float value)
{
    esp_err_t ret;
    uint8_t *data = (uint8_t *)&value;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, EEPROM_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, address >> 8, true);
    i2c_master_write_byte(cmd, address & 0xFF, true);
    i2c_master_write(cmd, data, 4, true);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief update a float value in the specified address of EEPROM
 *
 * @param i2c_port
 * @param address
 * @param value
 * @return esp_err_t
 */
esp_err_t eeprom_update_float(uint16_t address, float value)
{
    float current_value;
    esp_err_t ret = eeprom_read_float(address, &current_value);

    if (ret == ESP_OK && current_value != value)
    {
        ret = eeprom_write_float(address, value);
    }

    return ret;
}

esp_err_t eeprom_erase_all(void)
{
    uint8_t erase_buf[32] = {0};                  // 32 bytes buffer to write zeros
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // create command link

    // Iterate over all EEPROM pages and write zeros to them
    for (int page = 0; page < (EEPROM_SIZE / 32); page++)
    {
        i2c_master_start(cmd);                                                   // send start bit
        i2c_master_write_byte(cmd, (EEPROM_ADDR << 1) | I2C_MASTER_WRITE, true); // write EEPROM address
        i2c_master_write_byte(cmd, (page * 32) >> 8, true);                      // write high byte of memory address
        i2c_master_write_byte(cmd, (page * 32) & 0xFF, true);                    // write low byte of memory address

        // Write 32 zeros to the current EEPROM page
        for (int i = 0; i < 32; i++)
        {
            i2c_master_write_byte(cmd, erase_buf[i], true);
        }

        i2c_master_stop(cmd); // send stop bit

        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
        if (ret != ESP_OK)
        {
            i2c_cmd_link_delete(cmd); // delete command link
            return ret;
        }

        // Wait for EEPROM write cycle to complete
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    i2c_cmd_link_delete(cmd); // delete command link
    return ESP_OK;
}

// void eeprom_write_string(i2c_port_t i2c_port, uint16_t address, const char *string)
//{
//	int str_len = strlen(string);
//	int i = 0;
//	while (i < str_len)
//	{
//		int remaining_bytes = str_len - i;
//		int bytes_to_write = remaining_bytes > I2C_MAX_DATA_LEN ? I2C_MAX_DATA_LEN : remaining_bytes;
//		uint8_t write_data[I2C_MAX_DATA_LEN + 2];
//		write_data[0] = address >> 8;
//		write_data[1] = address & 0xFF;
//		memcpy(&write_data[2], &string[i], bytes_to_write);
//		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//		i2c_master_start(cmd);
//		i2c_master_write_byte(cmd, EEPROM_ADDR << 1 | I2C_MASTER_WRITE, true);
//		i2c_master_write(cmd, write_data, bytes_to_write + 2, true);
//		i2c_master_stop(cmd);
//		esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(50));
//		i2c_cmd_link_delete(cmd);
//		if (ret != ESP_OK)
//		{
//			printf("Failed to write string to EEPROM, err = %d\n", ret);
//			break;
//		}
//		address += bytes_to_write;
//		i += bytes_to_write;
//		vTaskDelay(10 / portTICK_PERIOD_MS);
//	}
// }
