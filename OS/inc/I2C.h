/*!
 * @file  I2C.h
 * @brief TM4C123G I2C APIs and some settings.
 * ----------
 * For future development and updates, please follow this repository:
 * ----------
 * If you find any bug or problem, please create new issue or a pull request with a fix in the repository.
 * Or you can simply email me about the problem or bug at zeelivermorium@gmail.com
 * Much Appreciated!
 * ----------
 * @author Zee Livermorium
 * @date   Aug 4, 2018
 */

#include <stdint.h>
#include "VL53L0X.h"

/*
 *  I2C0 Conncection | I2C1 Conncection | I2C2 Conncection | I2C3 Conncection
 *  ---------------- | ---------------- | ---------------- | ----------------
 *  SCL -------- PB2 | SCL -------- PA6 | SCL -------- PE4 | SCL -------- PD0
 *  SDA -------- PB3 | SDA -------- PA7 | SDA -------- PE5 | SDA -------- PD1
 */

#define LIDAR0_I2C         I2C0
#define LIDAR1_I2C         I2C2

#define I2C0    0xabcc
#define I2C1    0xabcd
#define I2C2    0xabce
#define I2C3    0xabcc

/****************************************************
 *                                                  *
 *                   Initializer                    *
 *                                                  *
 ****************************************************/

/**
 * I2C_Init
 * ----------
 * @brief initialize a I2C module with corresponding setting parameters.
 */
void I2C_Init(void);


/****************************************************
 *                                                  *
 *                     R/W API                      *
 *                                                  *
 ****************************************************/

/**
 * I2C_read
 * ----------
 * @param  deviceAddress  address of slave device.
 * @param  targetRegister target register of slave device.
 * @param  data           data address to store read data.
 * @param  count          number of bytes to be read.
 * ----------
 * @brief read 1 or more bytes from slave device.
 */
int I2C_read(uint8_t deviceAddress, uint8_t targetRegister, uint8_t* data, uint32_t count, int deviceIndex);

/**
 * I2C_write
 * ----------
 * @param  deviceAddress  address of slave device.
 * @param  targetRegister target register of slave device.
 * @param  data          data address of data to be writen.
 * @param  count          number of bytes to be writen.
 * ----------
 * @brief write 1 or more bytes to slave device.
 */
int I2C_write(uint8_t deviceAddress, uint8_t targetRegister, uint8_t* data, uint32_t count, int deviceIndex);

/**
 * I2C_read_byte
 * ----------
 * @param  deviceAddress  address of slave device.
 * @param  targetRegister target register of slave device.
 * @param  data           data address to store read data.
 * ----------
 * @brief read 1 byte from slave device.
 */
int I2C_read_byte(uint8_t deviceAddress, uint8_t targetRegister, uint8_t *data, int deviceIndex);


/**
 * I2C_write_byte
 * ----------
 * @param  deviceAddress  address of slave device.
 * @param  targetRegister target register of slave device.
 * @param  data           data to be writen.
 * ----------
 * @brief write 1 byte to slave device.
 */
int I2C_write_byte(uint8_t deviceAddress, uint8_t targetRegister, uint8_t data, int deviceIndex);

/**
 * I2C_read_2_bytes
 * ----------
 * @param  deviceAddress  address of slave device.
 * @param  targetRegister target register of slave device.
 * @param  data           data address to store read data.
 * ----------
 * @brief read 2 bytes from slave device.
 */
int I2C_read_2_bytes(uint8_t deviceAddress, uint8_t targetRegister, uint8_t* data, int deviceIndex);

/**
 * I2C_write_2_bytes
 * ----------
 * @param  deviceAddress  address of slave device.
 * @param  targetRegister target register of slave device.
 * @param  data           data to be writen.
 * ----------
 * @brief write 2 bytes to slave device.
 */
int I2C_write_2_bytes(uint8_t deviceAddress, uint8_t targetRegister, uint8_t* data, int deviceIndex);

/**
 * I2C_read_4_bytes
 * ----------
 * @param  deviceAddress  address of slave device.
 * @param  targetRegister target register of slave device.
 * @param  data           data address to store read data.
 * ----------
 * @brief read 4 bytes from slave device.
 */
int I2C_read_4_bytes(uint8_t deviceAddress, uint8_t targetRegister, uint8_t* data, int deviceIndex);

/**
 * I2C_write_4_bytes
 * ----------
 * @param  deviceAddress  address of slave device.
 * @param  targetRegister target register of slave device.
 * @param  data           data to be writen.
 * ----------
 * @brief write 4 bytes to slave device.
 */
int I2C_write_4_bytes(uint8_t deviceAddress, uint8_t targetRegister, uint8_t* data, int deviceIndex);
