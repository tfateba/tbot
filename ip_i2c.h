/**
 *
 * @file    ip_i2c.h
 *
 * @brief   I2C driver header file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    05 Jully 2016
 *
 * @update  17 November 2016
 *
 */

#ifndef _IP_I2C_H_
#define _IP_I2C_H_

/*===========================================================================*/
/* Include Libraries.                                                        */
/*===========================================================================*/
#include <hal.h>

/*===========================================================================*/
/* Driver Configuration.                                                     */
/*===========================================================================*/
/**
 * @brief I2C Configuration.
 */
static const I2CConfig i2cConfig = {
  400000,             /* I2C Clock speed                                    */
};

/*===========================================================================*/
/* Driver Functions                                                          */
/*===========================================================================*/

/**
 * @fn      i2cReadRegisters
 * @brief   Read a register or a block of registers from the sensor.
 *
 * @param[in] i2cp    pointer to the i2c interface
 * @param[in] addr    slave address without R/W bit
 * @param[in] reg     pointer to the first register address to read
 * @param[in] rxbuf   pointer to the buffer to store the data readed
 * @param[in] lenght  size of data to read
 *
 * @return    msg     the result of the reading operation
 */
msg_t i2cReadRegisters(I2CDriver *i2cp, uint8_t addr,
    uint8_t *reg, uint8_t *rxbuf, uint8_t lenght);

/**
 * @fn      i2cWriteRegisters
 * @brief   Write to a register or a block of registers on the sensor.
 *
 * @param[in] i2cp    pointer to the i2c interface
 * @param[in] addr    slave address without R/W bit
 * @param[in] txbuf   pointer to the data to write into the sensor
 *                    txbuf[0] is the first register to write
 * @param[in] lenght  size of data to write to the sensor
 *
 * @return    msg     the result of the reading operation
 */
msg_t i2cWriteRegisters(I2CDriver *i2cp, uint8_t addr,
    uint8_t *txbuf, uint8_t lenght);

#endif /* _IP_I2C_H_ */
