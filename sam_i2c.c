/**
 *
 * @file    i2c.c
 *
 * @brief   I2C driver
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    05 Jully 2016
 *
 * @update  24 August 2016
 *
 */

/*===========================================================================*/
/* Include Libraries                                                         */
/*===========================================================================*/
#include "sam_i2c.h"

/*===========================================================================*/
/* Driver Functions                                                          */ 
/*===========================================================================*/

/**
 * @fn    i2cReadRegisters
 * @brief Read a register or a block of registers from the sensor.
 *
 * @param[in] i2cp    Pointer to the i2c interface.
 * @param[in] addr    Slave address without R/W bit.
 * @param[in] reg     Pointer to the first register address to read.
 * @param[in] rxbuf   Pointer to the buffer to store the data readed.
 * @param[in] lenght  Size of data to read.
 * @retuen		msg			The result of the reading operation.
 */
msg_t i2cReadRegisters(I2CDriver *i2cp, uint8_t addr,
    uint8_t *reg, uint8_t *rxbuf, uint8_t lenght)
{
  msg_t msg;

  i2cAcquireBus(i2cp);
  msg = i2cMasterTransmitTimeout(i2cp, addr, reg, 1, rxbuf, lenght, MS2ST(1));
  i2cReleaseBus(i2cp);

  return msg;
}

/**
 * @fn    i2cWriteRegisters
 * @brief Write to a register or a block of registers on the sensor.
 *
 * @param[in] i2cp    Pointer to the i2c interface.
 * @param[in] addr    Slave address without R/W bit.
 * @param[in] txbuf   Pointer to the data to write into the sensor.
 *                    txbuf[0] is the first register to write.
 * @param[in] lenght  Size of data to write to the sensor.
 * @return		msg     The result of the reading operation.
 */
msg_t i2cWriteRegisters(I2CDriver *i2cp, uint8_t addr,
    uint8_t *txbuf, uint8_t lenght)
{
  msg_t msg;

  i2cAcquireBus(i2cp);
  msg = i2cMasterTransmitTimeout(i2cp, addr, txbuf, lenght, NULL, 0, MS2ST(1));
  i2cReleaseBus(i2cp);

  return msg;
}
