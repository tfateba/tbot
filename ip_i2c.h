/**
 *
 * @file    sam_i2c.h
 *
 * @brief   I2C driver header file.
 *
 * @author  Theodore Ateba
 *
 * @date    05 Jully 2016
 *
 * @update  29 Jully 2016
 *
 * @TODO: Put the correct details on the function prototype.
 *
 */

#ifndef _SAM_I2C_H_
#define _SAM_I2C_H_

/*===========================================================================*/
/* Include Libraries                                                         */
/*===========================================================================*/
#include <hal.h>

/*===========================================================================*/
/* Driver Configuration.                                                     */ 
/*===========================================================================*/
/**
 * brief I2C Configuration.
 */
static const I2CConfig i2cConfig = {
  400000,             /* I2C Clock speed                                    */
};

/*===========================================================================*/
/* Driver Functions                                                          */ 
/*===========================================================================*/

/**
 * @fn    i2cReadRegisters
 * @brief Read a register of the LM75A sensor
 *
 * @param[in] i2cp    Pointer to the i2c interface.
 * @param[in] sad     Slave address without R/W bit.
 * @param[in] reg     First register address to read.
 * @param[in] rxbuf   Buffer to store the data receive by i2c bus.
 * @param[in] lenght  Size of data to read
 * @retuen            The result of the reading operation.
 */
msg_t i2cReadRegisters(I2CDriver *i2cp, uint8_t addr,
    uint8_t *reg, uint8_t *rxbuf, uint8_t lenght);

/**
 * @fn    i2cWriteRegisters
 * @brief Write a register of the LM75A sensor
 *
 * @param[in] i2cp    Pointer to the i2c interface.
 * @param[in] sad     Slave address without R/W bit.
 * @param[in] txbuf   Data to write to the sensor register.
 *                    txbuf[0] is the first register to write.
 * @param[in] lenght  Size of data to write
 * @retuen            The result of the reading operation.
 */
msg_t i2cWriteRegisters(I2CDriver *i2cp, uint8_t addr,
    uint8_t *txbuf, uint8_t lenght);

#endif /* _I2C_H_ */

