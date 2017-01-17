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
#include "ip_motor.h"

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

msg_t i2cReadRegisters(I2CDriver *i2cp, uint8_t addr,
    uint8_t *reg, uint8_t *rxbuf, uint8_t lenght);

msg_t i2cWriteRegisters(I2CDriver *i2cp, uint8_t addr,
    uint8_t *txbuf, uint8_t lenght);

#endif /* _IP_I2C_H_ */
