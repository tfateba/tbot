
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
 */

#ifndef IP_I2C_H
#define IP_I2C_H

/*==========================================================================*/
/* Include Libraries.                                                       */
/*==========================================================================*/

/* ChibiOS files. */
#include "hal.h"

/*==========================================================================*/
/* Application macros.                                                      */
/*==========================================================================*/

#define I2C_CLOCK_SPEED 400000

/*==========================================================================*/
/* Driver Configuration structure.                                          */
/*==========================================================================*/

/**
 * @brief I2C Configuration.
 */
static const I2CConfig i2cConfig = {
  I2C_CLOCK_SPEED,  /* I2C Clock speed. */
};

/*==========================================================================*/
/* Functions prototypes.                                                    */
/*==========================================================================*/

msg_t i2cReadRegisters(I2CDriver *i2cp, uint8_t addr,
    uint8_t *reg, uint8_t *rxbuf, uint8_t lenght);
msg_t i2cWriteRegisters(I2CDriver *i2cp, uint8_t addr,
    uint8_t *txbuf, uint8_t lenght);

#endif /* IP_I2C_H */

