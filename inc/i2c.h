
/**
 *
 * @file    ip_i2c.h
 *
 * @brief   I2C driver header file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    05 July 2016
 *
 */

/*
    IP - Copyright (C) 2015..2018 Theodore Ateba

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef IP_I2C_H
#define IP_I2C_H

/*==========================================================================*/
/* Include Libraries.                                                       */
/*==========================================================================*/

/* ChibiOS files. */
#include "hal.h"

/*==========================================================================*/
/* Driver macros.                                                           */
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

msg_t i2c_read_registers( I2CDriver *i2cp, uint8_t addr, uint8_t *reg,
                        uint8_t *rxbuf, uint8_t lenght);
msg_t i2c_write_registers(I2CDriver *i2cp, uint8_t addr, uint8_t *txbuf,
                        uint8_t lenght);

#endif /* IP_I2C_H */

