/*
    TBOT - Copyright (C) 2015...2021 Theodore Ateba

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

/**
 * @file    i2c.h
 * @brief   I2C header file.
 *
 * @addtogroup I2C
 * @{
 */

#ifndef I2C_H
#define I2C_H

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

/* ChibiOS files. */
#include "hal.h"

/*==========================================================================*/
/* Enumerations, Structures and macros.                                     */
/*==========================================================================*/

#define I2C_CLOCK_SPEED 400000

/**
 * @brief I2C Configuration.
 */
static const I2CConfig i2cConfig = {
  I2C_CLOCK_SPEED,  /* I2C Clock speed. */
};

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

msg_t i2cReadRegisters(I2CDriver *i2cp, uint8_t addr, uint8_t *reg,
                        uint8_t *rxbuf, uint8_t lenght);
msg_t i2cWriteRegisters(I2CDriver *i2cp, uint8_t addr, uint8_t *txbuf,
                        uint8_t lenght);

#ifdef __cplusplus
}
#endif

#endif /* I2C_H */

/** @} */
