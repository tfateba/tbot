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
 * @file    conf.h
 * @brief   Robot configuration header file.
 *
 * @addtogroup CONF
 * @{
 */

#ifndef CONF_H
#define CONF_H

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/
#include "chprintf.h"

/*==========================================================================*/
/* Enumerations, Structures and macros.                                     */
/*==========================================================================*/

/**
 * @brief   Color define for chprintf.
 */
#define chprintf_r(x, y)  chprintf(x, "\x1b[31m" y "\x1b[0m") /**< Red.     */
#define chprintf_g(x, y)  chprintf(x, "\x1b[32m" y "\x1b[0m") /**< Green.   */
#define chprintf_y(x, y)  chprintf(x, "\x1b[33m" y "\x1b[0m") /**< Yellow.  */
#define chprintf_b(x, y)  chprintf(x, "\x1b[34m" y "\x1b[0m") /**< Blue.    */
#define chprintf_m(x, y)  chprintf(x, "\x1b[35m" y "\x1b[0m") /**< Magenta. */
#define chprintf_c(x, y)  chprintf(x, "\x1b[36m" y "\x1b[0m") /**< Cyant.   */
#define chprintf_w(x, y)  chprintf(x, "\x1b[0m"  y "\x1b[0m") /**< White.   */

/**
 * @brief   Available debug level.
 */
#define DEBUG           FALSE   /**< Debug activation in all source files.  */
#define DEBUG_MAIN      FALSE   /**< Debug activation in main file.         */
#define DEBUG_ASSERV    TRUE    /**< Debug activation in asserv file        */
#define DEBUG_ENCODER   FALSE   /**< Debug activation in asserv file        */
#define DEBUG_PID       FALSE   /**< Debug activation in pid file.          */
#define DEBUG_MOTOR     FALSE   /**< Debug activation in Motor file.        */
#define DEBUG_KALMAN    FALSE   /**< Debug activation in Kalman filter file.*/
#define DEBUG_PWM       FALSE   /**< Debug activation in PWM file.          */
#define DEBUG_I2C       FALSE   /**< Debug activation in I2C file.          */
#define DABUG_MPU       FALSE   /**< Debug activation in MPU6050 file.      */

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

#endif /* CONF_H */

/** @} */
