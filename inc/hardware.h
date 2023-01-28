/*
    TBOT - Copyright (C) 2015...2023 Theodore Ateba

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
 * @file    hardware.h
 * @brief   Hardware header file.
 *
 * @addtogroup HARDWARE
 * @{
 */

#ifndef HARDWARE_H
#define HARDWARE_H

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

/* ChibiOS files. */
#include "hal.h"

/*==========================================================================*/
/* Enumerations, Structures and macros.                                     */
/*==========================================================================*/

/**
 * @brief   List of the board that can be use to run the software.
 */
#define ARDUINO_MEGA    0
#define ARDUINO_UNO     1
#define NUCLEO_401RE    2

/**
 * @brief   Select board!!
 */
#define BOARD_USED      ARDUINO_MEGA

#if (BOARD_USED == ARDUINO_MEGA)

#define D2  PE4
#define D3  PE5
#define D4  PG5
#define D5  PE3
#define D6  PH3
#define D7  PH4
#define D8  PH5
#define D11 PB5
#define D12 PB6
#define D18 PD3
#define D19 PD2
#define D46 PL3

#elif (BOARD_USED == ARDUINO_UNO)
#define D12   PB4
#endif

#define MOTOR_MAX_SPEED         255

/* Arduino Interruption */
#define INT2 2      /**< ISR, D19 [PD2], BLACK wire motor Right.            */
#define INT3 3      /**< ISR, D18 [PD3], RED wire Motor Left.               */

#define L_ENCODER_EXT_INT       INT3    /**< Left encoder external isr pin. */
#define R_ENCODER_EXT_INT       INT2    /**< Right encoder external isr pin.*/

#define L_ENCODER_PORT_A        IOPORT4 /**< Left encoder A port.           */
#define L_ENCODER_PORT_B        IOPORT7 /**< Left encoder B port.           */

#define R_ENCODER_PORT_A        IOPORT4 /**< Right encoder A port.          */
#define R_ENCODER_PORT_B        IOPORT5 /**< Right encoder B port.          */

#define L_MOTOR_PORT_FORWARD    IOPORT5 /**< Left motor forward port.       */
#define L_MOTOR_PORT_BACKWARD   IOPORT5 /**< Left motor backward port.      */
#define L_MOTOR_PORT_ENABLE     IOPORT2 /**< Left motor enable port.        */

#define R_MOTOR_PORT_FORWARD    IOPORT8 /**< Right motor forward port.      */
#define R_MOTOR_PORT_BACKWARD   IOPORT8 /**< Right motor backward port.     */
#define R_MOTOR_PORT_ENABLE     IOPORT8 /**< Right motor enable port.       */


#define L_ENCODER_PIN_A         D18     /**< Left encoder A pin.            */
#define L_ENCODER_PIN_B         D4      /**< Left encoder B pin.            */

#define R_ENCODER_PIN_A         D19     /**< Right encoder A pin.           */
#define R_ENCODER_PIN_B         D5      /**< Right encoder B pin.           */

#define L_MOTOR_PIN_FORWARD     D3      /**< Left motor forward pin.        */
#define L_MOTOR_PIN_BACKWARD    D2      /**< Left motor backwad pin.        */
#define L_MOTOR_PIN_ENABLE      D11     /**< Left motor enable pin.         */

#define R_MOTOR_PIN_FORWARD     D6      /**< Right motor forward pin.       */
#define R_MOTOR_PIN_BACKWARD    D8      /**< Right motor backward pin.      */
#define R_MOTOR_PIN_ENABLE      D7      /**< Right motor enable pin.        */

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* HARDWARE_H */

/** @} */
