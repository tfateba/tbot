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
 * @file    motor.h
 * @brief   motor driver header file.
 *
 * @note
 *          Motor Wires, connexion to the Motor Drivers
 *          Motor + is the Yellow wire.
 *          Motor - is the White wire.
 *
 * @addtogroup MOTOR
 * @{
 */

#ifndef MOTOR_H
#define MOTOR_H

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

/* ChibiOS files. */
#include "hal.h"
#include "conf.h"

/*==========================================================================*/
/* Enumerations, Structures and macros.                                     */
/*==========================================================================*/

/**
 * @brief Motors identifier enumeration.
 */
typedef enum {
  MOTOR_L,  /**< Left motor. */
  MOTOR_R,  /**< Right motor. */
} motor_id_t;

/**
 * @brief Motors enumerations
 */
typedef enum {
  MOTOR_DIR_F,  /**< Motor forward direction. */
  MOTOR_DIR_B,  /**< Motor backward direction. */
} motor_dir_t;

/**
 * @brief Motor configuration structure.
 */
typedef struct {
  motor_id_t  mid;          /**< Motor identification name. */
  float       maxSpeed;     /**< Motor maximum speed. */
  ioportid_t  forwardPort;  /**< Motor driver forward pwm Port. */
  ioportid_t  backwardPort; /**< Motor driver backwad pwm Port. */
  ioportid_t  enablePort;   /**< Motor driver enable Port. */
  uint8_t     forwardPin;   /**< Motor driver forward pwm pin. */
  uint8_t     backwardPin;  /**< Motor driver backwad pwm pin. */
  uint8_t     enablePin;    /**< Motor driver enable pin. */
} MOTORConfig;

/**
 * @brief   Motor driver structure.
 */
typedef struct {
  MOTORConfig config;  /**< Motor configuration. */
  motor_dir_t dir;     /**< Motor rotation directory. */
  float       speed;   /**< Motor speed. */
} MOTORDriver;

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

void motorStop(MOTORDriver *mdp);
void motorMove(MOTORDriver *mdp);
void motorInit(MOTORDriver *mdp, MOTORConfig cfg);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */

/** @} */
