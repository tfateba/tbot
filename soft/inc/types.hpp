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
 * @file    types.hpp
 * @brief   types header file.
 *
 * @addtogroup TYPES
 * @{
 */

#ifndef TYPES_H
#define TYPES_H

#include "hal.h"

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

/*==========================================================================*/
/* Enumerations, Structures and macros.                                     */
/*==========================================================================*/

/*==========================================================================*/
/* External declarations.                                                   */
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
  motor_id_t  id;          /**< Motor identification name.       */
  float       maxSpeed;     /**< Motor maximum speed.             */
  ioportid_t  forwardPort;  /**< Motor driver forward pwm Port.   */
  ioportid_t  reversePort;  /**< Motor driver backwad pwm Port.   */
  ioportid_t  enablePort;   /**< Motor driver enable Port.        */
  uint8_t     forwardPin;   /**< Motor driver forward pwm pin.    */
  uint8_t     reversePin;   /**< Motor driver backwad pwm pin.    */
  uint8_t     enablePin;    /**< Motor driver enable pin.         */
  PWMDriver   *pwmDriver;   /**< Motor driver pwm for control.    */
  PWMConfig   *pwmConfig;   /**< Motor driver pwm config.         */
  uint8_t     pwmChannel1;  /**< Motoe driver pwm channel 1 used; */
  uint8_t     pwmChannel2;  /**< Motoe driver pwm channel 2 used; */
} MOTORConfig;

/**
 * @brief   Motor driver structure.
 */
typedef struct {
  MOTORConfig config;   /**< Motor configuration.         */
  motor_dir_t dir;      /**< Motor rotation directory.    */
  float       speed;    /**< Motor speed.                 */
  int         pwmValue; /**< Motor pwm value for control. */
} MOTORDriver;

/**
 * @brief Encoders identifier enumerations
 */
typedef enum {
  ENCODER_L,  /**< Left encoder.                                            */
  ENCODER_R,  /**< Right encoder.                                           */
} encoder_id_t;

/**
 * @brief   Encoder configuration structure.
 */
typedef struct {
  uint8_t     id;       /**< Encoder identification name.                   */
  uint8_t     eichan;   /**< Encoder external interruption channel.         */
  ioportid_t  porta;    /**< Encoder port A.                                */
  ioportid_t  portb;    /**< Encoder port B.                                */
  uint8_t     pina;     /**< Encoder pin A.                                 */
  uint8_t     pinb;     /**< Encoder pin B.                                 */
} ENCODERConfig;

/**
 * @brief   Encoder driver structure.
 */
typedef struct {
  ENCODERConfig config;
  volatile long counter;  /**< Rigth encoder counter.                       */
  bool          statea;   /**< Left motor encoder A.                        */
  bool          stateb;   /**< Left motor encoder B.                        */
} ENCODERDriver;

typedef struct {
  ioportid_t  port;
  iopadid_t   pin; 
} LEDDriver;

#endif /* TYPES_H */

/** @} */
