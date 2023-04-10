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
#include "pwm.hpp"
//#pragma once

/*==========================================================================*/
/* Enumerations, Structures and macros.                                     */
/*==========================================================================*/

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

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

class Motor {
  public:
  void        stop();
  void        move();
  void        init(MOTORConfig *cfg);

  void        setConfig(MOTORConfig *cfg) {config = cfg;}
  motor_id_t  getId(void) {return config->id;}
  void        setId(motor_id_t mid) {config->id = mid;}
  
  float       getMaxSpeed(void) {return config->maxSpeed;}
  void        setMaxSpeed(float ms) {config->maxSpeed = ms;}

  ioportid_t  getForwardPort(void) {return config->forwardPort;}
  ioportid_t  getReversePort(void) {return config->reversePort;}
  ioportid_t  getEnablePort(void) {return config->enablePort;}
  uint8_t     getForwardPin(void) {return config->forwardPin;}
  uint8_t     getReversePin(void) {return config->reversePin;}
  uint8_t     getEnablePin(void)  {return config->enablePin;}

  void        setPwmDriver(PWMDriver *pwmdp) {config->pwmDriver = pwmdp;}
  PWMDriver   *getPwmDriver(void) {return config->pwmDriver;}

  void        setPwmConfig(PWMConfig *pwmcfgp){config->pwmConfig;}
  PWMConfig   *getPwmConfig(void) {return config->pwmConfig;}

  uint8_t     getPwmChannel1(void) {return config->pwmChannel1;}
  uint8_t     getPwmChannel2(void) {return config->pwmChannel2;}

  void        setDirection(motor_dir_t dir) {direction = dir;}
  motor_dir_t getDirection(void) {return direction;}

  void        setSpeed(float speedVal) {speed = speedVal;}
  float       getSpeed(void) {return speed;}
  void        setVoltage(int voltage) {pwmValue = voltage;}
  int         getVoltage(void) {return pwmValue;}

  private:
  
  void enable();

  uint16_t      maxSpeedValue;  /**< Robot maximum speed value.   512!!! */
  motor_dir_t   direction;      /**< Motor rotation directory.    */
  float         speed;          /**< Motor speed.                 */
  int           pwmValue;       /**< Motor pwm value for control. */
  MOTORConfig   *config;        /**< Motor configuration.         */
  Pwm           pwmModule;
};

#endif /* MOTOR_H */

/** @} */
