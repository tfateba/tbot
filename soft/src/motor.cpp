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
 * @file          motor.c
 * @brief         motor driver source file.
 *
 * @description   Motor control and Encoder Read
 *                Description:
 *                Motor Power:  white
 *                Motor Power:  yellow
 *                Encoder GND:  blue
 *                Encoder VCC:  green
 *                Encoder A:    black
 *                Encoder B:    Red
 *
 * @addtogroup MOTOR
 * @{
 */

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/
#ifdef __cplusplus
extern "C" {
#endif

/* Standard files. */
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

/* ChibiOS files. */
#include "hal.h"
#include "chprintf.h"

/* Project files. */
#include "conf.h"

#include "motor.hpp"
#include "pwm.hpp"

/*==========================================================================*/
/* Global variables.                                                        */
/*==========================================================================*/

/*==========================================================================*/
/* Driver functions.                                                        */
/*==========================================================================*/

/**
 * @brief   Stop the corresponding motor.
 *
 * @param[in] mp   pointer of the motor that need to be stopped.
 */
void Motor::stop(void) {

  setVoltage(0);

  pwmModule.setDutyCycle( getId(),
                          getDirection(),
                          getEnablePort(),
                          getEnablePin(),
                          getPwmChannel1(),
                          getPwmChannel2(),
                          getVoltage());

  palClearPad(getEnablePort(), getEnablePin());
}

/**
 * @brief   Driving the motor to the given speed.
 *
 * @param[in] mp    pointer to the motor to move.
 */
void Motor::move(void) {

  float speedabs = 0;

  /* Get the motor direction:. */
  if (getSpeed() >= 0) {
    setDirection(MOTOR_DIR_F);
    speedabs = getSpeed();
  }
  else {
    setDirection(MOTOR_DIR_B);
    speedabs = abs(getSpeed());
  }

  if (speedabs > getMaxSpeed())
    speedabs = getMaxSpeed();

  setVoltage(speedabs*((float)PWMVALUE)/getMaxSpeed());
  pwmModule.setDutyCycle( getId(),
                          getDirection(),
                          getEnablePort(),
                          getEnablePin(),
                          getPwmChannel1(),
                          getPwmChannel2(),
                          getVoltage());

}

/**
 * @brief   Enable left or right motor with GPIO signal.
 *
 * @param[in] mp  pointer to the motor that need to be enable.
 */
void Motor::enable(void) {

  palSetPad(getEnablePort(), getEnablePin());
}

/**
 * @brief   Initialize all pins needs for motor control
 *
 * @param[in] mp  pointer to the motor that need to be initialize.
 */
void Motor::init(MOTORConfig *cfg) {

  setConfig(cfg);
  palSetPadMode(getForwardPort(),  getForwardPin(), PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(getReversePort(),  getReversePin(), PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(getEnablePort(),   getEnablePin(),  PAL_MODE_OUTPUT_PUSHPULL);
  enable();

  /* Init PWM modules. */
  pwmModule.init(getPwmDriver(), getPwmConfig(), getForwardPort(), getReversePort(), getForwardPin(), getReversePin());
}

#ifdef __cplusplus
}
#endif

/** @}  */
