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

/* Standard files. */
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

/* ChibiOS files. */
#include "hal.h"
#include "chprintf.h"

/* Project files. */
#include "conf.h"
#include "motor.h"
#include "pwm.h"

/*==========================================================================*/
/* Global variables.                                                        */
/*==========================================================================*/

const uint16_t  maxSpeedValue = 512;  /**< Robot maximum speed value.       */

#if (DEBUG_MOTOR)
extern BaseSequentialStream* chp;
#endif

/*==========================================================================*/
/* Driver functions.                                                        */
/*==========================================================================*/

/**
 * @brief   Stop the corresponding motor.
 *
 * @param[in] mId   motor id of the motor to stop, rigth/left
 */
void motorStop(MOTORDriver *mdp) {

  mdp->pwmValue = 0;
  pwmSetDutyCycle(mdp);
  palClearPad(mdp->config.enablePort, mdp->config.enablePin);
}

/**
 * @brief   Driving the motor to the given speed.
 *
 * @param[in] mdp    pointer to the motor to move.
 */
void motorMove(MOTORDriver *mdp) {

  float speedabs = 0;

  /* Get the motor direction:. */
  if (mdp->speed >= 0) {
    mdp->dir = MOTOR_DIR_F;
    speedabs = mdp->speed;
  }
  else {
    mdp->dir = MOTOR_DIR_B;
    speedabs = abs(mdp->speed);
  }

  if (speedabs > maxSpeedValue)
    speedabs = maxSpeedValue;

  mdp->pwmValue = speedabs*((float)PWMVALUE)/maxSpeedValue;
  pwmSetDutyCycle(mdp);
}

/**
 * @brief   Enable left or right motor with GPIO signal.
 *
 * @param[in] mid  id of the motor to be enable, left/right
 */
static void motorEnable(MOTORDriver *mdp) {

  palSetPad(mdp->config.enablePort, mdp->config.enablePin);
}

/**
 * @brief   Initialize all pins needs for motor control
 */
void motorInit(MOTORDriver *mdp, MOTORConfig cfg) {

  mdp->config = cfg;
  palSetPadMode(mdp->config.forwardPort,  mdp->config.forwardPin, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(mdp->config.reversePort,  mdp->config.reversePin, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(mdp->config.enablePort,   mdp->config.enablePin,  PAL_MODE_OUTPUT_PUSHPULL);
  motorEnable(mdp);

  /* Init PWM modules. */
  pwmInits(&mdp->config);
}

/** @}  */
