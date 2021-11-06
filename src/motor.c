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

#if (DEBUG == TRUE || DEBUG_MOT == TRUE)
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

  pwmSetDutyCycle(mdp->config.mid, mdp->dir, 0);
  palClearPad(mdp->config.enablePort, mdp->config.enablePin);
}

/**
 * @brief   Driving the motor to the given speed.
 *
 * @param[in] mdp    pointer to the motor (rigth/left)
 */
void motorMove(MOTORDriver *mdp) {

  int duty;
  float speedabs;

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

  duty = speedabs*((float)PWMVALUE)/maxSpeedValue;
  pwmSetDutyCycle(mdp->config.mid, mdp->dir, duty);
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
 * @brief   Disable left or right motor with GPIO signal.
 *
 * @param[in] mid  Id of the motor to be disable, left/right
 *//*
static void motorDisable(motor_id_t mid) {

  if (mid == MOTOR_L)
    palClearPad(LMD_EN_PORT, LMD_EN);

  if (mid == MOTOR_R)
    palClearPad(RMD_EN_PORT, RMD_EN);
}*/

/**
 * @brief   Initialize all pins needs for motor control
 */
void motorInit(MOTORDriver *mdp, MOTORConfig cfg) {

  mdp->config = cfg;
  palSetPadMode(mdp->config.forwardPort,  mdp->config.forwardPin,  PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(mdp->config.backwardPort, mdp->config.backwardPin, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(mdp->config.enablePort,   mdp->config.enablePin,   PAL_MODE_OUTPUT_PUSHPULL);
  motorEnable(mdp);
}

/** @}  */
