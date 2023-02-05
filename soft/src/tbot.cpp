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
 * @file    tbot.cpp
 * @brief   tbot source file.
 *
 * @addtogroup TBOT
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

/* Standard files. */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

/* ChibiOS files. */
#include "hal.h"
#include "chprintf.h"

/* Project files. */
#include "conf.h"
#include "kalman.hpp"
#include "main.h"
#include "motor.hpp"
#include "mpu6050.hpp"
#include "pid.hpp"
#include "pwm.hpp"
#include "asserv.hpp"
#include "tbot.hpp"

/*==========================================================================*/
/* Application macros.                                                      */
/*==========================================================================*/

/*==========================================================================*/
/* Global variables.                                                        */
/*==========================================================================*/


/* Extern variables. */
#if (DEBUG_TBOT)
extern BaseSequentialStream* chp; /* Pointer used for chpirntf. */
//#define pr_debug(x) chprintf(chp, x)
#endif


/*==========================================================================*/
/* Driver functions.                                                        */
/*==========================================================================*/

/**
 * @brief  Asservissement routine of the robot.
 *
 * @param[in] robot   the pointer to the robot driver
 */
void Tbot::init(void) {

  pr_debug("\n\rTbot initialization started.");
  // pidPosition.init();
  // pidAngle.init();
  // pidMotorL.init();
  // pidMotorR.init();
  kalmanFilter.init();

  //motorL.init();
  //motorR.init();

  //encoderL.init();
  //encoderR.init();
  //led1.init();
  buzzer.init();
  pr_debug("\n\rTbot initialization Done.");
}

#ifdef __cplusplus
}
#endif

/** @} */
