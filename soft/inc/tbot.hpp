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
 * @file    tbot.hpp
 * @brief   tbot header file.
 *
 * @addtogroup TBOT
 * @{
 */

#ifndef TBOT_H
#define TBOT_H

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

/* Project files. */
#include "encoder.hpp"
#include "motor.hpp"
#include "mpu6050.hpp"
#include "pid.hpp"
#include "led.hpp"
#include "buzzer.hpp"
#include "kalman.hpp"

/*==========================================================================*/
/* Enumerations, Structures and macros.                                     */
/*==========================================================================*/

/**
 * @brief   Structure representing a Robot driver.
 */
//struct ROBOTDriver {
class Tbot {

public:
  void init(void);

  Mpu6050 imu;            /**< Robot IMU.                             */

  Pid     pidPosition;    /**< must be pidSpeed.                      */
  Pid     pidAngle;       /**< Must be pidAngle.                      */
  Pid     pidMotorL;      /**< TODO: must be pidLeftMotor.            */
  Pid     pidMotorR;      /**< Robot pid for rigth motor.             */

  Motor   motorL;         /**< Robot left  motor.                     */
  Motor   motorR;         /**< Robot rigth motor.                     */

  Encoder encoderL;       /**< Robot left  encoder.                   */
  Encoder encoderR;       /**< Robot rigth encoder.                   */
  Led     led1;           /**< Robot led*/
  Buzzer  buzzer;         /**< Rbot Buzzer.                           */

  Kalman  kalmanFilter;
};


/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

#endif /* TBOT_H */

/** @} */
