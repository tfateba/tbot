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
 * @file    main.h
 * @brief   main application header file.
 *
 * @addtogroup MAIN
 * @{
 */

#ifndef MAIN_H
#define MAIN_H

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

/* Project files. */
#include "encoder.h"
#include "motor.h"
#include "mpu6050.h"
#include "pid.h"

/*==========================================================================*/
/* Enumerations, Structures and macros.                                     */
/*==========================================================================*/

/**
 * @brief   Structure representing a Robot driver.
 */
struct ROBOTDriver {

  MPU6050Driver imu;            /**< Robot IMU.                             */

  PIDDriver     pidPosition;    /**< must be pidSpeed.                      */
  PIDDriver     pidAngle;       /**< Must be pidAngle.                      */
  PIDDriver     pidMotorL;      /**< TODO: must be pidLeftMotor.            */
  PIDDriver     pidMotorR;      /**< TODO: must be pidRightMotor.           */

  MOTORDriver   motorL;         /**< Robot left  motor.                     */
  MOTORDriver   motorR;         /**< Robot rigth motor.                     */

  ENCODERDriver encoderL;       /**< Robot left  encoder.                   */
  ENCODERDriver encoderR;       /**< Robot rigth encoder.                   */
};

/**
 * @brief   Type representing a Robot driver.
 */
typedef struct ROBOTDriver ROBOTDriver;

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

#endif /* MAIN_H */

/** @} */
