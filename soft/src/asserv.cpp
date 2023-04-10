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
 * @file    asserv.c
 * @brief   Robot asservissement source file.
 *
 * @addtogroup ASSERV
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

/*==========================================================================*/
/* Application macros.                                                      */
/*==========================================================================*/

/*==========================================================================*/
/* Global variables, I2C TX and RX buffers, I2C and Serial Configurations   */
/*==========================================================================*/

// TODO: all thoses variables must be in the asserv class.
/* Local variables. */

/* TODO: See all the variable that can be put in t-bot structure.           */
/* TODO: put this variable in t-bot structure.                              */
bool  layingDown    = true;       /**< Robot position, down or not. */

/* Put  */
const float   dt = 0.01;          /**< Asservissement period. */

/* Extern variables. */
#if (DEBUG_ASSERV)
extern BaseSequentialStream* chp; /* Pointer used for chpirntf. */
#define pr_debug(x) chprintf(chp, x)
#endif

/* Inputs for Speed PID. */
float targetPosition    = 0;      /**< The robot target position. */

/* Inputs for Angle PID. */
const float targetAngle = 180;  /**< The robot target angle. */
float measuredAngle       = 0;    /**< Result of IMU and Kalman filter. */

bool printEnable = true;

float positionLMeasured;
float positionRMeasured;

const float   R = 0.04;   // Radius of the wheel
const float   PPR = 480;  // This was measured whit the wheels on the robot.

#define PI          3.14159265359
#define RAD_TO_DEG  180/PI

/*==========================================================================*/
/* Driver functions.                                                        */
/*==========================================================================*/

/**
 * @brief  Asservissement routine of the robot.
 *
 * @param[in] robot   the pointer to the robot driver
 */
void Asserv::run(Tbot *robot) {

  msg_t msg;

  /* Read the IMU data (x,y,z accel and gyroscope). */
  msg = robot->imu.getData(&I2CD1);

  if (msg != MSG_OK) {
#if (DEBUG_ASSERV)
    if (printEnable == true) {
      chprintf(chp, "\n\rasserv.%s: Error while reading the MPU6050 sensor data.",
              __func__);
      printEnable = false;
    }
#endif
    return;
  }

  /* Calcul of the Pitch angle of the robot. */
  robot->imu.setpitchAngle((atan2(robot->imu.getYAccel(), robot->imu.getZAccel()) + PI)*(RAD_TO_DEG));

  /* Get the Kalman estimation of the robot angle. */
  robot->imu.setFiltredpitchAngle(robot->kalmanFilter.getAngle(robot->imu.getPitchAngle(), (robot->imu.getXGyro() / 131.0), dt));

  measuredAngle = robot->imu.getFiltredPitchAngle();

  if ((layingDown && (robot->imu.getFiltredPitchAngle() < 170 || robot->imu.getFiltredPitchAngle() > 190)) ||
    (!layingDown && (robot->imu.getFiltredPitchAngle() < 135 || robot->imu.getFiltredPitchAngle() > 225))) {
    /*
     * The robot is in a unsolvable position, so turn off both motors and
     * wait until it's vertical again.
     */

#if (DEBUG_ASSERV)
    if (printEnable == true) {
      chprintf(chp, "\n\rasserv.%s: The Robot is laying down.", __func__);
      printEnable = false;
    }
#endif

    layingDown = true;

    robot->motorL.stop();
    robot->motorR.stop();

    robot->pidPosition.reset();
    robot->pidAngle.reset();
    robot->pidMotorL.reset();
    robot->pidMotorR.reset();
  }
  else {
    /*
     * It's no longer laying down,
     * so we can try to stabilized the robot now.
     */
    layingDown = false;
    printEnable = true;

    /* Four PID are used to stabilize the robot and control it:
     *
     * - speed pid :       this is use to move robot backward and forward
     * - angle pid :       this is use to stabilize the robot
     * - left motor pid :  this is use to turn robot
     * - right motor pid : this is use to turn robot
     */

    /* TODO: motor speed must be measure. */
    //positionLMeasured = ((2*PI*R)/PPR)*((float)robot->encoderL.getCounter());
    //positionRMeasured = ((2*PI*R)/PPR)*((float)robot->encoderR.getCounter());

    // @TODO : use the Encoder to measure this !!!
    positionLMeasured = 0;
    positionRMeasured = 0;

    /* Update the Position PID. */
    robot->pidPosition.setSetPoint(targetPosition);
    robot->pidPosition.setMeasure((positionRMeasured + positionLMeasured)/2);
    robot->pidPosition.compute();

    /* Update the Angle PID. */
    robot->pidAngle.setSetPoint(targetAngle - robot->pidPosition.getOutput());
    robot->pidAngle.setMeasure(measuredAngle);
    robot->pidAngle.compute();
    chprintf(chp, "%s: targetAngle = %.3f, measuredAngle = %.3f\r\n", __func__, targetAngle, measuredAngle);

    /* Manage PID for the left Motor. */
    robot->pidMotorL.setSetPoint(robot->pidAngle.getOutput());
    robot->pidMotorL.setMeasure(positionLMeasured);
    robot->pidMotorL.compute();

    /* Manage PID for the Right Motor. */
    robot->pidMotorR.setSetPoint(robot->pidAngle.getOutput());
    robot->pidMotorR.setMeasure(positionRMeasured);
    robot->pidMotorR.compute();

    /* Get the motors power that need to be apply. */
    robot->motorL.setSpeed(robot->pidMotorL.getOutput());
    robot->motorR.setSpeed(robot->pidMotorR.getOutput());

    /* Set power to the left motor. */
    robot->motorL.move();

    /* Set power to the rigth motor. */
    robot->motorR.move();

#if (DEBUG_ASSERV)
    //encoderGetDistance(&robot->encoderLeft);
    //encoderGetDistance(&robot->encoderRight);
    //chprintf(chp, "%s: angle = %.3f, distanceL = %.3f, distanceR = %.3f\r\n", __func__, robot->pidAngle.measure, positionL, positionR);

    //chprintf(chp, "\n\rasserv.%s: Tposi = %.3f, Mposi = %.3f, Tangle %.3f, Mangle %.3f", __func__, robot->pidPosition.getSetPoint(), robot->pidPosition.getMeasure(), robot->pidAngle.getSetPoint(), robot->pidAngle.getMeasure());
    //chprintf(chp, "\n\rasserv.%s: pidMotorL.output = %.3f, pidMotorR.output = %.3f", __func__, robot->pidMotorL.getOutput(), robot->pidMotorR.getOutput());
#endif
  }
}

#ifdef __cplusplus
}
#endif

/** @} */
