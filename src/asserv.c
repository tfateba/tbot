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
 * @file    asserv.c
 * @brief   Robot asservissement source file.
 *
 * @addtogroup ASSERV
 * @{
 */

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
#include "kalman.h"
#include "main.h"
#include "motor.h"
#include "mpu6050.h"
#include "pid.h"
#include "pwm.h"

/*==========================================================================*/
/* Application macros.                                                      */
/*==========================================================================*/

#define PI          3.14159265359
#define RAD_TO_DEG  180/PI

/*==========================================================================*/
/* Global variables, I2C TX and RX buffers, I2C and Serial Configurations   */
/*==========================================================================*/

/* Local variables. */

/* TODO: See all the variable that can be put in t-bot structure.           */
/* TODO: put this variable in t-bot structure.                              */
bool  layingDown    = true;       /**< Robot position, down or not. */

/* Put  */
const float   dt = 0.01;          /**< Asservissement period. */

/* Extern variables. */
#if (DEBUG == TRUE || DEBUG_ASS == TRUE)
extern BaseSequentialStream* chp; /* Pointer used for chpirntf. */
#endif

/* Inputs for Speed PID. */
float consigneSpeed = 0;          /**< The robot target speed. */
float speedMean     = 0;          /**< Left and right motor speed mean. */

/* Inputs for Angle PID. */
const float consigneAngle = 180;  /**< The robot target angle. */
float measuredAngle       = 0;    /**< Result of IMU and Kalman filter. */

float turnSpeed = 0;              /**< Can be changed by the User. */

float leftMeasuredSpeed  = 0;     /**< Left motor measured speed.  */
float rightMeasuredSpeed = 0;     /**< Right motor measured speed. */

/*==========================================================================*/
/* Driver functions.                                                        */
/*==========================================================================*/

/**
 * @brief  Asservissement routine of the robot.
 *
 * @param[in] rdp   the pointer to the robot driver
 */
void asserv(ROBOTDriver *rdp) {

  msg_t msg;

  /* Read the IMU data (x,y,z accel and gyroscope). */
  msg = mpu6050GetData(&I2CD1, &rdp->imu);

  if (msg != MSG_OK) {
#if (DEBUG == TRUE || DEBUG_ASS == TRUE)
    chprintf(chp, "\n\r %s: Error while reading the MPU6050 sensor data.",
              __func__);
#endif
    return;
  }

  /* Calcul of the Pitch angle of the robot. */
  rdp->imu.pitch = (atan2(rdp->imu.y_accel, rdp->imu.z_accel) + PI)*(RAD_TO_DEG);

  /* Get the Kalman estimation of the robot angle. */
  rdp->imu.pitch_k = kalmanGetAngle(rdp->imu.pitch, (rdp->imu.x_gyro / 131.0), dt);
  measuredAngle = rdp->imu.pitch_k;

  if ((layingDown && (rdp->imu.pitch_k < 170 || rdp->imu.pitch_k > 190)) ||
    (!layingDown && (rdp->imu.pitch_k < 135 || rdp->imu.pitch_k > 225))) {
    /*
     * The robot is in a unsolvable position, so turn off both motors and
     * wait until it's vertical again.
     */

#if (DEBUG == TRUE || DEBUG_ASS == TRUE)
    chprintf(chp, "%s: The Robot is laying down.\n\r", __func__);
#endif

    layingDown = true;
    motorStop(&rdp->motorLeft);
    motorStop(&rdp->motorRight);
    pidResetParameters(&rdp->pidSpeed);
    pidResetParameters(&rdp->pidAngle);
    pidResetParameters(&rdp->pidMotorLeft);
    pidResetParameters(&rdp->pidMotorRight);
  }
  else {

    /*
     * It's no longer laying down,
     * so we can try to stabilized the robot now.
     */
    layingDown = false;

    /* Four PID are used to stabilize the robot and control it:
     *
     * - speed pid :       this is use to move robot backward and forward
     * - angle pid :       this is use to stabilize the robot
     * - left motor pid :  this is use to turn robot
     * - right motor pid : this is use to turn robot
     */

    /* TODO: motor speed must be measure. */

    /* Update the speed PID. */
    rdp->pidSpeed.consigne = consigneSpeed;  /* Distance setpoint.  */
    rdp->pidSpeed.measure  = speedMean;      /* Distance feedback.  */
    pidCompute(&rdp->pidSpeed);              /* Set the PID output. */

    /* Update the angle PID. */
    rdp->pidAngle.consigne = consigneAngle + rdp->pidSpeed.output;
    rdp->pidAngle.measure  = measuredAngle;
    pidCompute(&rdp->pidAngle);

    /* Manage PID for the left Motor. */
    rdp->pidMotorLeft.consigne = (rdp->pidAngle.output + turnSpeed);
    rdp->pidMotorLeft.measure  = leftMeasuredSpeed;
    pidCompute(&rdp->pidMotorLeft);

    /* Manage PID for the Right Motor. */
    rdp->pidMotorRight.consigne = (rdp->pidAngle.output - turnSpeed);
    rdp->pidMotorRight.measure  = rightMeasuredSpeed;
    pidCompute(&rdp->pidMotorRight);

    /* Get the motors power that need to be apply. */
    rdp->motorLeft.speed  = rdp->pidMotorLeft.output;
    rdp->motorRight.speed = rdp->pidMotorRight.output;

    /* Set power to the left motor. */
    motorMove(&rdp->motorLeft);

    /* Set power to the rigth motor. */
    motorMove(&rdp->motorRight);

#if (DEBUG == TRUE || DEBUG_ASS == TRUE)
    chprintf(chp, "%s: filtered pitch = %.3f\r\n", __func__, rdp->imu.pitch_k);
#endif
  }
}

/** @} */
