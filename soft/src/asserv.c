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
#if (DEBUG_ASSERV)
extern BaseSequentialStream* chp; /* Pointer used for chpirntf. */
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
 * @param[in] rdp   the pointer to the robot driver
 */
void asserv(ROBOTDriver *rdp) {

  msg_t msg;

  /* Read the IMU data (x,y,z accel and gyroscope). */
  msg = mpu6050GetData(&I2CD1, &rdp->imu);

  if (msg != MSG_OK) {
#if (DEBUG_ASSERV)
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

#if (DEBUG_ASSERV)
    if (printEnable == true) {
      chprintf(chp, "%s: The Robot is laying down.\n\r", __func__);
      printEnable = false;
    }
#endif

    layingDown = true;
    motorStop(&rdp->motorL);
    motorStop(&rdp->motorR);
    pidResetParameters(&rdp->pidPosition);
    pidResetParameters(&rdp->pidAngle);
    pidResetParameters(&rdp->pidMotorL);
    pidResetParameters(&rdp->pidMotorR);
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
    positionLMeasured = ((2*PI*R)/PPR)*rdp->encoderL.counter;
    positionRMeasured = ((2*PI*R)/PPR)*rdp->encoderR.counter;

    /* Update the Position PID. */
    rdp->pidPosition.consigne = targetPosition;     /* Distance setpoint.   */
    rdp->pidPosition.measure  = ((positionRMeasured + positionLMeasured)/2);  /* Distance measured.   */
    pidCompute(&rdp->pidPosition);                  /* Set the PID output.  */

    /* Update the Angle PID. */
    rdp->pidAngle.consigne = targetAngle - rdp->pidPosition.output;
    rdp->pidAngle.measure  = measuredAngle;
    pidCompute(&rdp->pidAngle);

    /* Manage PID for the left Motor. */
    rdp->pidMotorL.consigne = (rdp->pidAngle.output);
    rdp->pidMotorL.measure  = positionLMeasured;
    pidCompute(&rdp->pidMotorL);

    /* Manage PID for the Right Motor. */
    rdp->pidMotorR.consigne = (rdp->pidAngle.output);
    rdp->pidMotorR.measure  = positionRMeasured;
    pidCompute(&rdp->pidMotorR);

    /* Get the motors power that need to be apply. */
    rdp->motorL.speed = rdp->pidMotorL.output;
    rdp->motorR.speed = rdp->pidMotorR.output;

    /* Set power to the left motor. */
    motorMove(&rdp->motorL);

    /* Set power to the rigth motor. */
    motorMove(&rdp->motorR);

#if (DEBUG_ASSERV)
    //encoderGetDistance(&rdp->encoderLeft);
    //encoderGetDistance(&rdp->encoderRight);
    //chprintf(chp, "%s: angle = %.3f, distanceL = %.3f, distanceR = %.3f\r\n", __func__, rdp->pidAngle.measure, positionL, positionR);

    chprintf(chp, "%s: Tposi = %.3f, Mposi = %.3f, Tangle %.3f, Mangle %.3f\r\n", __func__, rdp->pidPosition.consigne, rdp->pidPosition.measure, rdp->pidAngle.consigne, rdp->pidAngle.measure);
#endif
  }
}

/** @} */
