
/**
 *
 * @file    ipasserv.c
 *
 * @brief   Robot asservissement source file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    07 Septembre 2015
 *
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
#include "ipconf.h"
#include "ipkalman.h"
#include "ipmain.h"
#include "ipmotor.h"
#include "ipmpu6050.h"
#include "ippid.h"
#include "ippwm.h"

/*==========================================================================*/
/* Application macros.                                                      */
/*==========================================================================*/

#define PI          3.14159265359
#define RAD_TO_DEG  180/PI

/*==========================================================================*/
/* Global variables, I2C TX and RX buffers, I2C and Serial Configurations   */
/*==========================================================================*/

/* Local variables. */
bool  layingDown    = true;     /**< Robot position, down or not.           */
float targetAngle   = 180;      /**< The angle we want the robot to reach.  */
float targetOffset  = 0;        /**< Offset for going forward and backwrd.  */
float turningOffset = 0;        /**< Offset for turning left and right.     */

const float   dt = 0.01;         /**< Asservissement period.                */

/* Extern variables. */
#if (DEBUG == TRUE || DEBUG_ASS == TRUE)
extern BaseSequentialStream* chp; /*                                        */
#endif

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
  float pidlvalue;
  float pidrvalue;

  /* Read the IMU data (x,y,z accel and gyroscope). */
  msg = mpu6050GetData(&I2CD1, &rdp->imu);

  if (msg != MSG_OK) {
#if (DEBUG == TRUE || DEBUG_ASS == TRUE)
    chprintf(chp, "\n\r %s: Error while reading the MPU6050 sensor data.",
              __func__);
#endif
    return;
  }

  /* Calcul of the Pitch angle of the selbalancing robot. */
  rdp->imu.pitch = (atan2(rdp->imu.y_accel, rdp->imu.z_accel) + PI)*(RAD_TO_DEG);

  /* Get the Kalman estimation of the angle. */
  rdp->imu.pitch_k = kalmanGetAngle(rdp->imu.pitch, (rdp->imu.x_gyro / 131.0), dt);

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
    motorsStopAndReset();
  }
  else {
    /*
     * It's no longer laying down,
     * so we can try to stabilized the robot now.
     */
    layingDown = false;

    pidlvalue = pid(rdp->imu.pitch_k, targetAngle, targetOffset, turningOffset);
    pidrvalue = pid(rdp->imu.pitch_k, targetAngle, targetOffset, turningOffset);

    /* Set the left motor PWM value. */
    if (pidlvalue >= 0)
      motorMove(MOTOR_L, MOTOR_DIR_F, pidlvalue);
    else
      motorMove(MOTOR_L, MOTOR_DIR_B, abs(pidlvalue));

    /* Set the rigth motor PWM value. */
    if (pidrvalue >= 0)
      motorMove(MOTOR_R, MOTOR_DIR_F, pidrvalue);
    else
      motorMove(MOTOR_R, MOTOR_DIR_B, abs(pidrvalue));

#if (DEBUG == TRUE || DEBUG_ASS == TRUE)
    chprintf(chp, "%s: filtered pitch = %.3f\r\n", __func__, rdp->imu.pitch_k);
#endif
  }

  /* Update the robot wheel velocity every 100ms. */
  //motorGetWheelVelocity();
}

