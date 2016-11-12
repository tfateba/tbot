/**
 *
 * @file    ip_asserv.c
 *
 * @brief   Asservissement of inverted pendulum Robot.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    07 Septembre 2015
 *
 * @update  11 November 2016
 *
 * @version 1.2
 *
 */

/*=========================================================================*/
/* Includes files.                                                         */
/*=========================================================================*/
#include "ip_asserv.h"

/*=========================================================================*/
/* Application macros.                                                     */
/*=========================================================================*/
/* MPU6050 device name */
#define mpu "MPU6050"
/* Debug message activation. */
#define DEBUG TRUE

/*=========================================================================*/
/* Global variables, I2C TX and RX buffers, I2C and Serial Configurations  */
/*=========================================================================*/
extern BaseSequentialStream* chp; /*                                       */
extern mpu6050_t       imu;       /**< MPU6050 instance.                   */
extern msg_t           msg;       /**< Message error.                      */

const uint8_t   delta = 10; /*                                             */

bool    layingDown = true;
double  targetAngle = 178; /**< The angle we want the robot to reach.      */
double  targetOffset = 0;  /**< Offset for going forward and backwrd.      */
double  turningOffset = 0; /**< Offset for turning left and right.         */

/*=========================================================================*/
/* Functions.                                                              */
/*=========================================================================*/

/**
 * @fn     asserv
 * @brief  Asservissement routine of the robot.
 */
void asserv(void) {
  msg = mpu6050_getData(&I2CD1, &imu);

  if (msg != MSG_OK) {
    chprintf(chp, "\n\r %s: Error while reading the %s sensor data.", mpu, mpu);
    return -1;
  }

  imu.pitch = (atan2(imu.y_accel, imu.z_accel) + 3.14)*(180/3.14);
  imu.pitch_k = kalman_getAngle(imu.pitch, (imu.x_gyro / 131.0), delta);

  #if (DEBUG == TRUE)
  chprintf(chp, " pitch_kalman is %.3f °c\r\n", imu.pitch_k);
  #endif

  if ((layingDown && (imu.pitch_k < 170 || imu.pitch_k > 190)) ||
    (!layingDown && (imu.pitch_k < 135 || imu.pitch_k > 225))) {
    /*
     * The robot is in a unsolvable position, so turn off both motors and
     * wait until it's vertical again.
     */
    layingDown = true;
    motorsStopAndReset();
  }
  else {
    /*
     * It's no longer laying down,
     * so we can try to stabilized the robot now.
     */
    layingDown = false;
    pid(imu.pitch_k, targetAngle, targetOffset, turningOffset);
  }

  /* Update wheel velocity every 100ms. */
  motorGetWheelVelocity();
}
