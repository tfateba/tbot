
/**
 *
 * @file    ip_asserv.c
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
#include <math.h>
#include <stdint.h>

/* ChibiOS files. */
#include "hal.h"
#include "chprintf.h"

/* Project local files. */
#include "ip_conf.h"
#include "ip_kalman.h"
#include "ip_motor.h"
#include "ip_mpu6050.h"
#include "ip_pid.h"
#include "ip_pwm.h"

/*==========================================================================*/
/* Application macros.                                                      */
/*==========================================================================*/

#define PI          3.14159265359 /**< Mathematical PI constant.            */
#define RAD_TO_DEG  180/PI        /**< Constant for Radian to degre.        */

/*==========================================================================*/
/* Global variables.                                                        */
/*==========================================================================*/

/* Local variables. */
bool    layingDown    = true; /**< Robot position, down or not.             */
double  targetAngle   = 180;  /**< The angle we want the robot to reach.    */
double  targetOffset  = 0;    /**< Offset for going forward and backwrd.    */
double  turningOffset = 0;    /**< Offset for turning left and right.       */

const double   dt     = 0.01; /**< Robot, asservissement period.            */

/* Extern variables. */
#if (DEBUG == TRUE || DEBUG_ASS == TRUE)
extern BaseSequentialStream*  chp;
#endif
extern mpu6050_t              imu;
extern msg_t                  msg;

/*==========================================================================*/
/* Functions.                                                               */
/*==========================================================================*/

/**
 * @brief  Asservissement routine of the robot.
 */
void asserv(void) {

  /* Read the IMU data (x,y,z accel and gyroscope). */
  msg = mpu6050GetData(&I2CD1, &imu);

  if (msg != MSG_OK) {
#if (DEBUG == TRUE || DEBUG_ASS == TRUE)
    chprintf(chp, "\n\r %s: Error while reading the MPU6050 sensor data.",
    __func__);
#endif
    return;
  }

  /* Calcul of the Pitch angle of the selbalancing robot. */
  imu.pitch = (atan2(imu.y_accel, imu.z_accel) + PI)*(RAD_TO_DEG);

  /* Get the Kalman estimation of the angle. */
  imu.pitch_k = kalmanGetAngle(imu.pitch, (imu.x_gyro / 131.0), dt);

  if ((layingDown && (imu.pitch_k < 170 || imu.pitch_k > 190)) ||
    (!layingDown && (imu.pitch_k < 135 || imu.pitch_k > 225))) {
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
    pid(imu.pitch_k, targetAngle, targetOffset, turningOffset);
#if (DEBUG == TRUE || DEBUG_ASS == TRUE)
  //chprintf(chp, "asserv: pitch = %.3f\r\n", (int8_t) imu.pitch_k);
  chprintf(chp, "%s: kalman pitch angle = %i\r\n",
  __func__, (uint8_t) imu.pitch_k);
#endif
  }

  /* Update the robot wheel velocity every 100ms. */
  //motorGetWheelVelocity();
}

