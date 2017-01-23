/**
 *
 * @file    ip_kalman.c
 *
 * @brief   I2C driver
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    05 Jully 2016
 *
 */

 /* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com

  Modify by: Theodore Ateba, tf.ateba@gmail.com
 */

/*===========================================================================*/
/* Include Libraries                                                         */
/*===========================================================================*/

#include "ip_kalman.h"

/*===========================================================================*/
/* Application macros.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Global variables.                                                         */
/*===========================================================================*/

/* Kalman filter variables */
double Q_angle;   /**< Process noise variance for the accelerometer.         */
double Q_bias;    /**< Process noise variance for the gyro bias.             */
double R_measure; /**< The variance of the measurement noise.                */

double angle;     /**< The angle calculated by the Kalman filter.            */
double bias;      /**< The gyro bias calculated by the Kalman filter.        */
double rate;      /**< Unbiased rate calculated from the rate and the
                       calculated bias.                                      */

double P[2][2];   /**< Error covariance matrix.                              */
double K[2];      /**< Kalman gain.                                          */
double y;         /**< Angle difference.                                     */
double S;         /**< Estimate error.                                       */

/*===========================================================================*/
/* Functions.                                                                */
/*===========================================================================*/

/**
 * @brief  Initialize the Kalman filter.
 */
void kalman_init(void) {
  /* We will set the variables like so, these can also be tuned by the user */
  Q_angle   = 0.001;
  Q_bias    = 0.003;
  R_measure = 100;
  angle     = 0;      // Reset the angle
  bias      = 0;      // Reset bias

  /*
   * Since we assume that the bias is 0 and we know the starting
   * angle (use setAngle), the error covariance matrix is set like so see:
   * http://en.wikipedia.org/wiki/Kalman_filter#
   * Example_application.2C_technical
   */
  P[0][0] = 0;
  P[0][1] = 0;
  P[1][0] = 0;
  P[1][1] = 0;
};

/**
 * @brief   Calcul the estimate angle by using the kalman filter.
 *
 * @note    The angle should be in degrees.
 * @note    The rate should be in degrees per second.
 * @note    The delta time should be in seconds.
 *
 * @param[in] newAngle  current angle mesured by the IMU
 * @param[in] newRate   current angle rate mesured by the IMU
 *
 * @return    angle     the result of the kalman computing
 */
double kalman_getAngle(double newAngle, double newRate, double dt) {
  /*
   * KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
   * Modified by Kristian Lauszus
   * Modified by Theodore Ateba
   */

  /* Step 1 */
  rate = newRate - bias;
  angle += dt * rate;

  /* Update estimation error covariance - Project the error covariance ahead */
  /* Step 2 */
  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  /* Discrete Kalman filter measurement update equations
   * - Measurement Update ("Correct")
   * Calculate Kalman gain - Compute the Kalman gain
   */

  /* Step 4 */
  S = P[0][0] + R_measure;

  /* Step 5 */
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  /* Calculate angle and bias - Update estimate with measurement zk (newAngle)*/
  /* Step 3 */
  y = newAngle - angle;

  /* Step 6 */
  angle += K[0] * y;
  bias += K[1] * y;

  /* Calculate estimation error covariance - Update the error covariance */
  /* Step 7 */
  P[0][0] -= K[0] * P[0][0];
  P[0][1] -= K[0] * P[0][1];
  P[1][0] -= K[1] * P[0][0];
  P[1][1] -= K[1] * P[0][1];

  return angle;
}

/**
 * @brief   Used to set angle, this should be set as the starting angle
 *
 * @param[in] newAngle  Set kalman filter starting angle
 */
void set_angle(double newAngle) {
  angle = newAngle;
}

/**
 * @brief   Read the rate of the angle.
 *
 * @return  rate  the angle rate
 */
double kalman_getRate(void) {
  return rate;
}

/**
 * @brief   Tune the kalman filter.
 *
 * @param[in] newQ_angle  angle used to tune the Kalman filter
 */
void kalman_setQangle(double newQ_angle) {
  Q_angle = newQ_angle;
}

/**
 * @brief   Tune the kalman filter.
 *
 * @param[in] newQ_bias   bias angle used to set the Kalman filter
 */
void kalman_setQbias(double newQ_bias) {
  Q_bias = newQ_bias;
}

/**
 * @brief   set the variance of the measurement noise.
 *
 * @param[in] newR_measure  variance of the measurement noise value.
 */
void kalman_setRmeasure(double newR_measure) {
  R_measure = newR_measure;
}

/**
 * @brief   Read the process noise variance for the gyro bias.
 *
 * @return  Q_angle process noise variance for the gyro bias
 */
double kalman_getQangle(void) {
  return Q_angle;
}

/**
 * @brief   Read the process noise variance for the accelerometer.
 *
 * @return  Q_bias  process noise variance for the accelerometer
 */
double kalman_getQbias(void) {
  return Q_bias;
}

/**
 * @brief   Read the process noise variance for the gyro bias.
 *
 * @return  R_measure   process noise variance for the gyro bias.
 */
double kalman_getRmeasure(void) {
  return R_measure;
}
