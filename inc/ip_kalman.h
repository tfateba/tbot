
/**
 *
 * @file    ip_kalman.h
 *
 * @brief   Kalman filter header file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    05 July 2016
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
 Web      :   http://www.tkjelectronics.com
 e-mail   :   kristianl@tkjelectronics.com

 Modify by:   Theodore Ateba, tf.ateba@gmail.com
 */

#ifndef IP_KALMAN_H
#define IP_KALMAN_H

/*==========================================================================*/
/* Functions prototypes.                                                    */
/*==========================================================================*/

void  kalmanInit(void);
float kalmanGetAngle(float newAngle, float newRate, float dt);
void  kalmanSetAngle(float newAngle);
float kalmanGetRate(void);
void  kalmanSetQangle(float newQ_angle);
void  kalmanSetQbias(float newQ_bias);
void  kalmanSetRmeasure(float newR_measure);
float kalmanGetQangle(void);
float kalmanGetQbias(void);
float kalmanGetRmeasure(void);

#endif /* IP_KALMAN_H */

