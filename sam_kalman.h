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
 */

#ifndef __SAM_KALMAN_H_
#define __SAM_KALMAN_H_

void kalman_init(void);
double kalman_getAngle(double newAngle, double newRate, double dt);
void kalman_setAngle(double newAngle);
double kalman_getRate(void);
void kalman_setQangle(double newQ_angle);
void kalman_setQbias(double newQ_bias);
void kalman_setRmeasure(double newR_measure);
double kalman_getQangle(void);
double kalman_getQbias(void);
double kalman_getRmeasure(void);
#endif
