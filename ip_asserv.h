/**
 *
 * @file    ip_asserv.h
 *
 * @brief   Robot asservissement header file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    07 Septembre 2015
 *
 * @update  17 November 2016
 *
 */

#ifndef _IP_ASSERV_H_
#define _IP_ASSERV_H_

/*===========================================================================*/
/* Includes files.                                                           */
/*===========================================================================*/
/* Standard libraries. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* ChibiOS libraries. */
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
/* Local files. */
//#include "ip_adc.h"
//#include "ip_i2c.h"
#include "ip_kalman.h"
#include "ip_motor.h"
#include "ip_mpu6050.h"
#include "ip_pid.h"
#include "ip_pwm.h"

/*===========================================================================*/
/* Enumerations, Structures and macros.                                      */
/*===========================================================================*/

/*===========================================================================*/
/* Functions prototypes.                                                     */
/*===========================================================================*/

/**
 * @fn     asserv
 * @brief  Asservissement routine of the robot.
 */
void asserv(void);

#endif
