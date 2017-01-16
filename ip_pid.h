/**
 *
 * @file    ip_pid.h
 *
 * @brief   pid corrector header file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    13 July 2016
 *
 * @update  16 January 2017
 *
 */

#ifndef _IP_PID_H_
#define _IP_PID_H_

/*===========================================================================*/
/* Include file.                                                             */
/*===========================================================================*/
#include "ip_motor.h"

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Functions.                                                                */
/*===========================================================================*/

void pid(double pitch, double restAngle, double offset, double turning);
void pidParametersReset(void);

#endif /* _IP_PID_H_ */
