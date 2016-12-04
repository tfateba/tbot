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
 * @update  17 November 2016
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

// TODO: Get the function prototypes from the c file.
/**
 * @fn      pid
 * @brief   Calcul the command to send to the motors according to last error.
 *
 * @param[in] restAngle
 * @param[in] offset
 * @param[in] turning
 */
void pid(double pitch, double restAngle, double offset, double turning);

/**
 * @fn      pidParametersReset
 * @brief   Reset the PID parameters.
 */
void pidParametersReset(void);

#endif /* _IP_PID_H_ */
