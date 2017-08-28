
/**
 *
 * @file    ip_pid.h
 *
 * @brief   PID corrector header file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    13 July 2016
 *
 */

#ifndef IP_PID_H
#define IP_PID_H

/*==========================================================================*/
/* Functions prototypes.                                                    */
/*==========================================================================*/

void pid(double pitch, double restAngle, double offset, double turning);
void pidResetParameters(void);

#endif /* IP_PID_H */

