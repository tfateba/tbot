
/**
 *
 * @file    ippid.h
 *
 * @brief   PID corrector header file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    13 July 2016
 *
 */

#ifndef IPPID_H
#define IPPID_H

/*==========================================================================*/
/* Functions prototypes.                                                    */
/*==========================================================================*/

void pid(double pitch, double restAngle, double offset, double turning);
void pidResetParameters(void);

#endif /* IPPID_H */

