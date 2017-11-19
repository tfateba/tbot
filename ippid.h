
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

struct PIDDriver {
  uint8_t id;
  float   kp;
  float   ki;
  float   kd;
  float   result;
};

typedef struct PIDDriver PIDDriver;

/*==========================================================================*/
/* Functions prototypes.                                                    */
/*==========================================================================*/

void pidInit(float kpval, float kival, float kdval);
float pid(float pitch, float restAngle, float offset, float turning);
void pidResetParameters(void);

#endif /* IPPID_H */

