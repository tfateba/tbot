
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
/* Structures of the PID controller.                                        */
/*==========================================================================*/

/**
 * @brief   PID driver data structure.
 */
struct PIDDriver {
  uint8_t id;
  float   kp;
  float   ki;
  float   kd;
  float   result;
};

/**
 * @brief   PID driver type definition.
 */
typedef struct PIDDriver PIDDriver;

/*==========================================================================*/
/* Functions prototypes.                                                    */
/*==========================================================================*/

void pid_init(float kpval, float kival, float kdval);
float pid(float pitch, float restAngle, float offset, float turning);
void pid_Reset_Parameters(void);

#endif /* IP_PID_H */

