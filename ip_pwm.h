/**
 *
 * @file    ip_pwm.h
 *
 * @brief   Configuration and management of PWM signal header file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    30 June 2016
 *
 * @update	16 January 2017
 *
 */
#ifndef _IP_PWM_H_
#define _IP_PWM_H_

/*===========================================================================*/
/* Includes Files.                                                           */
/*===========================================================================*/
#include "ip_motor.h"

/*===========================================================================*/
/* Globals variable and configurations.                                      */
/*===========================================================================*/

/*===========================================================================*/
/* Functions.                                                                */
/*===========================================================================*/

void pwm_init(void);
void pwm_setPulseWidth(PWMDriver *pwmp, uint8_t channel, uint16_t width);
void pwm_enable(PWMDriver *pwmp, PWMConfig *pwmcfg, uint8_t channel);
void pwm_disable(PWMDriver *pwmp);

#endif /* _IP__PWM_H_ */
