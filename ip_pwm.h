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
 */

#ifndef IP_PWM_H
#define IP_PWM_H

/*==========================================================================*/
/* Functions.                                                               */
/*==========================================================================*/

void pwmPinsInit(void);
void pwmSetPulseWidth(PWMDriver *pwmp, uint8_t channel, uint16_t width);
void pwmEnable(PWMDriver *pwmp, PWMConfig *pwmcfg, uint8_t channel);
void pwmDisable(PWMDriver *pwmp);

#endif /* IP_PWM_H */

