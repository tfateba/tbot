
/**
 *
 * @file    ip_pwm.h
 *
 * @brief  PWM configuration and management header file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    30 June 2016
 *
 */

#ifndef IP_PWM_H
#define IP_PWM_H

/*==========================================================================*/
/* Functions prototypes.                                                    */
/*==========================================================================*/

void pwmInits(void);
void pwmSetPulseWidth(PWMDriver *pwmp, uint8_t channel, uint16_t width);
void pwmEnable(PWMDriver *pwmp, PWMConfig *pwmcfg, uint8_t channel);
void pwmDisable(PWMDriver *pwmp);
void pwmSetDutyCycle(uint8_t motor, uint8_t direction, uint16_t dutyCycle);

#endif /* IP_PWM_H */

