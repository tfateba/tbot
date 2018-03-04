
/**
 *
 * @file    ip_pwm.h
 *
 * @brief   PWM configuration and management header file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    30 June 2016
 *
 */

#ifndef IP_PWM_H
#define IP_PWM_H

/*==========================================================================*/
/* Enumerations, Structures and macros.                                     */
/*==========================================================================*/

/**
 * @brief  The motor driver can handle a pwm frequency up to 20kHz.
 */
#define PWM_FREQUENCY 20000 /**< PWM frequency value. */

/*
 * Frequency is given by F_CPU/(2*N*ICR) - where N is the pre-scaler,
 * we use no pre-scaling so:
 * frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2
 */
#define PWMVALUE F_CPU/PWM_FREQUENCY/2

/*==========================================================================*/
/* Functions prototypes.                                                    */
/*==========================================================================*/

void pwm_inits(void);
void pwm_set_pulse_width(PWMDriver *pwmp, uint8_t channel, uint16_t width);
void pwm_enable(PWMDriver *pwmp, PWMConfig *pwmcfg, uint8_t channel);
void pwm_disable(PWMDriver *pwmp);
void pwm_set_duty_cycle(uint8_t motor, uint8_t direction, uint16_t dutyCycle);

#endif /* IP_PWM_H */

