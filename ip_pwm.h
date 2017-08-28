
/**
 *
 * @file    ip_pwm.h
 *
<<<<<<< HEAD
 * @brief  PWM configuration and management header file.
=======
 * @brief   PWM configuration and management header file.
>>>>>>> 16.1.5
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    30 June 2016
 *
 */
<<<<<<< HEAD

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

=======

#ifndef IP_PWM_H
#define IP_PWM_H

/*==========================================================================*/
/* Enumerations, Structures and macros.                                     */
/*==========================================================================*/

/*
 * The motor driver can handle a pwm frequency up to 20kHz
 */
#define PWM_FREQUENCY 20000

/*
 * Frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, we use no
 * prescaling so:
 * frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2
 */
#define PWMVALUE F_CPU/PWM_FREQUENCY/2

/*==========================================================================*/
/* Functions.                                                               */
/*==========================================================================*/

void pwmInits(void);
void pwmSetPulseWidth(PWMDriver *pwmp, uint8_t channel, uint16_t width);
void pwmEnable(PWMDriver *pwmp, PWMConfig *pwmcfg, uint8_t channel);
void pwmDisable(PWMDriver *pwmp);
void pwmSetDutyCycle(uint8_t motor, uint8_t direction, uint16_t dutyCycle);

>>>>>>> 16.1.5
#endif /* IP_PWM_H */

