
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

/*
    IP - Copyright (C) 2015..2018 Theodore Ateba

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
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

