/*
    TBOT - Copyright (C) 2015...2021 Theodore Ateba

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

/**
 * @file    pwm.h
 * @brief   PWM configuration and management header file.
 *
 * @addtogroup PWM
 * @{
 */

#ifndef PWM_H
#define PWM_H

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

/*==========================================================================*/
/* Enumerations, Structures and macros.                                     */
/*==========================================================================*/

/**
 * @brief  The motor driver can handle a pwm frequency up to 20kHz.
 */
#define PWM_FREQUENCY 20000 /**< PWM frequency value. */

/**
 * Frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, we use no
 * prescaling so:
 * frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2
 */
#define PWMVALUE F_CPU/PWM_FREQUENCY/2

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

void pwmInits(void);
void pwmSetPulseWidth(PWMDriver *pwmp, uint8_t channel, uint16_t width);
void pwmEnable(PWMDriver *pwmp, PWMConfig *pwmcfg, uint8_t channel);
void pwmDisable(PWMDriver *pwmp);
void pwmSetDutyCycle(motor_id_t mid, motor_dir_t dir, uint16_t dutyCycle);

#ifdef __cplusplus
}
#endif

#endif /* PWM_H */

/** @} */
