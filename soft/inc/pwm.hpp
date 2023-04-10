/*
    TBOT - Copyright (C) 2015...2023 Theodore Ateba

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

#include "hal.h"

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

/*==========================================================================*/
/* Enumerations, Structures and macros.                                     */
/*==========================================================================*/

/**
 * @brief Motors identifier enumeration.
 */
typedef enum {
  MOTOR_L,  /**< Left motor. */
  MOTOR_R,  /**< Right motor. */
} motor_id_t;

/**
 * @brief Motors enumerations
 */
typedef enum {
  MOTOR_DIR_F,  /**< Motor forward direction. */
  MOTOR_DIR_B,  /**< Motor backward direction. */
} motor_dir_t;

/**
 * @brief  The motor driver can handle a pwm frequency up to 20kHz.
 */
#define PWM_FREQUENCY 20000 /**< PWM frequency value. */

/**
 * @brief   Frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler,
 *          we use no prescaling so:
 *          frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2
 */
#define PWMVALUE F_CPU/PWM_FREQUENCY/2

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

/* @todo Rename this class interfacePwm. */
class Pwm {
    public:
    void init(PWMDriver *pdp, PWMConfig *pcp, ioportid_t fport, ioportid_t rport, uint8_t fpin, uint8_t rpin);
    void enable(PWMDriver *pwmp, PWMConfig *pwmcfg, uint8_t channel);
    void disable(PWMDriver *pwmp);
    void setDutyCycle(motor_id_t mid, motor_dir_t mdir, ioportid_t port, uint8_t pin, uint8_t chan1, uint8_t chan2, int volt);

    private:
    void setPulseWidth(PWMDriver *pwmp, uint8_t channel, uint16_t width);
};

#endif /* PWM_H */

/** @} */
