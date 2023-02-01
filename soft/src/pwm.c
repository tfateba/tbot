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
 * @file    pwm.c
 * @brief   PWM configuration and management source file.
 *
 * @addtogroup PWM
 * @{
 */

/*==========================================================================*/
/* Includes Files.                                                          */
/*==========================================================================*/

/* ChibiOS files. */
#include "hal.h"
#include "chprintf.h"

/* Project files. */
#include "conf.h"
#include "motor.h"
#include "hardware.h"

/*==========================================================================*/
/* Global variables.                                                        */
/*==========================================================================*/

const uint16_t maxPwmValue = 512; /**< Max PWM value.                       */

/* Extern variables. */
#if (DEBUG_PWM)
extern BaseSequentialStream*  chp;
#endif

/*==========================================================================*/
/* Configurations structure.                                                */
/*==========================================================================*/

/*==========================================================================*/
/* Driver functions.                                                        */
/*==========================================================================*/

/**
 * @brief   Initialize the PWM output for motor.
 */
void pwmInits(MOTORConfig *mdp) {

  palSetPadMode(mdp->forwardPort, mdp->forwardPin, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(mdp->reversePort, mdp->reversePin, PAL_MODE_OUTPUT_PUSHPULL);
  pwmStart(mdp->pwmDriver, mdp->pwmConfig);
}

/**
 * @brief   Set the pulse width on the specify channel of a PWM driver.
 *
 * @param[in] pwmp      pointer to the pwm driver
 * @param[in] channel   channel to control
 * @param[in] width     pwm width to generate
 */
static void pwmSetPulseWidth(PWMDriver *pwmp, uint8_t channel, uint16_t width) {

  pwmEnableChannel(pwmp, channel, width);
}

/**
 * @brief   Enable PWM channel.
 *
 * @param[in] pwmp      pointer to the pwm driver
 * @param[in] pwmcfg    configuration of the pwm driver
 * @param[in] channel   channel to enable
 */
void pwmEnable(PWMDriver *pwmp, PWMConfig *pwmcfg, uint8_t channel) {

  pwmStart(pwmp, pwmcfg);
  pwmSetPulseWidth(pwmp, channel, 0);
}

/**
 * @brief   Disable PWM channel.
 *
 * @param[in] pwmp  pointer of the pwm driver to disable
 */
void pwmDisable(PWMDriver *pwmp) {

  pwmStop(pwmp);
}

/**
 * @brief   Generate the corresponding PWM for speed control.
 *
 * @param[in] mId         the motor to pilot, rigth or left.
 * @param[in] direction   the direction of the motor, backward or forward.
 * @param[in] dutyCycle   the duty cycle to set the pwm.
 */
void pwmSetDutyCycle(MOTORDriver *mdp) {

  palSetPad(mdp->config.enablePort, mdp->config.enablePin);

  if (mdp->config.mid == MOTOR_L) {

    if (mdp->dir == MOTOR_DIR_F) {
      pwmSetPulseWidth(&PWMD4, mdp->config.pwmChannel1, maxPwmValue);
      pwmSetPulseWidth(&PWMD4, mdp->config.pwmChannel2, mdp->pwmValue);
    }
    else if (mdp->dir == MOTOR_DIR_B) {
      pwmSetPulseWidth(&PWMD4, mdp->config.pwmChannel1, mdp->pwmValue);
      pwmSetPulseWidth(&PWMD4, mdp->config.pwmChannel2, maxPwmValue);
    }
  }
  else if (mdp->config.mid == MOTOR_R) {

    if (mdp->dir == MOTOR_DIR_F) {
      pwmSetPulseWidth(&PWMD3, mdp->config.pwmChannel1, maxPwmValue);
      pwmSetPulseWidth(&PWMD3, mdp->config.pwmChannel2, mdp->pwmValue);
    }
    else if (mdp->dir == MOTOR_DIR_B) {
      pwmSetPulseWidth(&PWMD3, mdp->config.pwmChannel1, mdp->pwmValue);
      pwmSetPulseWidth(&PWMD3, mdp->config.pwmChannel2, maxPwmValue);
    }
  }
}

/** @} */
