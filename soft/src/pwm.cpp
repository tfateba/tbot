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
#include "hardware.h"
#include "motor.hpp"
#include "pwm.hpp"

#ifdef __cplusplus
extern "C" {
#endif

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
 * @brief   Initialize and start the PWM output for motor.
 *
 * @param[in] mp  pointer to the motor that pwm need to be initialized.
 */
void Pwm::init(PWMDriver *pdp, PWMConfig *pcp, ioportid_t fport, ioportid_t rport, uint8_t fpin, uint8_t rpin) {

  palSetPadMode(fport, fpin, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(rport, rpin, PAL_MODE_OUTPUT_PUSHPULL);
  pwmStart(pdp, pcp);
}

/**
 * @brief   Set the pulse width on the specify channel of a PWM driver.
 *
 * @param[in] pwmp      pointer to the pwm driver
 * @param[in] channel   channel to control
 * @param[in] width     pwm width to generate
 */
void Pwm::setPulseWidth(PWMDriver *pwmp, uint8_t channel, uint16_t width) {

  pwmEnableChannel(pwmp, channel, width);
}

/**
 * @brief   Enable PWM channel.
 *
 * @param[in] pwmp      pointer to the pwm driver
 * @param[in] pwmcfg    configuration of the pwm driver
 * @param[in] channel   channel to enable
 */
void Pwm::enable(PWMDriver *pwmp, PWMConfig *pwmcfg, uint8_t channel) {

  pwmStart(pwmp, pwmcfg);
  setPulseWidth(pwmp, channel, 0);
}

/**
 * @brief   Disable PWM channel.
 *
 * @param[in] pwmp  pointer of the pwm driver to disable
 */
void Pwm::disable(PWMDriver *pwmp) {

  pwmStop(pwmp);
}

/**
 * @brief   Generate the corresponding PWM for speed control.
 *
 * @param[in] mId         the motor to pilot, rigth or left.
 * @param[in] direction   the direction of the motor, backward or forward.
 * @param[in] dutyCycle   the duty cycle to set the pwm.
 */
void Pwm::setDutyCycle(motor_id_t mid, motor_dir_t mdir, ioportid_t port, uint8_t pin, uint8_t chan1, uint8_t chan2, int volt)  {

  palSetPad(port, pin);

  if (MOTOR_L == mid) {

    if (MOTOR_DIR_F == mdir) {
      setPulseWidth(&PWMD4, chan1, maxPwmValue);
      setPulseWidth(&PWMD4, chan2, volt);
    }
    else if (MOTOR_DIR_B == mdir) {
      setPulseWidth(&PWMD4, chan1, volt);
      setPulseWidth(&PWMD4, chan2, maxPwmValue);
    }
  }
  else if (MOTOR_R == mid) {

    if (MOTOR_DIR_F == mdir) {
      setPulseWidth(&PWMD3, chan1, maxPwmValue);
      setPulseWidth(&PWMD3, chan2, volt);
    }
    else if (MOTOR_DIR_B == mdir) {
      setPulseWidth(&PWMD3, chan1, volt);
      setPulseWidth(&PWMD3, chan2, maxPwmValue);
    }
  }
}

#ifdef __cplusplus
}
#endif

/** @} */
