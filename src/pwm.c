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

/*
 * @brief   PWM3 configuration structure.
 */
static PWMConfig pwm3cfg = {
  512,  /* Not real clock.     */
  512,  /* Maximum PWM count.  */
  NULL,
  {
    {PWM_OUTPUT_DISABLED, NULL},    /* PE3 use as PWM, OC3A. */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PE4 Not use as PWM.   */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PE5 use as PWM, 0C3C. */
  },
};

/*
 * @brief   PWM4 configuration structure.
 */
static PWMConfig pwm4cfg = {
  512,  /* Not real clock.     */
  512,  /* Maximum PWM count.  */
  NULL,
  {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PH3 use as PWM, OC4A .*/
    {PWM_OUTPUT_DISABLED, NULL},    /* PH4 Not use as PWM.   */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PH5 use as PWM, 0C4C. */
  },
};

/*==========================================================================*/
/* Driver functions.                                                        */
/*==========================================================================*/

/**
 * @brief   Initialize the PWM output.
 */
void pwmInits(void) {

  /* PH3 and PH5 are timer 4 pwm channels outputs. */
  palSetPadMode(IOPORT8, PH3, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(IOPORT8, PH5, PAL_MODE_OUTPUT_PUSHPULL);

  /* PE4 and PE5 are timer 3 pwm channels outputs. */
  palSetPadMode(IOPORT5, PE4, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(IOPORT5, PE5, PAL_MODE_OUTPUT_PUSHPULL);

  /* Start PWM3 and PWM4. */
  pwmStart(&PWMD4, &pwm4cfg);
  pwmStart(&PWMD3, &pwm3cfg);
}

/**
 * @brief   Set the pulse width on the specify channel of a PWM driver.
 *
 * @param[in] pwmp      pointer to the pwm driver
 * @param[in] channel   channel to control
 * @param[in] width     pwm width to generate
 */
void pwmSetPulseWidth(PWMDriver *pwmp, uint8_t channel, uint16_t width) {

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
void pwmSetDutyCycle(motor_id_t mid, motor_dir_t dir, uint16_t dutyCycle) {

#if (DEBUG_PWM)
  chprintf(chp, "pwm: %d\t", dutyCycle);
#endif

  if (mid == MOTOR_L) {
    palSetPad(L_MOTOR_PORT_ENABLE, L_MOTOR_PIN_ENABLE);

    if (dir == MOTOR_DIR_F) {
      pwmSetPulseWidth(&PWMD4, 0, maxPwmValue);
      pwmSetPulseWidth(&PWMD4, 2, dutyCycle);
    }
    else if (dir == MOTOR_DIR_B) {
      pwmSetPulseWidth(&PWMD4, 0, dutyCycle);
      pwmSetPulseWidth(&PWMD4, 2, maxPwmValue);
    }
  }
  else if (mid == MOTOR_R) {
    palSetPad(R_MOTOR_PORT_ENABLE, R_MOTOR_PIN_ENABLE);

    if (dir == MOTOR_DIR_F) {
      pwmSetPulseWidth(&PWMD3, 1, maxPwmValue);
      pwmSetPulseWidth(&PWMD3, 2, dutyCycle);
    }
    else if (dir == MOTOR_DIR_B) {
      pwmSetPulseWidth(&PWMD3, 1, dutyCycle);
      pwmSetPulseWidth(&PWMD3, 2, maxPwmValue);
    }
  }
}

/** @} */
