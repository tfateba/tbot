
/**
 *
 * @file    ip_pwm.c
 *
 * @brief   PWM configuration and management source file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    30 June 2016
 *
 */

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

/* ChibiOS libraries. */
#include "hal.h"

/* Project local files. */
#include "ip_conf.h"
#include "ip_motor.h"

/*==========================================================================*/
/* Global variables.                                                        */
/*==========================================================================*/

const   uint16_t  maxValue      = 512;    /**< Maximum value.               */

/*==========================================================================*/
/* Configurations structure.                                                */
/*==========================================================================*/

/*
 * @brief   PWM3 configuration structure.
 */
static PWMConfig pwm3cfg = {
  512,                              /* Period, Not real clock.        */
  512,                              /* Frequence, Maximum PWM count.  */
  NULL,                             /* pwm channel callback.          */
  {
    {PWM_OUTPUT_DISABLED, NULL},    /* PE3 use as PWM, OC3A.          */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PE4 Not use as PWM.            */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PE5 use as PWM, 0C3C.          */
  },
};

/*
 * @brief   PWM4 configuration structure.
 */
static PWMConfig pwm4cfg = {
  512,                              /* Period, Not real clock.        */
  512,                              /* Frequency, Maximum PWM count.  */
  NULL,                             /* pwm channel callback.          */
  {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PH3 use as PWM, OC4A.          */
    {PWM_OUTPUT_DISABLED, NULL},    /* PH4 Not use as PWM.            */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PH5 use as PWM, 0C4C.          */
  },
};

/*==========================================================================*/
/* Functions.                                                               */
/*==========================================================================*/

/**
 * @brief   Initialize the PWM output.
 */
void pwmInits(void) {

  /* PH3 and PH5 are timer 4 pwm channels outputs */
  palSetPadMode(IOPORT8, PH3, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(IOPORT8, PH5, PAL_MODE_OUTPUT_PUSHPULL);

  /* PE4 and PE5 are timer 3 pwm channels outputs */
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
 * @param[in] motor       the motor to pilot, rigth or left.
 * @param[in] direction   the direction of the motor, backward or forward.
 * @param[in] dutyCycle   the duty cycle to set the pwm.
 */
void pwmSetDutyCycle(uint8_t motor, uint8_t direction, uint16_t dutyCycle) {

#if (DEBUG == TRUE)
  chprintf(chp, "pwm: %d\t", dutyCycle);
#endif

  if (motor == MOTOR_L) {
    palSetPad(LMD_EN_PORT, LMD_EN);

    if (direction == MOTOR_DIR_F) {
      pwmSetPulseWidth(&PWMD4, 0, maxValue);
      pwmSetPulseWidth(&PWMD4, 2, dutyCycle);
    }
    else if (direction == MOTOR_DIR_B) {
      pwmSetPulseWidth(&PWMD4, 0, dutyCycle);
      pwmSetPulseWidth(&PWMD4, 2, maxValue);
    }
  }
  else if (motor == MOTOR_R) {
    palSetPad(RMD_EN_PORT, RMD_EN);

    if (direction == MOTOR_DIR_F) {
      pwmSetPulseWidth(&PWMD3, 1, maxValue);
      pwmSetPulseWidth(&PWMD3, 2, dutyCycle);
    }
    else if (direction == MOTOR_DIR_B) {
      pwmSetPulseWidth(&PWMD3, 1, dutyCycle);
      pwmSetPulseWidth(&PWMD3, 2, maxValue);
    }
  }
}

