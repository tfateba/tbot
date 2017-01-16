/**
 *
 * @file    ip_pwm.c
 *
 * @brief   Configuration, management of PWM signal.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    30 June 2016
 *
 * @update  16 January 2017
 *
 */

/*===========================================================================*/
/* Includes Files.                                                           */
/*===========================================================================*/
#include "ip_pwm.h"

/*===========================================================================*/
/* Globals variable and configurations                                       */
/*===========================================================================*/

/*
 * @brief   PWM3 configuration structure.
 */
static PWMConfig pwm3cfg = {
  512,  /* Not real clock.     */
  512,  /* Maximum PWM count.  */
  NULL,
  {
    {PWM_OUTPUT_DISABLED, NULL},    /* PE3 use as PWM, OC3A.  */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PE4 Not use as PWM.    */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PE5 use as PWM, 0C3C.  */
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
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PH3 use as PWM, OC4A.  */
    {PWM_OUTPUT_DISABLED, NULL},    /* PH4 Not use as PWM.    */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PH5 use as PWM, 0C4C.  */
  },
};

/*===========================================================================*/
/* Functions.                                                                */
/*===========================================================================*/

/**
 * @brief   Initialize the PWM output.
 */
void pwm_init(void) {
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
void pwm_setPulseWidth(PWMDriver *pwmp, uint8_t channel, uint16_t width) {
  pwmEnableChannel(pwmp, channel, width);
}

/**
 * @brief   Enable PWM channel.
 *
 * @param[in] pwmp      pointer to the pwm driver
 * @param[in] pwmcfg    configuration of the pwm driver
 * @param[in] channel   channel to enable
 */
void pwm_enable(PWMDriver *pwmp, PWMConfig *pwmcfg, uint8_t channel) {
  pwmStart(pwmp, pwmcfg);
  pwm_setPulseWidth(pwmp, channel, 0);
}

/**
 * @brief   Disable PWM channel.
 *
 * @param[in] pwmp  pointer of the pwm driver to disable
 */
void pwm_disable(PWMDriver *pwmp) {
  pwmStop(pwmp);
}

