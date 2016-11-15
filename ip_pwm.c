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
 * @update	24 October 2016
 *
 */

/*==========================================================================*/
/* Includes Files                                                           */
/*==========================================================================*/
#include "ip_pwm.h"

/*==========================================================================*/
/* Globals variable and configurations                                      */
/*==========================================================================*/

/*
 * PWM3 configuration.
 */
static PWMConfig pwm3cfg = {
  1023,   /* Not real clock */
  1023,   /* Maximum PWM count */
  NULL,
  {
    {PWM_OUTPUT_DISABLED, NULL},    /* PE3 use as PWM, OC3A */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PE4 Not use as PWM   */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PE5 use as PWM, 0C3C */
  },
};

/*
 * PWM4 configuration.
 */
static PWMConfig pwm4cfg = {
  1023,   /* Not real clock */
  1023,   /* Maximum PWM count */
  NULL,
  {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PH3 use as PWM, OC4A */
    {PWM_OUTPUT_DISABLED, NULL},    /* PH4 Not use as PWM   */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PH5 use as PWM, 0C4C */
  },
};

/*==========================================================================*/
/* Functions.                                                               */
/*==========================================================================*/

/**
 * @fn      pwm_init.
 * @brief   Initialize the PWM output
 *
 * @param[in] pwmp    pointer to the pwm driver
 * @param[in] pwmcfg  configuration of the pwm driver
 */
void pwm_init(void) {
  /* PH3 and PH5 are timer 4 pwm channels outputs */
  palSetPadMode(IOPORT8, PH3, PAL_MODE_OUTPUT_PUSHPULL); // left motor PWM forward
  palSetPadMode(IOPORT8, PH5, PAL_MODE_OUTPUT_PUSHPULL); // left motor PWM backward

  /* PE4 and PE5 are timer 3 pwm channels outputs */
  palSetPadMode(IOPORT5, PE4, PAL_MODE_OUTPUT_PUSHPULL); // rigth motor PWM forward
  palSetPadMode(IOPORT5, PE5, PAL_MODE_OUTPUT_PUSHPULL); // rigth motor PWM backward

  // Start PWM
  pwmStart(&PWMD4, &pwm4cfg);
  pwmStart(&PWMD3, &pwm3cfg);
}

/**
 * @fn      pwm_setPulseWidth
 * @brief   Set the pulse width on the specify channel of a PWM driver.
 *
 * @param[in] pwmp      pointer to the pwm driver
 * @param[in] channel   channel to control
 * @param[in] width     pwm width to generate
 */
void pwm_setPulseWidth(PWMDriver *pwmp, uint8_t channel, uint16_t width) {
  //pwmEnableChannel(pwmp, channel, PWM_PERCENTAGE_TO_WIDTH(pwmp, width));
  pwmEnableChannel(pwmp, channel, width);
}

/**
 * @fn      pwm_enable.
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
 * @fn      pwm_disable.
 * @brief   Disable PWM channel.
 *
 * @param[in] pwmp  pointer of the pwm driver to disable
 */
void pwm_disable(PWMDriver *pwmp) {
  pwmStop(pwmp);
}
/*
void pwmDemo(void) {
  int i;

  for (i = 0; i < 1023; i++) {
     pwm_setPulseWidth(&PWMD4, 0, i);
     pwm_setPulseWidth(&PWMD4, 2, i);
     pwm_setPulseWidth(&PWMD3, 1, i);
     pwm_setPulseWidth(&PWMD3, 2, i);

     chThdSleepMilliseconds(1);
    }
    for (i = 1023; i > 0; i--) {
      pwm_setPulseWidth(&PWMD4, 0, i);
      pwm_setPulseWidth(&PWMD4, 2, i);
      pwm_setPulseWidth(&PWMD3, 1, i);
      pwm_setPulseWidth(&PWMD3, 2, i);

      chThdSleepMilliseconds(1);
    }
}
*/
