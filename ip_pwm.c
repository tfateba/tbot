/**
 *
 * @file    ip_pwm.c
 *
 * @brief   Configuration, management of PWM signal source file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    30 June 2016
 *
 */

/*==========================================================================*/
/* Includes Files                                                           */
/*==========================================================================*/

/* Standard include files.  */
#include <stdint.h>

/* ChibiOS files.           */
#include "hal.h"

/*==========================================================================*/
/* Application macros.                                                      */
/*==========================================================================*/

#define PWM_FREQUENCY 512 /**< Frequency of the PWM driver. */
#define PWM_PERIOD    512 /**< Period of the PWM driver.    */

/*==========================================================================*/
/* Globals variable and configurations                                      */
/*==========================================================================*/

/**
 * @brief   PWM3 configuration structure.
 */
static PWMConfig pwm3cfg = {
  PWM_FREQUENCY,                      /* Not real clock.                */
  PWM_PERIOD,                         /* Maximum PWM count.             */
  NULL,                               /* No call back for the pwm pin.  */
  {
    {PWM_OUTPUT_DISABLED,     NULL},  /* PE3 use as PWM, OC3A.          */
    {PWM_OUTPUT_ACTIVE_HIGH,  NULL},  /* PE4 Not use as PWM.            */
    {PWM_OUTPUT_ACTIVE_HIGH,  NULL},  /* PE5 use as PWM, 0C3C.          */
  },
};

/*
 * @brief   PWM4 configuration structure.
 */
static PWMConfig pwm4cfg = {
  PWM_FREQUENCY,                      /* Not real clock.                */
  PWM_PERIOD,                         /* Maximum PWM count.             */
  NULL,                               /* No call back for the pwm pin.  */
  {
    {PWM_OUTPUT_ACTIVE_HIGH,  NULL},  /* PH3 use as PWM, OC4A.          */
    {PWM_OUTPUT_DISABLED,     NULL},  /* PH4 Not use as PWM.            */
    {PWM_OUTPUT_ACTIVE_HIGH,  NULL},  /* PH5 use as PWM, 0C4C.          */
  },
};

/*==========================================================================*/
/* Functions.                                                               */
/*==========================================================================*/

/**
 * @brief   Initialize the PWM output.
 */
void pwmPinsInit(void) {

  /* PH3 and PH5 are timer 4 pwm channels outputs. */
  palSetPadMode(IOPORT8, PH3, PAL_MODE_OUTPUT_PUSHPULL); /* left motor PWM forward.   */
  palSetPadMode(IOPORT8, PH5, PAL_MODE_OUTPUT_PUSHPULL); /* left motor PWM backward.  */

  /* PE4 and PE5 are timer 3 pwm channels outputs. */
  palSetPadMode(IOPORT5, PE4, PAL_MODE_OUTPUT_PUSHPULL); /* rigth motor PWM forward.  */
  palSetPadMode(IOPORT5, PE5, PAL_MODE_OUTPUT_PUSHPULL); /* rigth motor PWM backward. */

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

