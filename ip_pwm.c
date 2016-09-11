/**
 *
 * @file    sam_pwm.c
 *
 * @brief   Configuration, management of PWM signal.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    30 June 2016
 *
 * @update	22 August 2016
 *
 */

/*==========================================================================*/
/* Includes Files                                                           */
/*==========================================================================*/
#include "hal.h"

/*==========================================================================*/
/* Globals variable and configurations                                      */
/*==========================================================================*/

/*==========================================================================*/
/* Functions.                                                               */
/*==========================================================================*/

/**
 * @fn    pwm_setPulseWidth
 * @brief Set the pulse width on the specify channel of a PWM driver.
 */
void pwm_setPulseWidth(PWMDriver *pwmp, uint8_t channel, uint16_t width)
{
  pwmEnableChannel(pwmp, channel, PWM_PERCENTAGE_TO_WIDTH(pwmp, width));
}

/**
 * @fn    pwm_init
 *@brief  Initialize the PWM output
 */
void pwm_init(PWMDriver *pwmp, PWMConfig *pwmcfg)
{
  pwmStart(pwmp, pwmcfg);
  palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(1));
  palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(1));
}

/**
 * @fn    pwm_enable
 * @brief Enable PWM channel
 */
void pwm_enable(PWMDriver *pwmp, PWMConfig *pwmcfg, uint8_t channel)
{
  pwmStart(pwmp, pwmcfg);
  pwm_setPulseWidth(pwmp, channel, 0);
}

/**
 * @fn    pwm_disable
 * @brief Disable PWM channel
 */
void pwm_disable(PWMDriver *pwmp)
{
  pwmStop(pwmp);
}
