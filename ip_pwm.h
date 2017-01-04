/**
 *
 * @file    ip_pwm.h
 *
 * @brief   Configuration and management of PWM signal header file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    30 June 2016
 *
 * @update	07 December 2016
 *
 */
#ifndef _IP_PWM_H_
#define _IP_PWM_H_

/*===========================================================================*/
/* Includes Files                                                            */
/*===========================================================================*/
#include "ip_motor.h"

/*===========================================================================*/
/* Globals variable and configurations                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Functions.                                                                */
/*===========================================================================*/



/**
 * @fn      pwm_init
 * @brief   Initialize the PWM output.
 */
void pwm_init(void);

/**
 * @fn      pwm_setPulseWidth
 * @brief   Set the pulse width on the specify channel of a PWM driver.
 *
 * @param[in] pwmp      pointer to the pwm driver
 * @param[in] channel   channel to control
 * @param[in] width     pwm width to generate
 */
void pwm_setPulseWidth(PWMDriver *pwmp, uint8_t channel, uint16_t width);

/**
 * @fn      pwm_enable
 * @brief   Enable PWM channel.
 *
 * @param[in] pwmp      pointer to the pwm driver
 * @param[in] pwmcfg    configuration of the pwm driver
 * @param[in] channel   channel to enable
 */
void pwm_enable(PWMDriver *pwmp, PWMConfig *pwmcfg, uint8_t channel);

/**
 * @fn      pwm_disable
 * @brief   Disable PWM channel.
 *
 * @param[in] pwmp  pointer of the pwm driver to disable
 */
void pwm_disable(PWMDriver *pwmp);

#endif /* _IP__PWM_H_ */
