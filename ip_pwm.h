/*
 * TODO Find how to create a copy rigth for my robot such as the chibios 
 * copy rigth.
 */


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
 * @update	22 August 2016
 *
 * @TODO:		Complete the function description.(parameters, returned value
 */
#ifndef _IP_PWM_H_
#define _IP_PWM_H_

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
 * @fn      pwm_init.
 * @brief   Initialize the PWM output.
 */
void pwm_init(void);
//void pwm_init(PWMDriver *pwmp, PWMConfig *pwmcfg);

/**
 * @fn      pwm_enable.
 * @brief   Enable PWM channel.
 */
void pwm_enable(PWMDriver *pwmp, PWMConfig *pwmcfg, uint8_t channel);

/**
 * @fn      pwm_setPulseWidth.
 * @brief   Set the pulse width on the specify channel of a PWM driver.
 */
void pwm_setPulseWidth(PWMDriver *pwmp, uint8_t channel, uint16_t width);

/**
 * @fn      pwm_disable.
 * @brief   Disable PWM channel.
 */
void pwm_disable(PWMDriver *pwmp);

//void pwmDemo(void);

#endif /* _IP__PWM_H_ */