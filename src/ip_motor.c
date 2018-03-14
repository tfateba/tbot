
/**
 *
 * @file          ip_motor.c
 *
 * @brief         Motor driver source file.
 *
 * @author        Theodore Ateba, tf.ateba@gmail.com
 *
 * @date          07 September 2015
 *
 * @description   Motor control and Encoder Read
 *                Description:
 *                Get the PWM control value from the I2C.
 *                Send the Encoder value to the I2C master when required.
 *                Motor Power:  white
 *                Motor Power:  yellow
 *                Encoder GND:  blue
 *                Encoder VCC:  green
 *                Encoder A:    black
 *                Encoder B:    Red
 */

/*
    IP - Copyright (C) 2015..2018 Theodore Ateba

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

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

/* Standard files. */
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

/* ChibiOS files. */
#include "hal.h"
#include "chprintf.h"

/* Project files. */
#include "ip_conf.h"
#include "ip_motor.h"
#include "ip_pwm.h"

/*==========================================================================*/
/* Global variables.                                                        */
/*==========================================================================*/

const uint16_t  maxSpeedValue = 512;  /**< Robot maximum speed value.       */

#if (DEBUG == TRUE || DEBUG_MOT == TRUE)
extern BaseSequentialStream* chp;
#endif

/*==========================================================================*/
/* Driver functions.                                                        */
/*==========================================================================*/

/**
 * @brief   Stop the corresponding motor.
 *
 * @param[in] motor   the motor to stop, right or left
 */
void motor_stop(motor_e motor) {

  if (motor == MOTOR_L) {
    pwm_set_duty_cycle(MOTOR_L, MOTOR_DIR_F, 0);
    palClearPad(LMD_EN_PORT, LMD_EN);
  }

  if (motor == MOTOR_R) {
    pwm_set_duty_cycle(MOTOR_R, MOTOR_DIR_F, 0);
    palClearPad(RMD_EN_PORT, RMD_EN);
  }
}

/**
 * @brief   Driving the motor to the given speed.
 *
 * @param[in] motor       the motor to pilot, right or left
 * @param[in] direction   the direction of the motor, backward or forward
 * @param[in] speedRaw    the speed to set the motor
 */
void motor_move(motor_e motor, uint8_t direction, float speedRaw) {

  int speed;

  if (speedRaw > maxSpeedValue)
    speedRaw = maxSpeedValue;

  speed = speedRaw*((float)PWMVALUE)/maxSpeedValue;
  pwm_set_duty_cycle(motor, direction, speed);
}

/**
 * @brief   Enable left or right motor with GPIO signal.
 *
 * @param[in] motor   the motor to be enable
 */
void motor_enable(motor_e motor) {

  if (motor == MOTOR_L)
    palSetPad(LMD_EN_PORT, LMD_EN);
  else
    palSetPad(RMD_EN_PORT, RMD_EN);
}

/**
 * @brief   Disable left or right motor with GPIO signal.
 *
 * @param[in] motor   the motor to be disable
 */
void motor_disable(motor_e motor) {

  if (motor == MOTOR_L)
    palClearPad(LMD_EN_PORT, LMD_EN);
  else
    palClearPad(RMD_EN_PORT, RMD_EN);
}

/**
 * @brief   Initialise all pins needs for motor control
 */
void motor_init(void) {

  /* Setup Left Motor Driver ( LMD ). */
  palSetPadMode(LMD_RPWM_PORT,  LMD_RPWM, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(LMD_LPWM_PORT,  LMD_LPWM, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(LMD_EN_PORT,    LMD_EN,   PAL_MODE_OUTPUT_PUSHPULL);

  /* Setup Right Motor Driver ( RMD ).  */
  palSetPadMode(RMD_RPWM_PORT,  RMD_RPWM, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(RMD_LPWM_PORT,  RMD_LPWM, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(RMD_EN_PORT,    RMD_EN,   PAL_MODE_OUTPUT_PUSHPULL);

  /* Enable Right and Left Motors.  */
  motor_enable(MOTOR_R);
  motor_enable(MOTOR_L);
}

