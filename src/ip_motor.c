
/**
 *
 * @file          ipmotor.c
 *
 * @brief         motor driver source file.
 *
 * @author        Theodore Ateba, tf.ateba@gmail.com
 *
 * @date          07 Septembre 2015
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
#include "ip_encoder.h"
#include "ip_motor.h"
#include "ip_pid.h"
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
void motor_stop(uint8_t motor) {

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
 * @brief   Stop both motors and initialise wheels position, PID parameters.
 */
void motors_stop_and_reset(void) {

  motor_stop(MOTOR_R);
  motor_stop(MOTOR_L);
  pid_Reset_Parameters();
}

/**
 * @brief   Driving the motor to the given speed.
 *
 * @param[in] motor       the motor to pilot, right or left
 * @param[in] direction   the direction of the motor, backward or forward
 * @param[in] speedRaw    the speed to set the motor
 */
void motor_move(uint8_t motor, uint8_t direction, float speedRaw) {

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

