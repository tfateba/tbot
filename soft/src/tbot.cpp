/*
    TBOT - Copyright (C) 2015...2023 Theodore Ateba

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
 * @file    tbot.cpp
 * @brief   tbot source file.
 *
 * @addtogroup TBOT
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

/* Standard files. */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

/* ChibiOS files. */
#include "hal.h"
#include "chprintf.h"

/* Project files. */
#include "conf.h"
#include "hardware.h"
#include "kalman.hpp"
#include "main.h"
#include "motor.hpp"
#include "mpu6050.hpp"
#include "pid.hpp"
#include "pwm.hpp"
#include "asserv.hpp"
#include "tbot.hpp"
#include "i2c.hpp"

/*==========================================================================*/
/* Application macros.                                                      */
/*==========================================================================*/

/*==========================================================================*/
/* Global variables.                                                        */
/*==========================================================================*/


/* Extern variables. */
#if (DEBUG_TBOT)
extern BaseSequentialStream* chp; /* Pointer used for chpirntf. */
//#define pr_debug(x) chprintf(chp, x)
#endif

/*
 * @brief   PWM3 configuration structure.
 */
static PWMConfig motorLPwmCfg = {
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
static PWMConfig motorRPwmCfg = {
  512,  /* Not real clock.     */
  512,  /* Maximum PWM count.  */
  NULL,
  {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PH3 use as PWM, OC4A .*/
    {PWM_OUTPUT_DISABLED, NULL},    /* PH4 Not use as PWM.   */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PH5 use as PWM, 0C4C. */
  },
};

static MOTORConfig motorLConfig = {
  .id           = MOTOR_L,                /**< Motor ID.            */
  .maxSpeed     = MOTOR_MAX_SPEED,        /**< Motor max speed.     */
  .forwardPort  = L_MOTOR_PORT_FORWARD,   /**< Motor Forward  port. */
  .reversePort  = L_MOTOR_PORT_BACKWARD,  /**< Motor Backward port. */
  .enablePort   = L_MOTOR_PORT_ENABLE,    /**< Motor Enable   port. */
  .forwardPin   = L_MOTOR_PIN_FORWARD,    /**< Motor Forwxard pin.  */
  .reversePin   = L_MOTOR_PIN_BACKWARD,   /**< Motor Backward pin.  */
  .enablePin    = L_MOTOR_PIN_ENABLE,     /**< Motor enable   pin.  */
  .pwmDriver    = &PWMD3,                 /**< Motor pwm driver.    */
  .pwmConfig    = &motorLPwmCfg,          /**< Motor pwm config.    */
  .pwmChannel1  = 0,
  .pwmChannel2  = 2
};

static MOTORConfig motorRConfig = {
  .id           = MOTOR_R,                /**< Motor ID.            */
  .maxSpeed     = MOTOR_MAX_SPEED,        /**< Motor max speed.     */
  .forwardPort  = R_MOTOR_PORT_FORWARD,   /**< Motor Forward  port. */
  .reversePort  = R_MOTOR_PORT_BACKWARD,  /**< Motor Backward port. */
  .enablePort   = R_MOTOR_PORT_ENABLE,    /**< Motor Enable   port. */
  .forwardPin   = R_MOTOR_PIN_FORWARD,    /**< Motor Forwxard pin.  */
  .reversePin   = R_MOTOR_PIN_BACKWARD,   /**< Motor Backward pin.  */
  .enablePin    = R_MOTOR_PIN_ENABLE,     /**< Motor enable   pin.  */
  .pwmDriver    = &PWMD4,                 /**< Motor pwm driver.    */
  .pwmConfig    = &motorRPwmCfg,          /**< Motor pwm config.    */
  .pwmChannel1  = 1,
  .pwmChannel2  = 2
};

/*==========================================================================*/
/* Driver functions.                                                        */
/*==========================================================================*/

/**
 * @brief  Asservissement routine of the robot.
 *
 * @param[in] robot   the pointer to the robot driver
 */
void Tbot::init(void) {

  pr_debug("\n\rTbot initialization started.");

  /* Initialize buzzer. */
  buzzer.init();

  /* Play buzzer sound to notify the beginning of the initialization process. */
  buzzer.startSound();
  chThdSleepMilliseconds(10);
  buzzer.stopSound();

  /* Initialize Position PID. */
  pidPosition.init(0, 0, 0);

  /* Initialize Angle PID. */
  pidAngle.init(55.468, 0.554, 42.524);

  /* Initialize Motors PIDs. */
  pidMotorL.init(1, 0, 0);
  pidMotorR.init(1, 0, 0);

  /* Initialize Kalman Filter. */
  kalmanFilter.init();

  /* Initialize Motors. */
  motorL.setMaxSpeed(MOTOR_MAX_SPEED);
  motorR.setMaxSpeed(MOTOR_MAX_SPEED);
  motorL.init(&motorLConfig);
  motorR.init(&motorRConfig);

  /* Stop the motor. */
  motorL.stop();
  motorR.stop();

  /* Start I2C interface. */
  i2cStart(&I2CD1, &i2cConfig);
#if (DEBUG_MAIN)
  pr_debug("\n\rI2C bus interface initialization done.");
#endif

  /* MPU initialization. */
  imu.init(&I2CD1, MPU6050_ADDR);
  imu.doCalibration(&I2CD1);

  /* Initialize Encoders. */
  //encoderL.init();
  //encoderR.init();

  /* Initialize onbard LED. */
  led1.init(IOPORT2, PORTB_LED1);

  pr_debug("\n\rTbot initialization Done.");
}

#ifdef __cplusplus
}
#endif

/** @} */
