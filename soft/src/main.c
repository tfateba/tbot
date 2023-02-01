/*
    TBOT - Copyright (C) 2015...2021 Theodore Ateba

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
 * @file    main.c
 * @brief   main file of inverted pendulum Robot.
 *
 * @addtogroup MAIN
 * @{
 */

/*==========================================================================*/
/* Includes Files.                                                          */
/*==========================================================================*/

/* ChibiOS files. */
#include "hal.h"
#include "ch.h"
#include "chprintf.h"

/* Project files. */
#include "asserv.h"
#include "buzzer.h"
#include "conf.h"
#include "encoder.h"
#include "i2c.h"
#include "kalman.h"
#include "main.h"
#include "motor.h"
#include "mpu6050.h"
#include "pid.h"
#include "pwm.h"
#include "hardware.h"

/*==========================================================================*/
/* Global variables, I2C TX and RX buffers, I2C and Serial Configurations.  */
/*==========================================================================*/

BaseSequentialStream* chp = (BaseSequentialStream*) &SD1;/**< Pointer for
                                                                the serial
                                                                stream to
                                                                output the
                                                                data on the
                                                                USB connector
                                                                of the arduino
                                                                board.      */

#define pr_debug(x) chprintf(chp, x)

ROBOTDriver tbot;

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
  .mid          = MOTOR_L,                /**< Motor ID.            */
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
  .mid          = MOTOR_R,                /**< Motor ID.            */
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

/* Left encoder Config. */
static ENCODERConfig encoderLConfig = {
  .id     = ENCODER_L,         /**< Encoderr identifier.         */
  .eichan = L_ENCODER_EXT_INT, /**< External interrupt channel.  */
  .porta  = L_ENCODER_PORT_A,  /**< Port A.                      */
  .portb  = L_ENCODER_PORT_B,  /**< Port B.                      */
  .pina   = L_ENCODER_PIN_A,   /**< Pin A.                       */
  .pinb   = L_ENCODER_PIN_B    /**< Pin B.                       */
};

/* Rigth encoder Config. */
static ENCODERConfig encoderRConfig = {
  .id     = ENCODER_R,         /**< Encoderr identifier.         */
  .eichan = R_ENCODER_EXT_INT, /**< External interrupt channel.  */
  .porta  = R_ENCODER_PORT_A,  /**< Port A.                      */
  .portb  = R_ENCODER_PORT_B,  /**< Port B.                      */
  .pina   = R_ENCODER_PIN_A,   /**< Pin A.                       */
  .pinb   = R_ENCODER_PIN_B    /**< Pin B.                       */
};

/*==========================================================================*/
/* Threads function and main function.                                      */
/*==========================================================================*/

/*
 * @brief   Onboard led Blink thread.
 */
static THD_WORKING_AREA(waBlink, 64);
static THD_FUNCTION(blinkThd, arg) {

  (void)arg;
  systime_t time = chVTGetSystemTimeX();
  uint16_t  init_time = 0;
  bool      spf = false; /* Song Played Flag. */

  chRegSetThreadName("Blinker");

  while (true) {
    time += MS2ST(100);
    if (init_time <= 200) {
      init_time++;
    }
    else {
      if (spf == false) {
        buzzerSound();
        buzzerStopSound();
        spf = true;
      }
      palTogglePad(IOPORT2, PORTB_LED1);
    }

    chThdSleepUntil(time);
  }
}

/*
 * @brief   tbot asservissement thread.
 */
static THD_WORKING_AREA(waAsser, 128);
static THD_FUNCTION(asserThd, arg) {

  (void)arg;
  systime_t time = chVTGetSystemTimeX();

  chRegSetThreadName("Asservissement");

  while (true) {
    time += MS2ST(10);
    asserv(&tbot);
    chThdSleepUntil(time);
  }
}

/*
 * Application entry point.
 */
int main(void) {

  static msg_t msg;

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /* Start the serial. */
  sdStart(&SD1, NULL);

  /* Buzzer initialization. */
  buzzerInit();

  /* Start beep. */
  buzzerSound();

  /* Stop beep. */
  buzzerStopSound();

#if (DEBUG_MAIN)
  /* TODO: Make a function, so that the code will be more clear. */
  pr_debug("\n\r");
  pr_debug("\n\r");
  pr_debug("\n\r    _   _           _");
  pr_debug("\n\r   | | | |         | |");
  pr_debug("\n\r   | |_| |__   ___ | |_");
  pr_debug("\n\r   | __| '_ \\ / _ \\| __|");
  pr_debug("\n\r   | |_| |_) | (_) | |_");
  pr_debug("\n\r    \\__|_.__/ \\___/ \\__|");

  pr_debug("\n\r");
  pr_debug("\n\rInverted Pendulum Robot");
  pr_debug("\n\rMade by tfateba, tf.ateba@gmail.com");
  pr_debug("\n\rControlled with ChibiOS/RT (trunk)");
  pr_debug("\n\rTarget board is an Arduino Mega");
  pr_debug("\n\rCopyrigth: 2015...2023");

  pr_debug("\n\r");
  pr_debug("\n\rInitialization started...");
#endif

  /* Turn off the debug LED. */
  palClearPad(IOPORT2, PORTB_LED1);

#if (DEBUG_MAIN)
  pr_debug("\n\rOn-board LED initialization done");
#endif

  /* Start I2C interface. */
  i2cStart(&I2CD1, &i2cConfig);

#if (DEBUG_MAIN)
  pr_debug("\n\rI2C bus interface initialization done");
#endif

  /* Init Kalman filter. */
  kalmanInit();

#if (DEBUG_MAIN)
  pr_debug("\n\rKalman filter initialization done");
#endif

  /* Init MPU module. */
   msg = mpu6050Init(&I2CD1, &tbot.imu, MPU6050_ADDR);

  if (msg != MSG_OK) {
#if (DEBUG_MAIN)
    pr_debug("\n\r Error while initialising the IMU");
#endif
    return -1;
  }

#if (DEBUG_MAIN)
  pr_debug("\n\rIMU sensor initialization started, please wait...");
#endif

  /* Start MPU calibration process. */
  //mpu6050Calibration(&I2CD1, &tbot.imu);

#if (DEBUG_MAIN)
  pr_debug("\n\rIMU sensor calibration done");
#endif

  /* Init Position PID controller. */
  pidInit(&tbot.pidPosition, 0, 0, 0); /* PI */

  /* Init Angle PID controller.  */
  pidInit(&tbot.pidAngle, 55.468, 0.554, 42.524); /* PID  */

  /* Init motors PID controllers.  */
  pidInit(&tbot.pidMotorL, 1, 0, 0);
  pidInit(&tbot.pidMotorR, 1, 0, 0);

#if (DEBUG_MAIN)
  pr_debug("\n\rPID initialization done");
#endif

  /* Init Motors. */
  motorInit(&tbot.motorL, motorLConfig);
  motorInit(&tbot.motorR, motorRConfig);

#if (DEBUG_MAIN)
  pr_debug("\n\rMotors initialization done");
#endif

  /* Init Encoders. */
  encoderInit(&tbot.encoderR, encoderRConfig);
  encoderInit(&tbot.encoderL, encoderLConfig);

#if (DEBUG_MAIN)
  pr_debug("\n\rEncoder initialization done");
#endif

#if (DEBUG_MAIN)
  pr_debug("\n\rInverted Pendulum Robot initialization done");
  pr_debug("\n\rBlink and Asservissement thread will be created and started");
#endif

  /* Create and starts the LED blinker thread. */
  chThdCreateStatic(waBlink, sizeof(waBlink), NORMALPRIO + 4, blinkThd, NULL);

  /* Create and starts asservissement thread. */
  chThdCreateStatic(waAsser, sizeof(waAsser), NORMALPRIO + 8, asserThd, NULL);

#if (DEBUG_MAIN)
  pr_debug("\n\rApplication started");
#endif

  while (TRUE) {
    chThdSleepMilliseconds(100);
  }
}

/** @} */
