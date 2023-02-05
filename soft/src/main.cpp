/*
    ChibiOS - Copyright (C) 2006..2022 Theodore Ateba

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

#include "ch.h"
#include "hal.h"

/*==========================================================================*/
/* Includes Files.                                                          */
/*==========================================================================*/

/* Project files. */
#include "conf.h"
#include "main.h"
#include "hardware.h"

#include "asserv.hpp"
#include "led.hpp"
#include "buzzer.hpp"
#include "encoder.hpp"
#include "i2c.hpp"
#include "kalman.hpp"
#include "motor.hpp"
#include "mpu6050.hpp"
#include "pid.hpp"
#include "pwm.hpp"
#include "tbot.hpp"

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

Tbot tbot;

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

// TODO this must be removed here!
Asserv  asserv;

/*==========================================================================*/
/* Threads function and main function.                                      */
/*==========================================================================*/

static virtual_timer_t asservvt;
/*
 * LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waBlink, 64);
static THD_FUNCTION(blinkThd, arg) {

  (void)arg;
  
  systime_t time = chVTGetSystemTimeX();
  int32_t   init_time = 0;
  bool      spf = false; /* Song Played Flag. */
  
  chRegSetThreadName("Blinker");

  while (true) {

    // //buzzer.playFailed();
    // tbot.led1.on();
    // chThdSleepMilliseconds(50);
    // tbot.led1.off();
    // chThdSleepMilliseconds(1000);

    // TODO Find the equivalent of this function !!!
    time += TIME_MS2I(100);
    time++;
    if (init_time <= 200) {
      init_time++;
    }
    else {
      if (spf == false) {
        tbot.buzzer.startSound();
        chThdSleepMilliseconds(10);
        tbot.buzzer.startSound();
        tbot.buzzer.stopSound();

        spf = true;
      }
      //palTogglePad(IOPORT2, PORTB_LED1);
      tbot.led1.toggle();
    }

    chThdSleepUntil(time);
    //chThdSleepUntil(10);
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
    time += TIME_MS2I(10);
    asserv.startAsserv(&tbot);
    chThdSleepUntil(time);
  }
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*  Activates the serial driver 1 using the driver default configuration. */
  sdStart(&SD1, NULL);

  /*  Initialize the led object. */
  tbot.led1.init(IOPORT2, PORTB_LED1);

  /* Buzzer initialization. */
  //tbot.buzzer.init();

  tbot.buzzer.startSound();
  chThdSleepMilliseconds(10);
  tbot.buzzer.startSound();
  tbot.buzzer.stopSound();

  // chThdSleepMilliseconds(100);
  // tbot.buzzer.playFailed();
  // chThdSleepMilliseconds(100);
  // tbot.buzzer.playPassed();
  // tbot.buzzer.baby();

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
  pr_debug("\n\rSoftware made by tfateba, tf.ateba@gmail.com");
  pr_debug("\n\rHardware made by Thibaud2399, thibaud.jenny@gmail.com");
  pr_debug("\n\rControlled with ChibiOS/RT (trunk)");
  pr_debug("\n\rRunning on Arduino Mega board");
  pr_debug("\n\rC++ port");
  pr_debug("\n\rCopyrigth: 2015...2023");

  pr_debug("\n\r");
  pr_debug("\n\rInitialization started...");
#endif

  /* Start I2C interface. */
  i2cStart(&I2CD1, &i2cConfig);

  tbot.init();

#if (DEBUG_MAIN)
  pr_debug("\n\rI2C bus interface initialization done");
#endif

  /*  Starts the LED blinker thread.  */
  chThdCreateStatic(waBlink, sizeof(waBlink), NORMALPRIO, blinkThd, NULL);

  /* Create and starts asservissement thread. */
  chThdCreateStatic(waAsser, sizeof(waAsser), NORMALPRIO + 8, asserThd, NULL);

  while (true) {
    chThdSleepMilliseconds(1000);
  }
}
