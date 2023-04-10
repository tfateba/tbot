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
//#include "encoder.hpp"
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

    time += TIME_MS2I(100);
    time++;
    if (init_time <= 200) {
      init_time++;
    }
    else {
      if (spf == false) {
        tbot.buzzer.startSound();
        chThdSleepMilliseconds(10);
        //tbot.buzzer.startSound();
        tbot.buzzer.stopSound();

        spf = true;
      }
      tbot.led1.toggle();
    }

    chThdSleepUntil(time);
  }
}

/*
 * @brief   tbot asservissement thread.
 */
static THD_WORKING_AREA(waAsserv, 128);
static THD_FUNCTION(asservThd, arg) {

  (void)arg;
  systime_t time = chVTGetSystemTimeX();

  chRegSetThreadName("Asserv");

  while (true) {
    time += TIME_MS2I(10);
    asserv.run(&tbot);
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
#endif

  tbot.init();

  /*  Starts the LED blinker thread.  */
  chThdCreateStatic(waBlink, sizeof(waBlink), NORMALPRIO, blinkThd, NULL);

  /* Create and starts asservissement thread. */
  chThdCreateStatic(waAsserv, sizeof(waAsserv), NORMALPRIO + 8, asservThd, NULL);

  while (true) {
    chThdSleepMilliseconds(1000);
  }
}
