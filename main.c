/*
    ChibiOS - Copyright (C) 2016 Theodore Ateba

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

/*========================================================================*/
/* Includes Files.                                                        */
/*========================================================================*/
// Standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
// ChibiOS libraries
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
// Local files
#include "ip_asserv.h"

/*=========================================================================*/
/* Application macros.                                                     */
/*=========================================================================*/
/* MPU6050 device name */
#define mpu "MPU6050"
/* Debug message activation. */
#define DEBUG TRUE

/*=========================================================================*/
/* Global variables, I2C TX and RX buffers, I2C and Serial Configurations  */
/*=========================================================================*/
BaseSequentialStream* chp = (BaseSequentialStream*) &SD1; /*               */
mpu6050_t       imu;        /**< MPU6050 instance.                         */
msg_t           msg;        /**< Message error.                            */

/*=========================================================================*/
/* Local functions.                                                        */
/*=========================================================================*/

/*=========================================================================*/
/* Threads and main function.                                              */
/*=========================================================================*/

/*
 * @brief Onboard led Blink thread.
 */
static WORKING_AREA(waBlink, 64);
static THD_FUNCTION(blinkThd, arg) {
  (void)arg;

  chRegSetThreadName("Blinker");

  while (true) {
    palTogglePad(IOPORT2, PORTB_LED1);
    chThdSleepMilliseconds(100);
  }
}

/*
 * @brief Robot asservissement thread.
 * @TODO: Find the correct size of the working area.
 */
static THD_WORKING_AREA(waAsser, 1024);
static THD_FUNCTION(asserThd, arg) {
  (void)arg;

  chRegSetThreadName("Asservissement");

  while (true) {
    asserv();
    chThdSleepMilliseconds(10);
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

  /* Start the serial. */
  sdStart(&SD1, NULL);
  #if (DEBUG == TRUE)
  chprintf(chp, "\n\r");
  chprintf(chp, "\n\r/*==============================================*/");
  chprintf(chp, "\n\r/*                                              */");
  chprintf(chp, "\n\r/*   Self  balancing robot.                     */");
  chprintf(chp, "\n\r/*                                              */");
  chprintf(chp, "\n\r/*    - Made by:   Theodore Ateba.              */");
  chprintf(chp, "\n\r/*    - RTOS:      ChibiOS v16.1.5              */");
  chprintf(chp, "\n\r/*    - Target:    Arduino Mega2560.            */");
  chprintf(chp, "\n\r/*    - Version:   1.2                          */");
  chprintf(chp, "\n\r/*    - Copyrigth: 2016.                        */");
  chprintf(chp, "\n\r/*                                              */");
  chprintf(chp, "\n\r/*==============================================*/");
  chprintf(chp, "\n\r");
  chprintf(chp, "\n\r Start Robot initialization:");
  chprintf(chp, "\n\r Serial driver initialization ended.");
  chThdSleepMilliseconds(3000);
  #endif

  /* Turn off the debug LED. */
  palClearPad(IOPORT2, PORTB_LED1);
  #if (DEBUG == TRUE)
  chprintf(chp, "\n\r On-board LED initialization ended.");
  chThdSleepMilliseconds(1000);
  #endif

  /* Start I2C interface. */
  i2cStart(&I2CD1, &i2cConfig);
  #if (DEBUG == TRUE)
  chprintf(chp, "\n\r I2C bus interface initialization ended.");
  chThdSleepMilliseconds(1000);
  #endif

  /* Init Kalman filter. */
  kalman_init();
  #if (DEBUG == TRUE)
  chprintf(chp, "\n\r Kalman filter initialization ended.");
  chThdSleepMilliseconds(1000);
  #endif

  /* Init MPU module. */
  msg = mpu6050_init(&I2CD1, &imu, MPU6050_ADDR);

  if (msg != MSG_OK) {
    chprintf(chp, "\n\r Error while initialising the mpu sensor.");
    return -1;
  }

  #if (DEBUG == TRUE)
  chprintf(chp, "\n\r IMU sensor initialization ended.");
  chThdSleepMilliseconds(1000);
  #endif

  /* Start MPU calibration process. */
  mpu6050_calibration(&I2CD1, &imu);
  #if (DEBUG == TRUE)
  chprintf(chp, "\n\r IMU sensor calibration ended.");
  chThdSleepMilliseconds(1000);
  #endif

  /* Init Motors. */
  motorInit();
  #if (DEBUG == TRUE)
  chprintf(chp, "\n\r Motors initialization ended.");
  chThdSleepMilliseconds(1000);
  #endif

  /* Init PWM modules. */
  pwm_init();
  #if (DEBUG == TRUE)
  chprintf(chp, "\n\r PWM initialization ended.");
  chThdSleepMilliseconds(1000);
  #endif

  #if (DEBUG == TRUE)
  chprintf(chp, "\n\r Robot initialization end.");
  chThdSleepMilliseconds(3000);
  #endif

  /* Create and starts the LED blinker thread. */
  chThdCreateStatic(waBlink, sizeof(waBlink), NORMALPRIO, blinkThd, NULL);
  #if (DEBUG == TRUE)
  chprintf(chp, "\n\r Create Blink thread.");
  chThdSleepMilliseconds(1000);
  #endif

  /* Create and starts asservissement thread. */
  chThdCreateStatic(waAsser, sizeof(waAsser), NORMALPRIO+1, asserThd, NULL);
  #if (DEBUG == TRUE)
  chprintf(chp, "\n\r Create Asservissement thread.");
  chThdSleepMilliseconds(1000);
  #endif

  while (TRUE) {
    chThdSleepMilliseconds(1000);
  }
}
