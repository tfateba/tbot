// TODO: Make a test to use a mutex, binary semaphore and count semaphore.

/**
 *
 * @file    main.c
 *
 * @brief   main file of inverted pendulum Robot.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    07 Septembre 2015
 *
 * @update  08 December 2016
 *
 */

/*
    IP - Copyright (C) 2016 Theodore Ateba

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

/*===========================================================================*/
/* Includes Files.                                                           */
/*===========================================================================*/
#include "ip_asserv.h"
#include "ip_conf.h"

/*===========================================================================*/
/* Application macros.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Global variables, I2C TX and RX buffers, I2C and Serial Configurations.   */
/*===========================================================================*/
#if (DEBUG == TRUE)
BaseSequentialStream* chp = (BaseSequentialStream*) &SD1; /*                 */
#endif

mpu6050_t       imu;        /**< MPU6050 instance.                           */
msg_t           msg;        /**< Message error.                              */

/*===========================================================================*/
/* Local functions.                                                          */
/*===========================================================================*/

/*===========================================================================*/
/* Threads and main function.                                                */
/*===========================================================================*/

/*
 * @brief Onboard led Blink thread.
 */
static THD_WORKING_AREA(waBlink, 32);
static THD_FUNCTION(blinkThd, arg) {
  (void)arg;
  systime_t time = chVTGetSystemTimeX();
  uint16_t init_time = 0;

  chRegSetThreadName("Blinker");

  while (true) {
    time += MS2ST(100);
    if (init_time <= 200) {
      init_time++;
    }
    else {
      palTogglePad(IOPORT2, PORTB_LED1);
    }

    chThdSleepUntil(time);
  }
}

/*
 * @brief Robot asservissement thread.
 * @TODO: Find the correct size of the working area.
 */
static THD_WORKING_AREA(waAsser, 64);
static THD_FUNCTION(asserThd, arg) {
  (void)arg;
  systime_t time = chVTGetSystemTimeX();

  chRegSetThreadName("Asservissement");

  while (true) {
    time += MS2ST(10);
    asserv();
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
  chThdSleepMilliseconds(10);
#endif

  /* Turn off the debug LED. */
  palClearPad(IOPORT2, PORTB_LED1);
#if (DEBUG == TRUE)
  chprintf(chp, "\n\r On-board LED initialization ended.");
  chThdSleepMilliseconds(10);
#endif

  /* Start I2C interface. */
  i2cStart(&I2CD1, &i2cConfig);
#if (DEBUG == TRUE)
  chprintf(chp, "\n\r I2C bus interface initialization ended.");
  chThdSleepMilliseconds(10);
#endif

  /* Init Kalman filter. */
  kalman_init();
#if (DEBUG == TRUE)
  chprintf(chp, "\n\r Kalman filter initialization ended.");
  chThdSleepMilliseconds(10);
#endif

  /* Init MPU module. */
  msg = mpu6050_init(&I2CD1, &imu, MPU6050_ADDR);

  if (msg != MSG_OK) {
#if (DEBUG == TRUE)
    chprintf(chp, "\n\r Error while initialising the mpu sensor.");
#endif
    return -1;
  }

#if (DEBUG == TRUE)
  chprintf(chp, "\n\r IMU sensor initialization ended.");
  chThdSleepMilliseconds(10);
#endif

  /* Start MPU calibration process. */
  mpu6050_calibration(&I2CD1, &imu);
#if (DEBUG == TRUE)
  chprintf(chp, "\n\r IMU sensor calibration ended.");
  chThdSleepMilliseconds(10);
#endif

  /* Init Motors. */
  motorInit();
#if (DEBUG == TRUE)
  chprintf(chp, "\n\r Motors initialization ended.");
  chThdSleepMilliseconds(10);
#endif

  /* Init PWM modules. */
  pwm_init();
#if (DEBUG == TRUE)
  chprintf(chp, "\n\r PWM initialization ended.");
  chThdSleepMilliseconds(10);
#endif

#if (DEBUG == TRUE)
  chprintf(chp, "\n\r Robot initialization end.");
  chThdSleepMilliseconds(10);
#endif

  /* Create and starts the LED blinker thread. */
  chThdCreateStatic(waBlink, sizeof(waBlink), NORMALPRIO + 4, blinkThd, NULL);
#if (DEBUG == TRUE)
  chprintf(chp, "\n\r Create Blink thread.");
  chThdSleepMilliseconds(10);
#endif

  /* Create and starts asservissement thread. */
  chThdCreateStatic(waAsser, sizeof(waAsser), NORMALPRIO + 8, asserThd, NULL);
#if (DEBUG == TRUE)
  chprintf(chp, "\n\r Create Asservissement thread.");
  chThdSleepMilliseconds(10);
#endif

  while (TRUE) {
    chThdSleepMilliseconds(100);
  }
}
