
/**
 *
 * @file    main.c
 *
 * @brief   Inverted pendulum Robot main source file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    07 Septembre 2015
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

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

/* ChibiOS files. */
#include "hal.h"
#include "ch.h"
#include "chprintf.h"

/* Project local files. */
#include "ip_asserv.h"
#include "ip_conf.h"
#include "ip_i2c.h"
#include "ip_kalman.h"
#include "ip_motor.h"
#include "ip_mpu6050.h"
#include "ip_pwm.h"

/*==========================================================================*/
/* Global variables, I2C TX and RX buffers, I2C and Serial Configurations.  */
/*==========================================================================*/

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
BaseSequentialStream* chp = (BaseSequentialStream*) &SD1; /**< Pointer for
                                                                the serial
                                                                stream to
                                                                output the
                                                                data on the
                                                                USB connector
                                                                of the arduino
                                                                board.      */
#endif

mpu6050_t       imu;        /**< MPU6050 instance.                          */
msg_t           msg;        /**< Message error.                             */

/*==========================================================================*/
/* Threads and main function.                                               */
/*==========================================================================*/

/*
 * @brief   Onboard led blink thread.
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
 * @brief   Robot asservissement thread.
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

/**
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

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r\n\r/*===========================================*/");
  chprintf(chp, "\n\r/*                                           */");
  chprintf(chp, "\n\r/* Self  balancing robot.                    */");
  chprintf(chp, "\n\r/*                                           */");
  chprintf(chp, "\n\r/* - Made by:   Theodore Ateba(tfateba).     */");
  chprintf(chp, "\n\r/* - RTOS:      ChibiOS trunk.               */");
  chprintf(chp, "\n\r/* - Target:    Arduino Mega2560.            */");
  chprintf(chp, "\n\r/* - Version:   1.2                          */");
  chprintf(chp, "\n\r/* - Copyrigth: 2015...2017                  */");
  chprintf(chp, "\n\r/*                                           */");
  chprintf(chp, "\n\r/*===========================================*/\n\r");
  chprintf(chp, "\n\r %s: Initialisation of the robot is starting:", __func__);
  chprintf(chp, "\n\r %s: Initialisation of Serial driver is done.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Turn off the debug LED. */
  palClearPad(IOPORT2, PORTB_LED1);

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r %s: Initialisation of onbaord LED is done.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Start I2C interface. */
  i2cStart(&I2CD1, &i2cConfig);

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r %s: Initialisation of I2C interface is done.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Init Kalman filter. */
  kalmanInit();

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r %s: Initialisation of Kalman filter is done.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Init MPU module. */
  msg = mpu6050Init(&I2CD1, &imu, MPU6050_ADDR);

  if (msg != MSG_OK) {
#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
    chprintf(chp, "\n\r %s: Initialisation of MPU6050 failed.", __func__);
#endif
    return -1;
  }

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r %s: Initialisation of MPU6050 is done.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Start MPU calibration process. */
  mpu6050Calibration(&I2CD1, &imu);

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r %s: Calibration of IMU sensor is done.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Init Motors. */
  motorInit();

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r %s: Initialisation of Motors is done.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Init PWM modules. */
  pwmInits();

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r %s: Initialisation of PWM module is done.", __func__);
  chThdSleepMilliseconds(10);
#endif

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r %s: Initialisation of the robot is done.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Create and starts the LED blinker thread. */
  chThdCreateStatic(waBlink, sizeof(waBlink), NORMALPRIO + 4, blinkThd, NULL);

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r %s: Creation of the Blink thread.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Create and starts asservissement thread. */
  chThdCreateStatic(waAsser, sizeof(waAsser), NORMALPRIO + 8, asserThd, NULL);

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r %s: Creation of the Asservissement thread.", __func__);
  chprintf(chp, "\n\r\n\r/*=============================================*/");
  chprintf(chp, "\n\r/* Robot application is started.               */");
  chprintf(chp, "\n\r/*=============================================*/");
  chThdSleepMilliseconds(10);
#endif

  while (TRUE) {
    chThdSleepMilliseconds(100);
  }
}

