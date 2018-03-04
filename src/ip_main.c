
/**
 *
 * @file    ip_main.c
 *
 * @brief   main file of inverted pendulum Robot.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    07 September 2015
 *
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
/* Includes Files.                                                          */
/*==========================================================================*/

/* ChibiOS files. */
#include "hal.h"
#include "ch.h"
#include "chprintf.h"

/* Project files. */
#include "ip_asserv.h"
#include "ip_buzzer.h"
#include "ip_conf.h"
#include "ip_encoder.h"
#include "ip_i2c.h"
#include "ip_kalman.h"
#include "ip_main.h"
#include "ip_motor.h"
#include "ip_mpu6050.h"
#include "ip_pid.h"
#include "ip_pwm.h"

/*==========================================================================*/
/* Global variables, I2C TX and RX buffers, I2C and Serial Configurations.  */
/*==========================================================================*/

BaseSequentialStream* chp = (BaseSequentialStream*) &SD1; /**< Pointer for
                                                                the serial
                                                                stream to
                                                                output the
                                                                data on the
                                                                USB connector
                                                                of the arduino
                                                                board.      */

ROBOTDriver iprobot;

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
  bool      spf = FALSE; /* Song Played Flag. */

  chRegSetThreadName("Blinker");

  while (TRUE) {
    time += MS2ST(100);
    if (init_time <= 200) {
      init_time++;
    }
    else {
      if (spf == FALSE) {
        buzzerSound();
        buzzerStopSound();
        spf = TRUE;
      }
      palTogglePad(IOPORT2, PORTB_LED1);
    }

    chThdSleepUntil(time);
  }
}

/*
 * @brief   ip-robot asservissement thread.
 */
static THD_WORKING_AREA(waAsser, 128);
static THD_FUNCTION(asserThd, arg) {

  (void)arg;
  systime_t time = chVTGetSystemTimeX();

  chRegSetThreadName("Asservissement");

  while (TRUE) {
    time += MS2ST(10);
    asserv(&iprobot);
    chThdSleepUntil(time);
  }
}

/*
 * Application entry point.
 */
int main(void) {

  static msg_t msg;

  /*
   * System initialisations.
   * - HAL initialisation, this also initialises the configured device drivers
   *   and performs the board-specific initialisations.
   * - Kernel initialisation, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /* Start the serial. */
  sdStart(&SD1, NULL);

  buzzerInit();

  buzzerSound();
  buzzerStopSound();

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r");
  chprintf_g(chp, "\n\r   IP-Robot logo.");
  chprintf_c(chp, "\n\r    _                    _           _");
  chprintf_c(chp, "\n\r   (_)                  | |         | |");
  chprintf_c(chp, "\n\r    _ _ __     _ __ ___ | |__   ___ | |_");
  chprintf_c(chp, "\n\r   | | '_ \\   | '__/ _ \\| '_ \\ / _ \\| __|");
  chprintf_c(chp, "\n\r   | | |_) |  | | | (_) | |_) | (_) | |_");
  chprintf_c(chp, "\n\r   |_| .__/   |_|  \\___/|_.__/ \\___/ \\__|");
  chprintf_c(chp, "\n\r     | |");
  chprintf_c(chp, "\n\r     |_|");

  chprintf(chp, "\n\r");
  chprintf_g(chp, "\n\r   General informations.");
  chprintf(chp, "\n\r ip-robot is Inverted Pendulum Robot.");
  chprintf(chp, "\n\r Made by Theodore Ateba (tfateba), tf.ateba@gmail.com");
  chprintf(chp, "\n\r The robot is controlled with ChibiOS/RT v16.1.5");
  chprintf(chp, "\n\r The target board is an Arduino Mega.");
  chprintf(chp, "\n\r The micro controller is an ATMEGA2560.");
  chprintf(chp, "\n\r Copyright: 2015...2018");

  chprintf(chp, "\n\r");
  chprintf_g(chp, "\n\r   IP-Robot initialisation.");
  chprintf(chp, "\n\r%s: Initialisation of Inverted pendulum Robot started:",
  __func__);
  chprintf(chp, "\n\r%s: Serial driver initialisation done.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Turn off the debug LED. */
  palClearPad(IOPORT2, PORTB_LED1);

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r%s: On-board LED initialisation done.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Start I2C interface. */
  i2cStart(&I2CD1, &i2cConfig);

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r%s: I2C bus interface initialisation done.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Init Kalman filter. */
  kalmanInit();

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r%s: Kalman filter initialisation done.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Init MPU module. */
   msg = mpu6050Init(&I2CD1, &iprobot.imu, MPU6050_ADDR);

  if (msg != MSG_OK) {
#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
    chprintf_r(chp, "\n\r Error while initialising the IMU.");
#endif
    return -1;
  }

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r%s: IMU sensor initialisation started, please wait...",
  __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Start MPU calibration process. */
  mpu6050Calibration(&I2CD1, &iprobot.imu);

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r%s: IMU sensor calibration done.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Configure the left pid for the robot. */
  iprobot.lpid.kp = 55.468;
  iprobot.lpid.ki = 0.554;
  iprobot.lpid.kd = 42.524;

  /* Configure the right pid for the robot. */
  iprobot.rpid.kp = 55.468;
  iprobot.rpid.ki = 0.554;
  iprobot.rpid.kd = 42.524;

  /* Init PID controller. */
  pidInit(iprobot.rpid.kp, iprobot.rpid.ki, iprobot.rpid.kd);

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r%s: PID initialisation done.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Init Motors. */
  motorInit();

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r%s: Motors initialisation done.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Init Encoders. */
  encoderInit();

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r%s: Encoder initialisation done.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Init PWM modules. */
  pwmInits();

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r%s: PWM initialisation done.", __func__);
  chThdSleepMilliseconds(10);
#endif

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r%s: Buzzer initialisation done.",
  __func__);
  chThdSleepMilliseconds(10);
#endif

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r%s: Inverted Pendulum Robot initialisation done.",
  __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Create and starts the LED blinker thread. */
  chThdCreateStatic(waBlink, sizeof(waBlink), NORMALPRIO + 4, blinkThd, NULL);

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r");
  chprintf_g(chp, "\n\r   Threads creation.");
  chprintf(chp, "\n\r%s: Create Blink thread.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Create and starts asservissement thread. */
  chThdCreateStatic(waAsser, sizeof(waAsser), NORMALPRIO + 8, asserThd, NULL);

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r%s: Create Asservissement thread.", __func__);
  chThdSleepMilliseconds(10);
  chprintf(chp, "\n\r");
  chprintf_g(chp, "\n\r   Application is started.");
#endif

  while (TRUE) {
    chThdSleepMilliseconds(100);
  }
}

