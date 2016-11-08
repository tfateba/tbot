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
#include "ip_adc.h"
#include "ip_asserv.h"
#include "ip_i2c.h"
#include "ip_kalman.h"
#include "ip_motor.h"
#include "ip_mpu6050.h"
#include "ip_pid.h"
#include "ip_pwm.h"

/*=========================================================================*/
/* Application macros.                                                     */
/*=========================================================================*/
#define mpu "MPU6050"

/*=========================================================================*/
/* Global variables, I2C TX and RX buffers, I2C and Serial Configurations  */
/*=========================================================================*/
BaseSequentialStream* chp = (BaseSequentialStream*) &SD1; /*               */
mpu6050_t       imu;        /**< MPU6050 instance.                         */
msg_t           msg;        /**< Message error.                            */
const uint8_t   delta = 10; /*                                             */

bool    layingDown = true;
double  targetAngle = 178; /**< The angle we want the robot to reach.      */
double  targetOffset = 0;  /**< Offset for going forward and backwrd.      */
double  turningOffset = 0; /**< Offset for turning left and right.         */

/*=========================================================================*/
/* Local functions.                                                        */
/*=========================================================================*/

void asservisment(void) {
  msg = mpu6050_getData(&I2CD1, &imu);

  imu.pitch = (atan2(imu.y_accel, imu.z_accel) + 3.14)*(180/3.14);
  imu.pitch_k = kalman_getAngle(imu.pitch, (imu.x_gyro / 131.0), delta);

  chprintf(chp, " pitch_kalman is %.3f °c\r\n", imu.pitch_k);

  if ((layingDown && (imu.pitch_k < 170 || imu.pitch_k > 190)) ||
    (!layingDown && (imu.pitch_k < 135 || imu.pitch_k > 225))) {
      /*
       * The robot is in a unsolvable position, so turn off both motors and
       * wait until it's vertical again.
       */
      layingDown = true;
      motorsStopAndReset();
    } else {
      /*
       * It's no longer laying down,
       * so we can try to stabilized the robot now.
       */
      layingDown = false;
      pid(imu.pitch_k, targetAngle, targetOffset, turningOffset);
    }

    /* Update wheel velocity every 100ms. */
    motorGetWheelVelocity();
}

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
    // TODO: rmove the demo code.
    //pwmDemo();
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
		asservisment();
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

  /* Turn off the debug LED. */
  palClearPad(IOPORT2, PORTB_LED1);

  /* Start the serial. */
  sdStart(&SD1, NULL);

  /* Start I2C interface. */
  i2cStart(&I2CD1, &i2cConfig);

  /* Init Kalman filter. */
	kalman_init();

  /* Init MPU module. */
	chprintf(chp, "\n\r MPU: Configurations started...");
  msg = mpu6050_init(&I2CD1, &imu, MPU6050_ADDR);

  if (msg != MSG_OK) {
  	chprintf(chp, "\n\r %s: Error while initialising the %s sensor.", mpu, mpu);
		return -1;
	}

  chprintf(chp, "\n\r MPU: Initialisation Ok.");
  chprintf(chp, "\n\r MPU: Calibration, please wait while processing...");

  /* Start MPU calibration process. */
  mpu6050_calibration(&I2CD1, &imu);
  chprintf(chp, "\n\r MPU: Calibration ended.", msg);

  /* Init Motors. */
  motorInit();

  /* Init PWM modules. */
  pwm_init();

  //chThdSleepMilliseconds(1000);

  /* Create and starts the LED blinker thread. */
  chThdCreateStatic(waBlink, sizeof(waBlink), NORMALPRIO, blinkThd, NULL);

  /* Create and starts asservissement thread. */
  chThdCreateStatic(waAsser, sizeof(waAsser), NORMALPRIO+1, asserThd, NULL);

  while (TRUE) {
    chThdSleepMilliseconds(1000);
  }
}

