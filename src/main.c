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

ROBOTDriver tbot;

static MOTORConfig motorLeftConfig = {
  MOTOR_L,                /**< Motor ID.            */
  /* TODO: Make a define for this parameter. */
  255,                    /**< Motor max speed.     */
  L_MOTOR_FORWARD_PORT,   /**< Motor Forward  port. */
  L_MOTOR_BACKWARD_PORT,  /**< Motor Backward port. */
  L_MOTOR_ENABLE_PORT,    /**< Motor Enable   port. */
  L_MOTOR_FORWARD_PIN,    /**< Motor Forwxard pin.  */
  L_MOTOR_BACKWARD_PIN,   /**< Motor Backward pin.  */
  L_MOTOR_ENABLE_PIN,     /**< Motor enable   pin.  */
};

static MOTORConfig motorRightConfig = {
  MOTOR_R,                /**< Motor ID.            */
  255,                    /**< Motor max speed.     */
  R_MOTOR_FORWARD_PORT,   /**< Motor Forward  port. */
  R_MOTOR_BACKWARD_PORT,  /**< Motor Backward port. */
  R_MOTOR_ENABLE_PORT,    /**< Motor Enable   port. */
  R_MOTOR_FORWARD_PIN,    /**< Motor Forwxard pin.  */
  R_MOTOR_BACKWARD_PIN,   /**< Motor Backward pin.  */
  R_MOTOR_ENABLE_PIN,     /**< Motor enable   pin.  */
};

/* Left encoder Config. */
static ENCODERConfig encoderLeftConfig = {
  ENCODER_L,         /**< Encoderr identifier.         */
  L_ENCODER_EXT_INT, /**< External interrupt channel.  */
  L_ENCODER_PORT_A,  /**< Port A.                      */
  L_ENCODER_PORT_B,  /**< Port B.                      */
  L_ENCODER_PIN_A,   /**< Pin A.                       */
  L_ENCODER_PIN_B    /**< Pin B.                       */
};

/* Rigth encoder Config. */
static ENCODERConfig encoderRightConfig = {
  ENCODER_R,         /**< Encoderr identifier.         */
  R_ENCODER_EXT_INT, /**< External interrupt channel.  */
  R_ENCODER_PORT_A,  /**< Port A.                      */
  R_ENCODER_PORT_B,  /**< Port B.                      */
  R_ENCODER_PIN_A,   /**< Pin A.                       */
  R_ENCODER_PIN_B    /**< Pin B.                       */
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

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  /* TODO: Make a function, so that the code will be more clear. */
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
  chprintf(chp, "\n\r The microcontroller is an ATMEGA2560.");
  chprintf(chp, "\n\r Copyrigth: 2015...2017");

  chprintf(chp, "\n\r");
  chprintf_g(chp, "\n\r   IP-Robot initialization.");
  chprintf(chp, "\n\r%s: Initialization of Inverted pendulum Robot started:",
  __func__);
  chprintf(chp, "\n\r%s: Serial driver initialization done.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Turn off the debug LED. */
  palClearPad(IOPORT2, PORTB_LED1);

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r%s: On-board LED initialization done.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Start I2C interface. */
  i2cStart(&I2CD1, &i2cConfig);

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r%s: I2C bus interface initialization done.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Init Kalman filter. */
  kalmanInit();

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r%s: Kalman filter initialization done.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Init MPU module. */
   msg = mpu6050Init(&I2CD1, &tbot.imu, MPU6050_ADDR);

  if (msg != MSG_OK) {
#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
    chprintf_r(chp, "\n\r Error while initialising the IMU.");
#endif
    return -1;
  }

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r%s: IMU sensor initialization started, please wait...",
  __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Start MPU calibration process. */
  mpu6050Calibration(&I2CD1, &tbot.imu);

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r%s: IMU sensor calibration done.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Init PID controller. */
  pidInit(&tbot.pidSpeed, 0, 0, 0); /* PI */
  pidInit(&tbot.pidAngle, 55.468, 0.554, 42.524); /* PID  */

  /* OK, but the PID still need to be tuned.  */
  /* TODO: motorLeftPid ==> must be rename pidLeftMotor.   */
  pidInit(&tbot.pidMotorLeft,  1, 0, 0);
  /* TODO: motorRightPid ==> must be rename pidRightMotor.  */
  pidInit(&tbot.pidMotorRight, 1, 0, 0);

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r%s: PID initialization done.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Init Motors. */
  motorInit(&tbot.motorLeft, motorLeftConfig);
  motorInit(&tbot.motorRight, motorRightConfig);

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r%s: Motors initialization done.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Init Encoders. */
  encoderInit(&tbot.encoderRight, encoderRightConfig);
  encoderInit(&tbot.encoderLeft, encoderLeftConfig);

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r%s: Encoder initialization done.", __func__);
  chThdSleepMilliseconds(10);
#endif

  /* Init PWM modules. */
  pwmInits();

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r%s: PWM initialization done.", __func__);
  chThdSleepMilliseconds(10);
#endif

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r%s: Buzzer initialization done.",
  __func__);
  chThdSleepMilliseconds(10);
#endif

#if (DEBUG == TRUE || DEBUG_MAI == TRUE)
  chprintf(chp, "\n\r%s: Inverted Pendulum Robot initialization done.",
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

/** @} */
