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

static MOTORConfig motorLConfig = {
  MOTOR_L,                /**< Motor ID.            */
  MOTOR_MAX_SPEED,        /**< Motor max speed.     */
  L_MOTOR_FORWARD_PORT,   /**< Motor Forward  port. */
  L_MOTOR_BACKWARD_PORT,  /**< Motor Backward port. */
  L_MOTOR_ENABLE_PORT,    /**< Motor Enable   port. */
  L_MOTOR_FORWARD_PIN,    /**< Motor Forwxard pin.  */
  L_MOTOR_BACKWARD_PIN,   /**< Motor Backward pin.  */
  L_MOTOR_ENABLE_PIN,     /**< Motor enable   pin.  */
};

static MOTORConfig motorRConfig = {
  MOTOR_R,                /**< Motor ID.            */
  MOTOR_MAX_SPEED,        /**< Motor max speed.     */
  R_MOTOR_FORWARD_PORT,   /**< Motor Forward  port. */
  R_MOTOR_BACKWARD_PORT,  /**< Motor Backward port. */
  R_MOTOR_ENABLE_PORT,    /**< Motor Enable   port. */
  R_MOTOR_FORWARD_PIN,    /**< Motor Forwxard pin.  */
  R_MOTOR_BACKWARD_PIN,   /**< Motor Backward pin.  */
  R_MOTOR_ENABLE_PIN,     /**< Motor enable   pin.  */
};

/* Left encoder Config. */
static ENCODERConfig encoderLConfig = {
  ENCODER_L,         /**< Encoderr identifier.         */
  L_ENCODER_EXT_INT, /**< External interrupt channel.  */
  L_ENCODER_PORT_A,  /**< Port A.                      */
  L_ENCODER_PORT_B,  /**< Port B.                      */
  L_ENCODER_PIN_A,   /**< Pin A.                       */
  L_ENCODER_PIN_B    /**< Pin B.                       */
};

/* Rigth encoder Config. */
static ENCODERConfig encoderRConfig = {
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

  /* Init PWM modules. */
  pwmInits();

#if (DEBUG_MAIN)
  pr_debug("\n\rPWM initialization done");
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
