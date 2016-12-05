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

/*
 * In this demo we use a single channel to sample voltage across
 * a potentiometer.
 */
 #define MY_NUM_CH           1
 #define MY_SAMPLING_NUMBER  10

/**
 * @brief Global variables
 */
BaseSequentialStream * chp;
static int32_t  adcConvA0 = 0;      /**< A0 adc conversion. */
static float    voltageA0 = 0;      /**< A0 voltage.        */
static adcsample_t sample_buff[MY_NUM_CH * MY_SAMPLING_NUMBER];

extern double Kp;   /* Proportional parameter of PID corrector.    */
extern double Kd;   /* Derivative parameter of PID corrector.      */
extern double Ki;   /* Integral parameter of PID corrector.        */

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 10 samples of 1 channel, SW triggered.
 * Channels:    IN0 (Arduino Pin A0).
 */
static const ADCConversionGroup my_conversion_group = {
  FALSE,      /* Not circular buffer.       */
  MY_NUM_CH,  /* Number of channels.        */
  NULL,       /* No ADC callback function.  */
  1,          /* Channel mask.              */
};

static const ADCConfig adcConfig = {
  ANALOG_REFERENCE_AVCC, /* Analog reference. */
};

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
static THD_WORKING_AREA(waAsser, 2048);
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
 * @brief Robot asservissement thread.
 * @TODO: Find the correct size of the working area.
 */
static THD_WORKING_AREA(waAdc, 128);
static THD_FUNCTION(adcThd, arg) {
  (void)arg;
  systime_t time = chVTGetSystemTimeX();
  uint8_t i;

  chRegSetThreadName("Adc");

  /* Activates the ADC1 driver. */
  adcStart(&ADCD1, &adcConfig);

  while (true) {
    time += MS2ST(10);
    /* Make ADC conversion of the voltage on A0. */
    adcConvert(&ADCD1, &my_conversion_group, sample_buff, MY_SAMPLING_NUMBER);

    /* Making mean of sampled values.*/
    for (i = 0; i < MY_NUM_CH * MY_SAMPLING_NUMBER; i++) {
      adcConvA0 += sample_buff[i];
    }

    adcConvA0 /= (MY_NUM_CH * MY_SAMPLING_NUMBER);
    voltageA0 = (((float)adcConvA0 * 5) / 1024);
    //Kp = (voltageA0 * 10);
    //Kd = (voltageA0 * 6);
    //Ki = (voltageA0 / 10);

    Kp = 55.468;
    Kd = 42.524;
    Ki = 0.554;
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
    chprintf(chp, "\n\r Error while initialising the mpu sensor.");
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

  /* Create and starts asservissement thread. */
  chThdCreateStatic(waAdc, sizeof(waAdc), NORMALPRIO + 6, adcThd, NULL);
  #if (DEBUG == TRUE)
  chprintf(chp, "\n\r Create ADC thread.");
  chThdSleepMilliseconds(10);
  #endif

  while (TRUE) {
    //chprintf(chp, "   %.3fv \r\n", voltageA0);
    //chprintf(chp, " Kp = %.3f \r\n", Kp);
    //chprintf(chp, " Kd = %.3f \r\n", Kd);
    //chprintf(chp, " Ki = %.3f \r\n", Ki);
    chThdSleepMilliseconds(100);
  }
}
