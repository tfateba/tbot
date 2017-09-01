
/**
 *
 * @file    ip_buzzer.c
 *
 * @brief   Buzzer source file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    22 August 2017
 *
 */

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

/* Standard files. */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/* ChibiOS files. */
#include "hal.h"

/**
 * @brief   Buzzer initialization function.
 */
void buzzerInit(void)  {

  // set up timer with prescaler = 64 and CTC mode
  TCCR5B |= (1 << WGM52)|(1 << CS51)|(1 << CS50);
  // set up timer OC1A pin in toggle mode
  TCCR5A |= (1 << COM5A0);
  // initialize counter
  TCNT5 = 0;
  // initialize compare value
  OCR5A = 29;
  // enable compare interrupt
  TIMSK1 |= (1 << OCIE5A);
  // enable global interrupts
  sei();
}

/**
 * @brief   Buzzer function to stop all playing sound.
 */
void buzzerStopSound(void) {
  OCR5A = 0;
}

/**
 * @brief   Buzzer function to play the first a sound.
 */
void buzzerSound1(void) {

  chThdSleepMilliseconds(100);
  OCR5A += 1;
  if ( OCR5A >= 200) {
    OCR5A = 15;
  }
}

/**
 * @brief   Buzzer function to play the second sound.
 */
void buzzerSound2(void) {

  OCR5A = 29;
  chThdSleepMilliseconds(200);
  OCR5A = 15;
  chThdSleepMilliseconds(200);
  OCR5A = 29;
  chThdSleepMilliseconds(200);
  OCR5A = 0;
  chThdSleepMilliseconds(3000);
}

