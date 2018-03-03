
/**
 *
 * @file    ipbuzzer.c
 *
 * @brief   Buzzer driver source file.
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
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/*==========================================================================*/
/* Driver functions.                                                        */
/*==========================================================================*/

/**
 * @brief   Initialization buffer.
 */
void buzzerInit(void)  {

  // set up timer with prescaler = 64 and CTC mode
  TCCR5B |= (1 << WGM52)|(1 << CS51)|(1 << CS50);
  // set up timer OC1A pin in toggle mode
  TCCR5A |= (1 << COM5A0);
  // initialize counter
  TCNT5 = 0;
  // initialize compare value
  OCR5A = 0;
  // enable compare interrupt
  TIMSK5 |= (1 << OCIE5A);
  // enable global interrupts
  //sei();

  // Initialize buzzer Pin
  // connect buzzer to pin PL3
  DDRL |= (1 << 3);
}

/**
 * @brief   Stop all playing sound.
 */
void buzzerStopSound(void) {

  OCR5A = 0;
}

/**
 * @brief   Play soung with buzzer.
 */
void buzzerSound(void) {

  OCR5A = 29;
  _delay_ms(100);
  OCR5A = 100;
  _delay_ms(100);
  OCR5A = 29;
  _delay_ms(100);
  OCR5A = 100;
  _delay_ms(100);
}

ISR (TIMER5_COMPA_vect) {

}

