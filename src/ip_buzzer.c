
/**
 *
 * @file    ip_buzzer.c
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
 * @brief   Initialisation buffer.
 */
void buzzer_init(void)  {

  // set up timer with pre-scaler = 64 and CTC mode
  TCCR5B |= (1 << WGM52)|(1 << CS51)|(1 << CS50);
  // set up timer OC1A pin in toggle mode
  TCCR5A |= (1 << COM5A0);
  // initialise counter
  TCNT5 = 0;
  // initialise compare value
  OCR5A = 0;
  // enable compare interrupt
  TIMSK5 |= (1 << OCIE5A);
  // enable global interrupts
  //sei();

  // Initialise buzzer Pin
  // connect buzzer to pin PL3
  DDRL |= (1 << 3);
}

/**
 * @brief   Stop all playing sound.
 */
void buzzer_sound_stop(void) {

  OCR5A = 0;
}

/**
 * @brief   Play sound with buzzer.
 */
void buzzer_sound_play(void) {

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

