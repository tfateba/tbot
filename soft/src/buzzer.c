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
 * @file    buzzer.c
 * @brief   Buzzer driver source file.
 *
 * @addtogroup BUZZER
 * @{
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

  /* Set up timer with prescaler = 64 and CTC mode  */
  TCCR5B |= (1 << WGM52)|(1 << CS51)|(1 << CS50);
  /* Set up timer OC1A pin in toggle mode.          */
  TCCR5A |= (1 << COM5A0);
  /* Initialize counter.                            */
  TCNT5 = 0;
  /* Initialize compare value.                      */
  OCR5A = 0;
  /* Enable compare interrupt.                      */
  TIMSK5 |= (1 << OCIE5A);
  /* Enable global interrupts.                      */
  /* @sei();                                        */

  /* Initialize buzzer Pin.                         */
  /* Connect buzzer to pin PL3.                     */
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

/**
 * @brief   Timer 5 ISR handler.
 */
ISR (TIMER5_COMPA_vect) {

}

/** @} */
