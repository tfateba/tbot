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

#ifdef __cplusplus
extern "C" {
#endif

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

/* Standard files. */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "hal.h"
#include "buzzer.hpp"
#include "conf.h"


/* Extern variables. */
#if (DEBUG_BUZZER)
extern BaseSequentialStream* chp; /* Pointer used for chpirntf. */
//#define pr_debug(x) chprintf(chp, x)
#endif

/*==========================================================================*/
/* Driver functions.                                                        */
/*==========================================================================*/


void Buzzer::playPassed(void) {

  int melodyOn[]  = {NOTE_C5, NOTE_C6, NOTE_D5, NOTE_A6};

  for (int index = 0; index < 4; index++) {
    tone(melodyOn[index], 100);
  }
  stopSound();
}

void Buzzer::playFailed(void) {

  //int melody[]  = {NOTE_A4, NOTE_D3}; // Failed
  //int melody[]  = {NOTE_A4, NOTE_D5}; // Police
  //int melody[]  = {NOTE_A4, NOTE_FS5};  // Gendarmerie
  //int melody[]  = {NOTE_A4, NOTE_B4};   // Pompier
  //int melody[]  = {NOTE_A4, NOTE_E5};   // SAMU
  //int melody[]  = {NOTE_E5, NOTE_A4};   // Bell
  //int melody[]  = {NOTE_B0, NOTE_A2};   // Bip
  //int melody[]  = {NOTE_B0, NOTE_C1};   // Bip
  int melody[]  = {NOTE_C4, NOTE_G3};   // Bip
  //int melody[] = {NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4};


  /*
  for (int index = 0; index < 7; index++) {
    tone(melody[index], 200);
  }
  */
  for (int index = 0; index < 2; index++) {
    tone(melody[index], 100);
  }
  for (int index = 0; index < 2; index++) {
    tone(melody[index], 100);
  }
  stopSound();
}

void Buzzer::tone(int note, int duration) {

  OCR5A = note;

  while (duration) {
    _delay_ms(1);
    duration--;
  }
}

/**
 * @brief   Initialization buffer.
 */
void Buzzer::init(void)  {

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
  pr_debug("\n\rBuzzer initialization done.");
}

/**
 * @brief   Stop all playing sound.
 */
void Buzzer::stopSound(void) {

  OCR5A = 0;
  // Also disable the Compare match interrupt!
}

/**
 * @brief   Play soung with buzzer.
 */
void Buzzer::startSound(void) {

#define SIZE 4
  //double notes[SIZE] = {29, 100, 29, 100};
  //double notes[SIZE] = {NOTE_B0, NOTE_A2, NOTE_B0, NOTE_A2};
  double notes[SIZE] = {NOTE_B1, NOTE_A3, NOTE_B1, NOTE_A3};

  for (int8_t index = 0; index < SIZE; index++) {
    tone(notes[index],  100);
  }
  stopSound();
}

void Buzzer::baby(void) {
  tone(NOTE_A4, 100);
  tone(NOTE_A4, 100);
  tone(NOTE_C5, 100);
  tone(NOTE_A4, 100);
  tone(NOTE_A4, 100);
  tone(NOTE_C5, 100);
  tone(NOTE_F5, 100);
  tone(NOTE_E5, 100);
  tone(NOTE_D5, 100);
  tone(NOTE_D5, 100);
  tone(NOTE_C5, 100);
  stopSound();
}

/**
 * @brief   Timer 5 ISR handler.
 */
ISR (TIMER5_COMPA_vect) {

}

#ifdef __cplusplus
}
#endif

/** @} */
