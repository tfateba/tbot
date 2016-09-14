/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

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

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

BaseSequentialStream* chp = (BaseSequentialStream*) &SD1;

static WORKING_AREA(waThread1, 32);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("Blinker");
  while (true) {
    palTogglePad(IOPORT2, PORTB_LED1);
    chThdSleepMilliseconds(1000);
		chprintf(chp, "\n\rHello world from Chibios RT on Arduino Mega2560\n");
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

	palSetPadMode(IOPORT2, 2, PAL_MODE_OUTPUT_PUSHPULL);
  palClearPad(IOPORT2, PORTB_LED1);
	palClearPad(IOPORT2, 2);

  /*
   * Activates the serial driver 1 using the driver default configuration.
   */
  sdStart(&SD1, NULL);

	chprintf(chp, "\n\r MPU: Configurations started...");
  chThdSleepMilliseconds(1000);
  /*
   * Starts the LED blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  while(TRUE) {
    //palTogglePad(IOPORT2, 2);
    //chThdSleepMilliseconds(1000);
		//chprintf(chp, "\n\rHello world from Chibios RT on Arduino Mega2560\n");
  }
}
