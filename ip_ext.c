/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

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

/**
 * @brief   This is the External Interruption 4 callback.
 *          The onboard LED is toggled every time an interruption is 
 *          detected. A varible is also incremented.
 */
static void extcb1(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;

  chSysLockFromISR();
  //Increment encoder counter. //palTogglePad(IOPORT2, PORTB_LED1);
  chSysUnlockFromISR();
}

/**
 * @brief   This is the External Interruption 4 callback.
 *          The onboard LED is toggled every time an interruption is 
 *          detected. A varible is also incremented.
 */
static void extcb2(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;

  chSysLockFromISR();
  //Increment encoder counter. //palTogglePad(IOPORT2, PORTB_LED1);
  chSysUnlockFromISR();
}

/**
 * @brief EXT Driver configurations.
 */
static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_DISABLED , NULL},      /***< INT0 Config */
    {EXT_CH_MODE_DISABLED , NULL},      /***< INT1 Config */
    {EXT_CH_MODE_RISING_EDGE , extcb1},      /***< INT2 Config */
    {EXT_CH_MODE_RISING_EDGE , extcb2},      /***< INT3 Config */
    {EXT_CH_MODE_DISABLED , NULL},  /***< INT4 Config */
    {EXT_CH_MODE_DISABLED , NULL},      /***< INT5 Config */
  }
};
