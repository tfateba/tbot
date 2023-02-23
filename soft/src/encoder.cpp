/*
    TBOT - Copyright (C) 2015...2023 Theodore Ateba

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
 * @file    encoder.c
 * @brief   Encoder driver source file.
 *
 * @addtogroup ENCODER
 * @{
 */

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/* Standard files. */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/* ChibiOS files. */
//#include "hal.h"


/* Project files. */
#include "encoder.hpp"
#include "main.h"
#include "hardware.h"
#include "chprintf.h"

/*==========================================================================*/
/* Global variables.                                                        */
/*==========================================================================*/

/* Extern variables. */
#if (DEBUG_ENCODER)
extern BaseSequentialStream* chp; /* Pointer used for chpirntf. */
#endif

/* Project local variables. */
//long wheelPosition;     /**< Position of the robot wheels.                  */
//long wheelVelocity;     /**< Velocity fo the robot wheels.                  */
//long lastWheelPosition; /**< Backup of the robot wheel position.            */
//long targetPosition;    /**< The robot target angle  position.              */

extern Tbot tbot;

/*==========================================================================*/
/* Encoders callback.                                                       */
/*==========================================================================*/

/**
 * @brief   Left motor encoder external interrupt callback.
 */
static void encoderLCallback(EXTDriver *extp, expchannel_t channel) {

  (void)extp;
  (void)channel;

  chSysLockFromISR();

  if (palReadPad(L_ENCODER_PORT_B, L_ENCODER_PIN_B))
    tbot.encoderL.incrementCounter();
  else
    tbot.encoderL.decrementCounter();

  tbot.encoderL.setStateA(palReadPad(L_ENCODER_PORT_A, L_ENCODER_PIN_A));
  tbot.encoderL.setStateB(palReadPad(L_ENCODER_PORT_B, L_ENCODER_PIN_B));

  chSysUnlockFromISR();
}

/**
 * @brief   Rigth motor encoder external interrupt callback.
 */
static void encoderRCallback(EXTDriver *extp, expchannel_t channel) {

  (void)extp;
  (void)channel;

  chSysLockFromISR();

  if (palReadPad(R_ENCODER_PORT_B, R_ENCODER_PIN_B))
    tbot.encoderR.incrementCounter();
  else
    tbot.encoderR.decrementCounter();

  tbot.encoderR.setStateA(palReadPad(R_ENCODER_PORT_A, R_ENCODER_PIN_A));
  tbot.encoderR.setStateB(palReadPad(R_ENCODER_PORT_B, R_ENCODER_PIN_B));

  chSysUnlockFromISR();
}

/**
 * @brief   EXT Driver configurations.
 */
static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_DISABLED, NULL},                     /**< INT0 Config. */
    {EXT_CH_MODE_DISABLED, NULL},                     /**< INT1 Config. */
    {EXT_CH_MODE_RISING_EDGE, encoderRCallback},      /**< INT2 Config. */
    {EXT_CH_MODE_RISING_EDGE, encoderLCallback},      /**< INT3 Config. */
    {EXT_CH_MODE_DISABLED, NULL},                     /**< INT4 Config. */
    {EXT_CH_MODE_DISABLED, NULL},                     /**< INT5 Config. */
  }
};

/*==========================================================================*/
/* Driver Functions.                                                        */
/*==========================================================================*/

/**
 * @brief   Get the state of the left encoder A.
 *
 * @return  ret   the value of the left encoder A
 */
bool Encoder::getStateA(void) {

  bool ret;

  if (stateA)
    ret = 1;
  else
    ret = 0;

  return ret;
}

/**
 * @brief   Get the state of the left encoder B.
 *
 * @return  ret   the value of the left encoder B
 */
bool Encoder::getStateB(void) {

  bool ret;

  if (stateB)
    ret = 1;
  else
    ret = 0;

  return ret;
}


/**
 * @brief   Get the state of the left encoder A.
 *
 * @return  ret   the value of the left encoder A
 *//*
bool Encoder::readLeftStateA(void) {

  bool ret;

  if (tbot.encoderL.statea)
    ret = 1;
  else
    ret = 0;

  return ret;
}
*/

/**
 * @brief   Get the state of the left encoder B.
 *
 * @return  ret   the value of the left encoder B
 *//*
bool Encoder::readLeftStateB(void) {

  bool ret;

  if (tbot.encoderL.stateb)
    ret = 1;
  else
    ret = 0;

  return ret;
}*/

/**
 * @brief   Get the state of the right encoder A.
 *
 * @return  ret  the value of the right encoder A
 *//*
bool Encoder::readRightStateA(void) {

  bool ret;

  if (tbot.encoderR.statea)
    ret = 1;
  else
    ret = 0;

  return ret;
}*/

/**
 * @brief   Get the state of the right encoder B.
 *
 * @return  ret  the value of the right encoder B
 *//*
bool Encoder::readRightStateB(void) {

  bool ret;

  if (tbot.encoderR.stateb)
    ret = 1;
  else
    ret = 0;

  return ret;
}*/

/**
 * @brief   Initialize all pins needs for motor control.
 */
void Encoder::init(ENCODERConfig cfg) {

  /* Initialise encoder . */
  config  = cfg;
  counter = 0;
  stateA  = false;
  stateB  = false;

  /* Set Motors Encoders. */
  palSetPadMode(config.porta, config.pina, PAL_MODE_INPUT);
  palSetPadMode(config.portb, config.pinb, PAL_MODE_INPUT);

  /* Start the EXT Driver with our configuration. */
  extStart(&EXTD1, &extcfg);

  /* Enable External interruptions for encoders. */
  extChannelEnable(&EXTD1, config.eichan);

  #if (DEBUG_ENCODER)
  pr_debug("\n\rEncoder initialized");
  #endif
}

/**
 * @brief   Get the wheel velocity for asservissement routine.
 *//*
void encoderGetWheelVelocity(void) {

  static  bool    stopped       = true; // Breaking target position.
  static  uint8_t loopCounter   = 0;    // Update wheel velocity.

  loopCounter++;

  if (loopCounter == 10) {
    loopCounter = 0;
    wheelPosition = tbot.encoderLeft.counter + tbot.encoderRight.counter;
    wheelVelocity = wheelPosition - lastWheelPosition;
    lastWheelPosition = wheelPosition;

    if (abs(wheelVelocity) <= 20 && !stopped) {
      // Set new targetPosition if braking.
      targetPosition = wheelPosition;
      stopped = true;
    }
  }
}
*/

#ifdef __cplusplus
}
#endif

/** @} */
