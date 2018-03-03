
/**
 * 
 * @file    ipencoder.c
 *
 * @brief   Encoder driver source file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    27 August 2017
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
#include "chprintf.h"

/* Project files. */
#include "ipencoder.h"
#include "ipmain.h"

/*==========================================================================*/
/* Global variables.                                                        */
/*==========================================================================*/

/* Project local variables. */
long wheelPosition;     /**< Position of the robot wheels.                  */
long wheelVelocity;     /**< Velocity fo the robot wheels.                  */
long lastWheelPosition; /**< Backup of the robot wheel position.            */
long targetPosition;    /**< The robot target angle  position.              */

extern ROBOTDriver iprobot;
/*==========================================================================*/
/* Encoders callback.                                                       */
/*==========================================================================*/

/**
 * @brief   Left motor encoder external interrupt callback.
 */
static void encoderLeftCallback(EXTDriver *extp, expchannel_t channel) {

  (void)extp;
  (void)channel;

  chSysLockFromISR();

  if (palReadPad(L_ENCODER_B_PORT, L_ENCODER_B))
    iprobot.lencoder.counter--;
  else
    iprobot.lencoder.counter++;

  iprobot.lencoder.statea = palReadPad(L_ENCODER_A_PORT, L_ENCODER_A);
  iprobot.lencoder.stateb = palReadPad(L_ENCODER_B_PORT, L_ENCODER_B);

  chSysUnlockFromISR();
}

/**
 * @brief   Rigth motor encoder external interrupt callback.
 */
static void encoderRightCallback(EXTDriver *extp, expchannel_t channel) {

  (void)extp;
  (void)channel;

  chSysLockFromISR();

  if (palReadPad(R_ENCODER_A_PORT, R_ENCODER_A))
    iprobot.rencoder.counter--;
  else
    iprobot.rencoder.counter++;

  iprobot.rencoder.statea = palReadPad(R_ENCODER_A_PORT, R_ENCODER_A);
  iprobot.rencoder.stateb = palReadPad(R_ENCODER_B_PORT, R_ENCODER_B);

  chSysUnlockFromISR();
}

/**
 * @brief   EXT Driver configurations.
 */
static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_DISABLED, NULL},                     /***< INT0 Config. */
    {EXT_CH_MODE_DISABLED, NULL},                     /***< INT1 Config. */
    {EXT_CH_MODE_RISING_EDGE, encoderRightCallback},  /***< INT2 Config. */
    {EXT_CH_MODE_RISING_EDGE, encoderLeftCallback},   /***< INT3 Config. */
    {EXT_CH_MODE_DISABLED, NULL},                     /***< INT4 Config. */
    {EXT_CH_MODE_DISABLED, NULL},                     /***< INT5 Config. */
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
bool encoderReadLeftStateA(void) {

  bool ret;

  if (iprobot.lencoder.statea)
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
bool encoderReadLeftEncoderStateB(void) {

  bool ret;

  if (iprobot.lencoder.stateb)
    ret = 1;
  else
    ret = 0;

  return ret;
}

/**
 * @brief   Get the state of the right encoder A.
 *
 * @return  ret  the value of the right encoder A
 */
bool encoderReadRightStateA(void) {

  bool ret;

  if (iprobot.rencoder.statea)
    ret = 1;
  else
    ret = 0;

  return ret;
}

/**
 * @brief   Get the state of the right encoder B.
 *
 * @return  ret  the value of the right encoder B
 */
bool encoderReadRightStateB(void) {

  bool ret;

  if (iprobot.rencoder.stateb)
    ret = 1;
  else
    ret = 0;

  return ret;
}

/**
 * @brief   Initialize all pins needs for motor control.
 */
void encoderInit(void) {

  /* Initialise left encoder . */
  iprobot.lencoder.id       = ENCODER_L;
  iprobot.lencoder.eichan   = INT3;
  iprobot.lencoder.porta    = IOPORT4;
  iprobot.lencoder.pina     = PD3;
  iprobot.lencoder.portb    = IOPORT7;
  iprobot.lencoder.pinb     = PG5;
  iprobot.lencoder.counter  = 0;
  iprobot.lencoder.statea   = false;
  iprobot.lencoder.stateb   = false;

  /* Initialise rigth encoder . */
  iprobot.rencoder.id       = ENCODER_R;
  iprobot.rencoder.eichan   = INT2;
  iprobot.rencoder.porta    = IOPORT4;
  iprobot.rencoder.pina     = PD2;
  iprobot.rencoder.portb    = IOPORT5;
  iprobot.rencoder.pinb     = PE3;
  iprobot.rencoder.counter  = 0;
  iprobot.rencoder.statea   = false;
  iprobot.rencoder.stateb   = false;

  /* Set left Motors Encoders. */
  palSetPadMode(L_ENCODER_A_PORT, L_ENCODER_A, PAL_MODE_INPUT);
  palSetPadMode(L_ENCODER_B_PORT, L_ENCODER_B, PAL_MODE_INPUT);

  /* Set Rigth motor encoders. */
  palSetPadMode(R_ENCODER_A_PORT, R_ENCODER_A, PAL_MODE_INPUT);
  palSetPadMode(R_ENCODER_B_PORT, R_ENCODER_B, PAL_MODE_INPUT);

  /* Start the EXT Driver with our configuration. */
  extStart(&EXTD1, &extcfg);

  /* Enable External interruptions for encoders. */
  extChannelEnable(&EXTD1, INT2);
  extChannelEnable(&EXTD1, INT3);
}

/**
 * @brief   Get the wheel velocity for asservissement routine.
 */
void encoderGetWheelVelocity(void) {

  static  bool    stopped       = true; /* Breaking target position.  */
  static  uint8_t loopCounter   = 0;    /**< Update wheel velocity.   */

  loopCounter++;

  if (loopCounter == 10) {
    loopCounter = 0;
    wheelPosition = iprobot.lencoder.counter + iprobot.rencoder.counter;
    wheelVelocity = wheelPosition - lastWheelPosition;
    lastWheelPosition = wheelPosition;

    if (abs(wheelVelocity) <= 20 && !stopped) {
      /* Set new targetPosition if braking. */
      targetPosition = wheelPosition;
      stopped = true;
    }
  }
}

