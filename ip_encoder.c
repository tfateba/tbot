
/**
 * 
 * @file    ip_encoder.c
 *
 * @brief   Encoder source file.
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

/* ChibiOS libraries. */
#include "hal.h"
#include "chprintf.h"

/* Project local files. */
#include "ip_encoder.h"

/*==========================================================================*/
/* Global variables.                                                        */
/*==========================================================================*/

/* Project local variables. */
long wheelPosition;     /**< Position of the robot wheels.                  */
long wheelVelocity;     /**< Velocity fo the robot wheels.                  */
long lastWheelPosition; /**< Backup of the robot wheel position.            */
long targetPosition;    /**< The robot target angle  position.              */

static  bool      stopped;                /**< Breaking target position.    */
static  long      leftCounter   = 0;      /**< Left encoder counter.        */
static  long      rightCounter  = 0;      /**< Rigth encoder counter.       */
static  bool      lstateA       = false;  /**< Left motor encoder A.        */
static  bool      lstateB       = false;  /**< Left motor encoder B.        */
static  bool      rstateA       = false;  /**< Rigth motor encoder A.       */
static  bool      rstateB       = false;  /**< Rigth motor encoder B.       */
static  uint8_t   loopCounter   = 0;      /**< Update wheel velocity.       */

/*==========================================================================*/
/* Encoders callback.                                                       */
/*==========================================================================*/

/**
 * @brief   This is the External interrupt callback for the Left motor
 *          encoder.
 */
static void encoderLeftCallback(EXTDriver *extp, expchannel_t channel) {

  (void)extp;
  (void)channel;

  chSysLockFromISR();

  if (palReadPad(L_ENCODER_B_PORT, L_ENCODER_B))
    leftCounter--;
  else
    leftCounter++;

  lstateA = palReadPad(L_ENCODER_A_PORT, L_ENCODER_A);
  lstateB = palReadPad(L_ENCODER_B_PORT, L_ENCODER_B);

  chSysUnlockFromISR();
}

/**
 * @brief   This is the External interrupt callback for the Rigth motor
 *          encoder.
 */
static void encoderRightCallback(EXTDriver *extp, expchannel_t channel) {

  (void)extp;
  (void)channel;

  chSysLockFromISR();

  if (palReadPad(R_ENCODER_A_PORT, R_ENCODER_A))
    rightCounter--;
  else
    rightCounter++;

  rstateA = palReadPad(R_ENCODER_A_PORT, R_ENCODER_A);
  rstateB = palReadPad(R_ENCODER_B_PORT, R_ENCODER_B);

  chSysUnlockFromISR();
}

/**
 * @brief EXT Driver configurations.
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
/* Functions.                                                               */
/*==========================================================================*/

/**
 * @brief   The encoders decrease when motor is traveling forward and increase
 *          when traveling backward.
 *
 * @return  leftCounter   the value of the left encoder
 */
long encoderReadLeftCounter(void) {

  return leftCounter;
}

/**
 * @brief   The encoders decrease when motor is traveling forward and increase
 *          when traveling backward.
 *
 * @return  rightCounter  the value of the right encoder
 */
long encoderReadRightCounter(void) {

  return rightCounter;
}

/**
 * @brief   Get the state of the left encoder A.
 *
 * @return  ret the value of the left encoder A
 */
long encoderReadLeftStateA(void) {

  long ret;

  if (lstateA)
    ret = 1;
  else
    ret = 0;

  return ret;
}

/**
 * @brief   Get the state of the left encoder B.
 *
 * @return  ret the value of the left encoder B
 */
long encoderReadLeftEncoderStateB(void) {

  long ret;

  if (lstateB)
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
long encoderReadRightStateA(void) {

  long ret;

  if (rstateA)
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
long encoderReadRightStateB(void) {

  long ret;

  if (rstateB)
    ret = 1;
  else
    ret = 0;

  return ret;
}

/**
 * @brief   Initialize all pins needs for motor control
 */
void encoderInit(void) {

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

  loopCounter++;

  if (loopCounter == 10) {
    loopCounter = 0;
    wheelPosition = encoderReadLeftCounter() + encoderReadRightCounter();
    wheelVelocity = wheelPosition - lastWheelPosition;
    lastWheelPosition = wheelPosition;

    if (abs(wheelVelocity) <= 20 && !stopped) {
      /* Set new targetPosition if braking. */
      targetPosition = wheelPosition;
      stopped = true;
    }
  }
}

