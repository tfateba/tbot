
/**
 * 
 * @file    ip_encoder.c
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
#include "ip_encoder.h"
#include "ip_main.h"

/*==========================================================================*/
/* Global variables.                                                        */
/*==========================================================================*/

/* Project local variables. */
long wheelPosition;     /**< Position of the robot wheels.                  */
long wheelVelocity;     /**< Velocity of the robot wheels.                  */
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
 * @brief   Right motor encoder external interrupt callback.
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
 * @brief   Get the state of the left encoder output A.
 *
 * @return  state   the value of the left encoder output A
 */
bool encoder_left_read_state_a(void) {

  bool state;

  if (iprobot.lencoder.statea)
    state = 1;
  else
    state = 0;

  return state;
}

/**
 * @brief   Get the state of the left encoder output B.
 *
 * @return  state   the value of the left encoder output B
 */
bool encoder_left_read_state_b(void) {

  bool state;

  if (iprobot.lencoder.stateb)
    state = 1;
  else
    state = 0;

  return state;
}

/**
 * @brief   Get the state of the right encoder output A.
 *
 * @return  state  the value of the right encoder output A
 */
bool encoder_right_read_state_a(void) {

  bool state;

  if (iprobot.rencoder.statea)
    state = 1;
  else
    state = 0;

  return state;
}

/**
 * @brief   Get the state of the right encoder output B.
 *
 * @return  state  the value of the right encoder output B
 */
bool encoder_right_read_state_b(void) {

  bool state;

  if (iprobot.rencoder.stateb)
    state = 1;
  else
    state = 0;

  return state;
}

/**
 * @brief   Initialise all pins needs for encoder.
 */
void encoder_init(void) {

  /* Initialise left encoder. */
  iprobot.lencoder.id       = ENCODER_L;
  iprobot.lencoder.eichan   = INT3;
  iprobot.lencoder.porta    = IOPORT4;
  iprobot.lencoder.pina     = PD3;
  iprobot.lencoder.portb    = IOPORT7;
  iprobot.lencoder.pinb     = PG5;
  iprobot.lencoder.counter  = 0;
  iprobot.lencoder.statea   = false;
  iprobot.lencoder.stateb   = false;

  /* Initialise right encoder.  */
  iprobot.rencoder.id       = ENCODER_R;
  iprobot.rencoder.eichan   = INT2;
  iprobot.rencoder.porta    = IOPORT4;
  iprobot.rencoder.pina     = PD2;
  iprobot.rencoder.portb    = IOPORT5;
  iprobot.rencoder.pinb     = PE3;
  iprobot.rencoder.counter  = 0;
  iprobot.rencoder.statea   = false;
  iprobot.rencoder.stateb   = false;

  /* Set left motor encoders ports and pins. */
  palSetPadMode(L_ENCODER_A_PORT, L_ENCODER_A, PAL_MODE_INPUT);
  palSetPadMode(L_ENCODER_B_PORT, L_ENCODER_B, PAL_MODE_INPUT);

  /* Set right motor encoders ports and pins. */
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
void encoder_get_wheel_velocity(void) {

  static  bool    stopped       = TRUE; /* Breaking target position.  */
  static  uint8_t loopCounter   = 0;    /* Update wheel velocity.     */

  loopCounter++;

  if (loopCounter == 10) {
    loopCounter = 0;
    wheelPosition = iprobot.lencoder.counter + iprobot.rencoder.counter;
    wheelVelocity = wheelPosition - lastWheelPosition;
    lastWheelPosition = wheelPosition;

    if (abs(wheelVelocity) <= 20 && !stopped) {
      /* Set new targetPosition if braking. */
      targetPosition = wheelPosition;
      stopped = TRUE;
    }
  }
}

