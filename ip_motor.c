/**
 *
 * @file          ip_motor.c
 *
 * @brief         motor driver source file.
 *
 * @author        Theodore Ateba, tf.ateba@gmail.com
 *
 * @date          07 Septembre 2015
 *
 * @description   Motor control and Encoder Read.
 *                Get the PWM control value from the pid controler.
 *
 * @note          Motor Power:	white wire
 *                Motor Power:	yellow wire
 *                Encoder GND:	blue wire
 *                Encoder VCC:	green wire
 *                Encoder A:		black wire
 *                Encoder B:		Red wire
 */

/*===========================================================================*/
/* Includes files.                                                           */
/*===========================================================================*/

/* Standard libraries. */
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/* ChibiOS libraries. */
#include "hal.h"
#include "chprintf.h"

/* Project local files. */
#include "ip_pwm.h"
#include "ip_pid.h"
#include "ip_motor.h"

/*==========================================================================*/
/* Application macros.                                                      */
/*==========================================================================*/

#define DEBUG           FALSE   /**< Disable/Enable Debug message.          */
#define PWM_MAX_VALUE   512     /**< Maximum value for a PWM signal.        */

/*==========================================================================*/
/* Global variables.                                                        */
/*==========================================================================*/

long wheelPosition;     /**< Position of the robot wheels.                  */
long wheelVelocity;     /**< Velocity fo the robot wheels.                  */
long lastWheelPosition; /**< Backup of the robot wheel position.            */
long targetPosition;    /**< The robot target angle  position.              */

static bool     stopped;                /**< Target position after breaking.*/
static uint32_t leftCounter   = 0;      /**< Left encoder counter.          */
static uint32_t rightCounter  = 0;      /**< Rigth encoder counter.         */
static bool     lstateA       = false;  /**< Left motor encoder A.          */
static bool     lstateB       = false;  /**< Left motor encoder B.          */
static bool     rstateA       = false;  /**< Rigth motor encoder A.         */
static bool     rstateB       = false;  /**< Rigth motor encoder B.         */
static uint8_t  loopCounter   = 0;      /**< Used to update wheel velocity. */

extern BaseSequentialStream* chp;

/*==========================================================================*/
/* Encoders callback.                                                       */
/*==========================================================================*/

/**
 * @brief   This is the External interrupt callback for the left motor
 *          encoder.
 */
static void motorLeftEncoderCallback(EXTDriver *extp, expchannel_t channel) {

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
 * @brief   This is the External interrupt callback for the rigth motor
 *          encoder.
 */
static void motorRightEncoderCallback(EXTDriver *extp, expchannel_t channel) {

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
    {EXT_CH_MODE_RISING_EDGE, motorRightEncoderCallback}, /***< INT2 Config */
    {EXT_CH_MODE_RISING_EDGE, motorLeftEncoderCallback},  /***< INT3 Config */
  }
};

/*===========================================================================*/
/* Functions.                                                                */
/*===========================================================================*/

/**
 * @brief   Activate the left driver to use left motor.
 */
static void motorsEnableLeftDriver(void) {

  palSetPad(LMD_EN_PORT, LMD_EN);
}

/**
 * @brief   Activate the right driver to use right motor.
 */
static void motorsEnableRightDriver(void) {

  palSetPad(RMD_EN_PORT, RMD_EN);
}

/**
 * @brief   Desactivate the left driver to disable left motor.
 */
static void motorsDisableLeftDriver(void) {

  palClearPad(LMD_EN_PORT, LMD_EN);
}

/**
 * @brief   Desactivate the right driver to disable right motor.
 */
static void motorsDisableRightDriver(void) {

  palClearPad(RMD_EN_PORT, RMD_EN);
}
/**
 * @brief   Generate the corresponding PWM for speed control.
 *
 * @param[in] motor       the motor to pilot, rigth or left
 * @param[in] direction   the direction of the motor, backward or forward
 * @param[in] dutyCycle   the duty cycle to set the pwm
 */
void motorsSetPWM(uint8_t motor, uint8_t direction, uint16_t dutyCycle) {

#if (DEBUG == TRUE)
  chprintf(chp, "pwm: %d\t", dutyCycle);
#endif

  if (motor == MOTOR_L) {
    palSetPad(LMD_EN_PORT, LMD_EN);

    if (direction == MOTOR_DIR_F) {
      pwmSetPulseWidth(&PWMD4, 0, PWM_MAX_VALUE);     /* Reverse. */
      pwmSetPulseWidth(&PWMD4, 2, dutyCycle);         /* Forward. */
    }
    else if (direction == MOTOR_DIR_B) {
      pwmSetPulseWidth(&PWMD4, 0, dutyCycle);         /* Reverse. */
      pwmSetPulseWidth(&PWMD4, 2, PWM_MAX_VALUE);     /* Forward. */
    }
  }
  else if (motor == MOTOR_R) {
    palSetPad(RMD_EN_PORT, RMD_EN);

    if (direction == MOTOR_DIR_F) {
      pwmSetPulseWidth(&PWMD3, 1, PWM_MAX_VALUE);     /* Reverse. */
      pwmSetPulseWidth(&PWMD3, 2, dutyCycle);         /* Forward. */
    }
    else if (direction == MOTOR_DIR_B) {
      pwmSetPulseWidth(&PWMD3, 1, dutyCycle);         /* Reverse. */
      pwmSetPulseWidth(&PWMD3, 2, PWM_MAX_VALUE);     /* Forward. */
    }
  }
}

/**
 * @brief   Stop the corresponding motor.
 *
 * @param[in] motor   the motor to stop, rigth or left
 */
void motorsStop(uint8_t motor) {

  if (motor == MOTOR_L) {
    motorsSetPWM(MOTOR_L, MOTOR_DIR_F, 0);
    motorsDisableLeftDriver();
  }

  if (motor == MOTOR_R) {
    motorsSetPWM(MOTOR_R, MOTOR_DIR_F, 0);
    motorsDisableRightDriver();
  }
}

/**
 * @brief   Stop both motors and initialize wheels position, PID parameters.
 */
void motorsStopAndReset(void) {

  motorsStop(MOTOR_R);
  motorsStop(MOTOR_L);
  pidParametersReset();
}

/**
 * @brief   Driving the motor to set to the given speed.
 *
 * @param[in] motor       the motor to pilot, rigth or left
 * @param[in] direction   the direction of the motor, backward or forward
 * @param[in] speedRaw    the speed to set the motor
 */
void motorsMove(uint8_t motor, uint8_t direction, double speedRaw) {

  uint16_t speed;

  if (speedRaw > PWM_MAX_VALUE)
    speedRaw = PWM_MAX_VALUE;

  speed = speedRaw*((double)PWMVALUE)/PWM_MAX_VALUE;
  motorsSetPWM(motor, direction, speed);
}

/**
 * @brief   The encoders decrease when motor is traveling forward and increase
 *          when traveling backward.
 *
 * @return  leftCounter   the value of the left encoder
 */
uint32_t motorsReadLeftEncoder(void) {

  return leftCounter;
}

/**
 * @brief   The encoders decrease when motor is traveling forward and increase
 *          when traveling backward.
 *
 * @return  rightCounter  the value of the right encoder
 */
uint32_t motorsReadRightEncoder(void) {

  return rightCounter;
}

// TODO: See how to use just one function to read encoder state by using an
// enumeration parameter.

/**
 * @brief   return the state of the left encder A.
 *
 * @return  ret   left encoder A state (can be true or false)
 *//*
static bool motorsReadLeftEncoderStateA(void) {

  if (lstateA)
    return true;
  else
    return false;
}*/

/**
 * @brief   return the state of the left encder B.
 *
 * @return  ret   left encoder B state (can be true or false)
 *//*
static bool motorsReadLeftEncoderStateB(void) {

  if (lstateB)
    return true;
  else
    return false;
}*/

/**
 * @brief   return the state of the rigth encder A.
 *
 * @return  ret  right encoder A state (can be true or false)
 *//*
static bool motorsReadRightEncoderStateA(void) {

  if (rstateA)
    return true;  //ret = 1;
  else
    return false; //ret = 0;
}*/

/**
 * @brief   return the state of the rigth encder B.
 *
 * @return  ret  right encoder B state (can be true or false).
 *//*
static bool motorsReadRightEncoderStateB(void) {

  if (rstateB)
    return true;
  else
    return false;
}*/

/**
 * @brief   Initialize all pins needs for motor control.
 */
void motorsInit(void) {

  /* Set left Motor encoders.                                               */
  palSetPadMode(L_ENCODER_A_PORT, L_ENCODER_A, PAL_MODE_INPUT);
  palSetPadMode(L_ENCODER_B_PORT, L_ENCODER_B, PAL_MODE_INPUT);

  /* Set Rigth motor encoders.                                              */
  palSetPadMode(R_ENCODER_A_PORT, R_ENCODER_A, PAL_MODE_INPUT);
  palSetPadMode(R_ENCODER_B_PORT, R_ENCODER_B, PAL_MODE_INPUT);

  /* Setup Left Motor Driver ( LMD ).                                       */
  palSetPadMode(LMD_RPWM_PORT,  LMD_RPWM, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(LMD_LPWM_PORT,  LMD_LPWM, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(LMD_EN_PORT,    LMD_EN,   PAL_MODE_OUTPUT_PUSHPULL);

  /* Setup Rigth Motor Driver ( RMD ).                                      */
  palSetPadMode(RMD_RPWM_PORT,  RMD_RPWM, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(RMD_LPWM_PORT,  RMD_LPWM, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(RMD_EN_PORT,    RMD_EN,   PAL_MODE_OUTPUT_PUSHPULL);

  /* Enable Motors.                                                         */
  motorsEnableRightDriver();
  motorsEnableLeftDriver();

  /* Configure the EXT Driver and Validate the Externals interrupts.        */
  extStart(&EXTD1, &extcfg);
  extChannelEnable(&EXTD1, INT2);
  extChannelEnable(&EXTD1, INT3);
}

/**
 * @brief   Get the wheel velocity for asservissement routine.
 */
void motorsGetWheelVelocity(void) {

  loopCounter++;

  if (loopCounter == 10) {
    loopCounter = 0;
    wheelPosition = motorsReadLeftEncoder() + motorsReadRightEncoder();
    wheelVelocity = wheelPosition - lastWheelPosition;
    lastWheelPosition = wheelPosition;

    if (abs(wheelVelocity) <= 20 && !stopped) {
      /* Set new targetPosition if braking. */
      targetPosition = wheelPosition;
      stopped = true;
    }
  }
}

