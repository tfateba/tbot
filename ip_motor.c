
/**
 *
 * @file          ip_motor.c
 *
 * @brief         Motor driver source file.
 *
 * @author        Theodore Ateba, tf.ateba@gmail.com
 *
 * @date          07 Septembre 2015
 *
 * @description   Motor control and Encoder Read:
 *                Get the PWM control value from the I2C.
 *                Send the Encoder value to the I2C master when required.
 *                Motor Power:  white
 *                Motor Power:  yellow
 *                Encoder GND:  blue
 *                Encoder VCC:  green
 *                Encoder A:    black
 *                Encoder B:    Red
 */

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

/* Standard libraries. */
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

/* ChibiOS libraries. */
#include "hal.h"
#include "chprintf.h"

/* Local files. */
#include "ip_conf.h"
#include "ip_motor.h"
#include "ip_pid.h"
#include "ip_pwm.h"

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
const   uint16_t  maxVal        = 512;    /**< Maximum value.               */

/* Extern variables.  */
extern BaseSequentialStream* chp;

/*==========================================================================*/
/* Encoders callback.                                                       */
/*==========================================================================*/

/**
 * @brief   This is the External interrupt callback for the Left motor
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
 * @brief   This is the External interrupt callback for the Rigth motor
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
    {EXT_CH_MODE_RISING_EDGE, motorRightEncoderCallback}, /**< INT2 Config. */
    {EXT_CH_MODE_RISING_EDGE, motorLeftEncoderCallback},  /**< INT3 Config. */
  }
};

/*==========================================================================*/
/* Functions.                                                               */
/*==========================================================================*/

/**
 * @brief   Stop the corresponding motor.
 *
 * @param[in] motor   the motor to stop, rigth or left
 */
void motorStop(uint8_t motor) {

  if (motor == MOTOR_L) {
    pwmSetDutyCycle(MOTOR_L, MOTOR_DIR_F, 0);
    palClearPad(LMD_EN_PORT, LMD_EN);
  }

  if (motor == MOTOR_R) {
    pwmSetDutyCycle(MOTOR_R, MOTOR_DIR_F, 0);
    palClearPad(RMD_EN_PORT, RMD_EN);
  }
}

/**
 * @brief   Stop both motors and initialize wheels position, PID parameters.
 */
void motorsStopAndReset(void) {

  motorStop(MOTOR_R);
  motorStop(MOTOR_L);
  pidResetParameters();
}

/**
 * @brief   Driving the motor to set to the given speed.
 *
 * @param[in] motor       the motor to pilot, rigth or left
 * @param[in] direction   the direction of the motor, backward or forward
 * @param[in] speedRaw    the speed to set the motor
 */
void motorMove(uint8_t motor, uint8_t direction, double speedRaw) {

  int speed;

  if(speedRaw > maxVal)
    speedRaw = maxVal;

  /* Scale from 100 to PWM_VALUE. */
  speed = speedRaw*((double)PWMVALUE)/maxVal;
  pwmSetDutyCycle(motor, direction, speed);
}

/**
 * @brief  Get the value of the encoder.
 *
 * @param[in] encoder   the encoder to get the value
 *
 * @return    the value of the encoder
 */
static long motorGetEncoderValue(encoder_e encoder) {

  if (encoder == ENCODER_L)
    return leftCounter;

  else if (encoder == ENCODER_R)
    return rightCounter;
  else
    return 0;
}

/* TODO: Change return type of this function to boolean if possible. */

/**
 * @brief  Get the state of the encoder.
 *
 * @param[in] encoder   the encoder to get the state
 * @param[in] state     the state of the encoder to get
 *
 * @return    the state of the encoder
 */
long motorGetEncoderState(encoder_e encoder, encoder_e state) {

  if (encoder == ENCODER_L) {
    if (state == ENCODER_L_STATE_A) {
      if (lstateA)
        return 1;
      else
        return 0;
    }
    else {
      if (lstateB)
        return 1;
      else
        return 0;
    }
  }

  else {
    if (state == ENCODER_R_STATE_A) {
      if (rstateA)
        return 1;
      else
        return 0;
    }
    else {
      if (rstateA)
        return 1;
      else
        return 0;
    }
  }
}

/**
 * @brief   Enable left or right motor with GPIO signal.
 *
 * @param[in] motor   the motor to be enable
 */
void motorEnable(motor_e motor) {

  if (motor == MOTOR_L)
    palSetPad(LMD_EN_PORT, LMD_EN);
  else
    palSetPad(RMD_EN_PORT, RMD_EN);
}

/**
 * @brief   Disable left or right motor with GPIO signal.
 *
 * @param[in] motor   the motor to be disable
 */
void motorDisable(motor_e motor) {

  if (motor == MOTOR_L)
    palClearPad(LMD_EN_PORT, LMD_EN);
  else
    palClearPad(RMD_EN_PORT, RMD_EN);
}

/**
 * @brief   Initialize all pins needs for motor control
 */
void motorInit(void) {

  /* Set left Motors Encoders. */
  palSetPadMode(L_ENCODER_A_PORT, L_ENCODER_A, PAL_MODE_INPUT);
  palSetPadMode(L_ENCODER_B_PORT, L_ENCODER_B, PAL_MODE_INPUT);

  /* Set Rigth motor encoders. */
  palSetPadMode(R_ENCODER_A_PORT, R_ENCODER_A, PAL_MODE_INPUT);
  palSetPadMode(R_ENCODER_B_PORT, R_ENCODER_B, PAL_MODE_INPUT);

  /* Setup Left Motor Driver ( LMD ). */
  palSetPadMode(LMD_RPWM_PORT,  LMD_RPWM, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(LMD_LPWM_PORT,  LMD_LPWM, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(LMD_EN_PORT,    LMD_EN,   PAL_MODE_OUTPUT_PUSHPULL);

  /* Setup Rigth Motor Driver ( RMD ). */
  palSetPadMode(RMD_RPWM_PORT,  RMD_RPWM, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(RMD_LPWM_PORT,  RMD_LPWM, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(RMD_EN_PORT,    RMD_EN,   PAL_MODE_OUTPUT_PUSHPULL);

  /* Eneble Motors. */
  motorEnable(MOTOR_R);
  motorEnable(MOTOR_L);

  /* Configure the EXT Driver. */
  extStart(&EXTD1, &extcfg);

  /* and Validate the Externals interrupts for encoders. */
  extChannelEnable(&EXTD1, INT2);
  extChannelEnable(&EXTD1, INT3);
}

/**
 * @brief   Get the wheel velocity for asservissement routine.
 */
void motorGetWheelVelocity(void) {

  loopCounter++;

  if (loopCounter == 10) {
    loopCounter = 0;
    wheelPosition = motorGetEncoderValue(ENCODER_L); + motorGetEncoderValue(ENCODER_R);
    wheelVelocity = wheelPosition - lastWheelPosition;
    lastWheelPosition = wheelPosition;

    if (abs(wheelVelocity) <= 20 && !stopped) {
      /* Set new targetPosition if braking. */
      targetPosition = wheelPosition;
      stopped = true;
    }
  }
}

