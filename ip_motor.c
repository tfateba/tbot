/**
 *
 * @file          ip_motor.c
 *
 * @brief         motor driver.
 *
 * @author        Theodore Ateba, tf.ateba@gmail.com
 *
 * @date          07 Septembre 2015
 *
 * @update        16 January 2017
 *
 * @description   Motor control and Encoder Read
 *                Description:
 *                Get the PWM control value from the I2C.
 *                Send the Encoder value to the I2C master when required.
 *                Motor Power:  white
 *                Motor Power:  yellow
 *                Encoder GND:  blue
 *                Encoder VCC:  green
 *                Encoder A:    black
 *                Encoder B:    Red
 */

/*===========================================================================*/
/* Includes files.                                                           */
/*===========================================================================*/
#include "ip_motor.h"

/*===========================================================================*/
/* Application macros.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Global variables.                                                         */
/*===========================================================================*/
#if (DEBUG == TRUE)
extern BaseSequentialStream* chp; /* Variable for debug message.             */
#endif
long wheelPosition;     /**< Position of the wheel.                          */
long wheelVelocity;     /**< Velociity of the wheel.                         */
long lastWheelPosition; /**< Memo for wheel position.                        */
long targetPosition;    /**< Angular position of the robot.                  */

static bool     stopped;                /**< Target position after breaking  */
static long     leftCounter   = 0;      /**< Left encoder counter.           */
static long     rightCounter  = 0;      /**< Rigth encoder counter.          */
static bool     lstateA       = false;  /**< Variable for asservissement.    */ 
static bool     lstateB       = false;  /**< Variable for asservissement.    */
static bool     rstateA       = false;  /**< Variable for asservissement.    */
static bool     rstateB       = false;  /**< Varaible for asservissement.    */
static uint8_t  loopCounter   = 0;      /**< Used to update wheel velocity   */
const uint16_t  maxValue = 512;

/*===========================================================================*/
/* Encoders callback.                                                        */
/*===========================================================================*/

/**
 * @brief   This is the External interrupt callback for the Left motor
 *          encoder.
 */
static void cbLeftEncoder(EXTDriver *extp, expchannel_t channel) {
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
static void cbRightEncoder(EXTDriver *extp, expchannel_t channel) {
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
    {EXT_CH_MODE_DISABLED, NULL},               /* INT0 Config */
    {EXT_CH_MODE_DISABLED, NULL},               /* INT1 Config */
    {EXT_CH_MODE_RISING_EDGE, cbRightEncoder},  /* INT2 Config */
    {EXT_CH_MODE_RISING_EDGE, cbLeftEncoder},   /* INT3 Config */
    {EXT_CH_MODE_DISABLED, NULL},               /* INT4 Config */
    {EXT_CH_MODE_DISABLED, NULL},               /* INT5 Config */
  }
};

/*===========================================================================*/
/* Functions.                                                                */
/*===========================================================================*/

/**
 * @brief   Stop the corresponding motor.
 *
 * @param[in] motor   the motor to stop, rigth or left
 */
void stopMotor(uint8_t motor) {
  if (motor == MOTOR_L) {
    setPWM(MOTOR_L, MOTOR_DIR_F, 0);
    palClearPad(LMD_EN_PORT, LMD_EN);
  }

  if (motor == MOTOR_R) {
    setPWM(MOTOR_R, MOTOR_DIR_F, 0);
    palClearPad(RMD_EN_PORT, RMD_EN);
  }
}

/**
 * @brief   Generate the corresponding PWM for speed control.
 *
 * @param[in] motor       the motor to pilot, rigth or left.
 * @param[in] direction   the direction of the motor, backward or forward.
 * @param[in] dutyCycle   the duty cycle to set the pwm.
 */
void setPWM(uint8_t motor, uint8_t direction, uint16_t dutyCycle) {
  #if (DEBUG == TRUE)
  chprintf(chp, "pwm: %d\t", dutyCycle);
  #endif

  if (motor == MOTOR_L) {
    palSetPad(LMD_EN_PORT, LMD_EN);

    if (direction == MOTOR_DIR_F) {
      pwm_setPulseWidth(&PWMD4, 0, maxValue);   /* Reverse. */
      pwm_setPulseWidth(&PWMD4, 2, dutyCycle);  /* Forward. */
    }
    else if (direction == MOTOR_DIR_B) {
      pwm_setPulseWidth(&PWMD4, 0, dutyCycle);  /* Reverse. */
      pwm_setPulseWidth(&PWMD4, 2, maxValue);   /* Forward. */
    }
  }
  else if (motor == MOTOR_R) {
    palSetPad(RMD_EN_PORT, RMD_EN);

    if (direction == MOTOR_DIR_F) {
      pwm_setPulseWidth(&PWMD3, 1, maxValue);   /* Reverse. */
      pwm_setPulseWidth(&PWMD3, 2, dutyCycle);  /* Forward. */
    }
    else if (direction == MOTOR_DIR_B) {
      pwm_setPulseWidth(&PWMD3, 1, dutyCycle);  /* Reverse. */
      pwm_setPulseWidth(&PWMD3, 2, maxValue);   /* Forward. */
    }
  }
}

/**
 * @brief   Stop both motors and initialize wheels position, PID parameters.
 */
void motorsStopAndReset(void) {
  stopMotor(MOTOR_R);
  stopMotor(MOTOR_L);
  pidParametersReset();
}

/**
 * @brief   Driving the motor to set to the given speed.
 *
 * @param[in] motor       the motor to pilot, rigth or left
 * @param[in] direction   the direction of the motor, backward or forward
 * @param[in] speedRaw    the speed to set the motor
 */
void moveMotor(uint8_t motor, uint8_t direction, double speedRaw) {
  int speed;

  if (speedRaw > maxValue)
    speedRaw = maxValue;

  speed = speedRaw*((double)PWMVALUE)/maxValue; /* Scale from 100 to PWM_VALUE. */
  setPWM(motor, direction, speed);
}

/**
 * @brief   The encoders decrease when motor is traveling forward and increase
 *          when traveling backward.
 *
 * @return  leftCounter   the value of the left encoder
 */
long readLeftEncoder(void) {
  return leftCounter;
}

/**
 * @brief   The encoders decrease when motor is traveling forward and increase
 *          when traveling backward.
 *
 * @return  rightCounter  the value of the right encoder
 */
long readRightEncoder(void) {
  return rightCounter;
}

/**
 * @brief   Initialize all pins needs for motor control
 */
void enableLeftMotor(void) {
  palSetPad(LMD_EN_PORT, LMD_EN);
}

/**
 * @brief   Initialize all pins needs for motor control
 */
void enableRightMotor(void) {
  palSetPad(RMD_EN_PORT, RMD_EN);
}

/**
 * @brief   Initialize all pins needs for motor control
 */
void disableLeftMotor(void) {
  palClearPad(LMD_EN_PORT, LMD_EN);
}

/**
 * @brief   Initialize all pins needs for motor control
 */
void disableRightMotor(void) {
  palClearPad(RMD_EN_PORT, RMD_EN);
}

/**
 * @brief   Initialize all pins needs for motor control
 */
void motorInit(void) {

  /* Set left Motors Encoders. */
  palSetPadMode(L_ENCODER_A_PORT, L_ENCODER_A, PAL_MODE_INPUT); /* D18 [Pxx] */
  palSetPadMode(L_ENCODER_B_PORT, L_ENCODER_B, PAL_MODE_INPUT); /* D4  [Pxx] */

  /* Set Rigth motor encoders */
  palSetPadMode(R_ENCODER_A_PORT, R_ENCODER_A, PAL_MODE_INPUT); /* D19 [Pxx] */
  palSetPadMode(R_ENCODER_B_PORT, R_ENCODER_B, PAL_MODE_INPUT); /* D5  [Pxx] */

  /* Setup Left Motor Driver ( LMD ) */
  palSetPadMode(LMD_RPWM_PORT,  LMD_RPWM, PAL_MODE_OUTPUT_PUSHPULL); /* D2  [Pxx] */
  palSetPadMode(LMD_LPWM_PORT,  LMD_LPWM, PAL_MODE_OUTPUT_PUSHPULL); /* D3  [Pxx] */
  palSetPadMode(LMD_EN_PORT,    LMD_EN,   PAL_MODE_OUTPUT_PUSHPULL); /* D11 [Pxx] */

  /* Setup Rigth Motor Driver ( RMD ) */
  palSetPadMode(RMD_RPWM_PORT,  RMD_RPWM, PAL_MODE_OUTPUT_PUSHPULL); /* D8  [Pxx] */
  palSetPadMode(RMD_LPWM_PORT,  RMD_LPWM, PAL_MODE_OUTPUT_PUSHPULL); /* D6  [Pxx] */
  palSetPadMode(RMD_EN_PORT,    RMD_EN,   PAL_MODE_OUTPUT_PUSHPULL); /* D7  [Pxx] */

  /* Eneble Motors. */
  enableRightMotor();
  enableLeftMotor();

  /* Configure the EXT Driver and Validate the Externals interrupts for encoders: */
  extStart(&EXTD1, &extcfg);
  extChannelEnable(&EXTD1, INT2); /* D19 [PD2] */
  extChannelEnable(&EXTD1, INT3); /* D18 [PD3] */
}

/**
 * @brief   Get the wheel velocity for asservissement routine.
 */
void motorGetWheelVelocity(void) {
  loopCounter++;

  if (loopCounter == 10) {
    loopCounter = 0;
    wheelPosition = readLeftEncoder() + readRightEncoder();
    wheelVelocity = wheelPosition - lastWheelPosition;
    lastWheelPosition = wheelPosition;

    if (abs(wheelVelocity) <= 20 && !stopped) {
      /* Set new targetPosition if braking. */
      targetPosition = wheelPosition;
      stopped = true;
    }
  }
}

