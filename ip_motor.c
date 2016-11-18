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
 * @update        17 November 2016
 *
 * @description   Motor control and Encoder Read
 *                Description:
 *                Get the PWM control value from the I2C.
 *                Send the Encoder value to the I2C master when required.
 *                Motor Power:	white
 *                Motor Power:	yellow
 *                Encoder GND:	blue
 *                Encoder VCC:	green
 *                Encoder A:		black
 *                Encoder B:		Red
 */

/*===========================================================================*/
/* Includes files.                                                           */
/*===========================================================================*/
#include "ip_motor.h"
#include "chprintf.h"

/*===========================================================================*/
/* Macro definitions.                                                        */
/*===========================================================================*/
#define DEBUG 1

/*===========================================================================*/
/* Global variables.                                                         */
/*===========================================================================*/
long wheelPosition;     /**< TODO: comment                                   */
long wheelVelocity;     /**< TODO: comment                                   */
long lastWheelPosition; /**< TODO: comment                                   */
long targetPosition;    /**< TODO: comment                                   */

static bool     stopped;                /**< Target position after breaking  */
static long     leftCounter   = 0;      /**< TODO: comment                   */
static long     rightCounter  = 0;      /**< TODO: comment                   */
static bool     lstateA       = false;
static bool     lstateB       = false;
static bool     rstateA       = false;
static bool     rstateB       = false;
static uint8_t  loopCounter   = 0;      /**< Used to update wheel velocity   */

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
    {EXT_CH_MODE_DISABLED, NULL},               /***< INT0 Config */
    {EXT_CH_MODE_DISABLED, NULL},               /***< INT1 Config */
    {EXT_CH_MODE_RISING_EDGE, cbRightEncoder},  /***< INT2 Config */
    {EXT_CH_MODE_RISING_EDGE, cbLeftEncoder},   /***< INT3 Config */
    {EXT_CH_MODE_DISABLED, NULL},               /***< INT4 Config */
    {EXT_CH_MODE_DISABLED, NULL},               /***< INT5 Config */
  }
};

/*===========================================================================*/
/* Functions.                                                                */
/*===========================================================================*/

/**
 * @fn      stopMotor
 * @brief   Stop the corresponding motor.
 *
 * @param[in] motor   the motor to stop, rigth or left
 */
void stopMotor(uint8_t motor) {
  if (motor == MOTOR_L)
    setPWM(MOTOR_L, MOTOR_DIR_F, 0);
  if (motor == MOTOR_R)
    setPWM(MOTOR_R, MOTOR_DIR_F, 0);
}

/**
 * @fn      setPWM
 * @brief   Generate the corresponding PWM for speed control.
 *
 * @param[in] motor       the motor to pilot, rigth or left.
 * @param[in] direction   the direction of the motor, backward or forward.
 * @param[in] dutyCycle   the duty cycle to set the pwm.
 */
void setPWM(uint8_t motor, uint8_t direction, uint16_t dutyCycle) {
  // Calcul the numeric value of the PWM.
  dutyCycle = (dutyCycle*1023)/100;

  if (motor == MOTOR_L) {
    palSetPad(LMD_EN_PORT, LMD_EN);

    if (direction == MOTOR_DIR_F) {
      pwmEnableChannel(&PWMD4, 0, 1023);      // Reverse
      pwmEnableChannel(&PWMD4, 2, dutyCycle); // Forward
    }
    else if (direction == MOTOR_DIR_B) {
      pwmEnableChannel(&PWMD4, 0, dutyCycle);  // Reverse
      pwmEnableChannel(&PWMD4, 2, 1023);       // Forward
    }
  }
  else if (motor == MOTOR_R) {
    palSetPad(RMD_EN_PORT, RMD_EN);

    if (direction == MOTOR_DIR_F) {
      pwmEnableChannel(&PWMD3, 1, 1023);      // Reverse
      pwmEnableChannel(&PWMD3, 2, dutyCycle); // Forward
    }
    else if (direction == MOTOR_DIR_B) {
      pwmEnableChannel(&PWMD3, 1, dutyCycle); // Reverse
      pwmEnableChannel(&PWMD3, 2, 1023);      // Forward
    }
  }
}

/**
 * @fn      motorsStopAndReset
 * @brief   Stop both motors and initialize wheels position, PID parameters.
 */
void motorsStopAndReset(void) {
  stopMotor(MOTOR_R);
  stopMotor(MOTOR_L);
  pidParametersReset();
}

/**
 * @fn      moveMotor
 * @brief   Driving the motor to set to the given speed.
 *
 * @param[in] motor       the motor to pilot, rigth or left
 * @param[in] direction   the direction of the motor, backward or forward
 * @param[in] speedRaw    the speed to set the motor
 */
void moveMotor(uint8_t motor, uint8_t direction, double speedRaw) {
  int speed;

  if(speedRaw > 255)
    speedRaw = 255;

  speed = speedRaw*((double)PWMVALUE)/255; // Scale from 100 to PWM_VALUE
  setPWM(motor, direction, speed);
}

/**
 * @fn      readLeftEncoder
 * @brief   The encoders decrease when motor is traveling forward and increase
 *          when traveling backward.
 *
 * @return  leftCounter   the value of the left encoder
 */
long readLeftEncoder(void) {
  return leftCounter;
}

/**
 * @fn      readRightEncoder
 * @brief   The encoders decrease when motor is traveling forward and increase
 *          when traveling backward.
 *
 * @return  rightCounter  the value of the right encoder
 */
long readRightEncoder(void) {
  return rightCounter;
}

/**
 * @fn      readLeftEncoderStateA
 * @brief   return the state of the left encder A.
 *
 * @return  ret the value of the left encoder
 */
long readLeftEncoderStateA(void) {
  long ret;

  if(lstateA)
    ret = 1;
  else
    ret = 0;

  return ret;
}

/**
 * @fn      readLeftEncoderStateB
 * @brief   return the state of the left encder B.
 *
 * @return  ret the value of the left encoder B
 */
long readLeftEncoderStateB(void) {
  long ret;

  if(lstateB)
    ret = 1;
  else
    ret = 0;

  return ret;
}

/**
 * @fn      readRightEncoderStateA
 * @brief   return the state of the rigth encder A.
 *
 * @return  ret  the value of the right encoder A
 */
long readRightEncoderStateA(void) {
  long ret;

  if(rstateA)
    ret = 1;
  else
    ret = 0;

  return ret;
}

/**
 * @fn      readRightEncoderStateB
 * @brief   return the state of the rigth encder B.
 *
 * @return  ret  the value of the right encoder B
 */
long readRightEncoderStateB(void) {
  long ret;

  if(rstateB)
    ret = 1;
  else
    ret = 0;

  return ret;
}

/**
 * @fn      motorInit
 * @brief   Initialize all pins needs for motor control
 */
void motorInit(void) {

  // Set left Motors Encoders
  palSetPadMode(L_ENCODER_A_PORT, L_ENCODER_A, PAL_MODE_INPUT); // pin D19 [PD2]
  palSetPadMode(L_ENCODER_B_PORT, L_ENCODER_B, PAL_MODE_INPUT); // pin D5  [PG5]

  // Set Rigth motor encoders
  palSetPadMode(R_ENCODER_A_PORT, R_ENCODER_A, PAL_MODE_INPUT); // pin D18 [PD3]
  palSetPadMode(R_ENCODER_B_PORT, R_ENCODER_B, PAL_MODE_INPUT); // pin D3  [PE5]

  // Setup Left Motor Driver ( LMD )
  palSetPadMode(LMD_RPWM_PORT, LMD_RPWM, PAL_MODE_OUTPUT_PUSHPULL); // pin
  palSetPadMode(LMD_LPWM_PORT, LMD_LPWM, PAL_MODE_OUTPUT_PUSHPULL); // pin
  palSetPadMode(LMD_EN_PORT, LMD_EN, PAL_MODE_OUTPUT_PUSHPULL);     // pin

  // Setup Rigth Motor Driver ( RMD )
  palSetPadMode(RMD_RPWM_PORT, RMD_RPWM, PAL_MODE_OUTPUT_PUSHPULL); // pin
  palSetPadMode(RMD_LPWM_PORT, RMD_LPWM, PAL_MODE_OUTPUT_PUSHPULL); // pin
  palSetPadMode(RMD_EN_PORT, RMD_EN, PAL_MODE_OUTPUT_PUSHPULL);     // pin

  // Eneble Motors.
  palSetPad(LMD_EN_PORT, LMD_EN);
  palSetPad(RMD_EN_PORT, RMD_EN);

  // Motor Enable pins configuration.
  //palSetPadMode(IOPORT2, PB4, PAL_MODE_OUTPUT_PUSHPULL); // rigth motor Enable
  //palSetPadMode(IOPORT8, PH4, PAL_MODE_OUTPUT_PUSHPULL); // left motor Enable

  // Configure the EXT Driver and Validate the External interrupt for encoders:
  extStart(&EXTD1, &extcfg);
  extChannelEnable(&EXTD1, INT2); // pin D19 [PD2]
  extChannelEnable(&EXTD1, INT3); // pin D18 [PD3]
}

/**
 * @fn      motorGetWheelVelocity
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
      // Set new targetPosition if braking.
      targetPosition = wheelPosition;
      stopped = true;
    }
  }
}

