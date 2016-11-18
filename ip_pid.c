/**
 * 
 * @file    ip_pid.c
 *
 * @brief   PID controller
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    17 July 2016
 *
 * @update  24 October 2016
 *
 */

/*===========================================================================*/
/* Includes files.                                                           */
/*===========================================================================*/
#include "ip_pid.h"

/*===========================================================================*/
/* Global variables.                                                         */
/*===========================================================================*/
static double lastError; /**< Store position error.                          */
static double iTerm;     /**< Store integral term                            */
static double error;     /**< Error between two measurement                  */
static double pTerm;     /**< Proportionnal error.                           */
static double dTerm;     /**< Derivate error.                                */
static double PIDValue;  /**< PID value, sum of all the errors.              */
static double PIDLeft;   /**< PID result for Left motor.                     */
static double PIDRight;  /**< PID result for Right motor.                    */

static double Kp = 13.6;   /**< Proportional parameter of PID corrector.     */
static double Ki = 0.0377; /**< Integral parameter of PID corrector.         */
static double Kd = 9.3;    /**< Derivate parameter of PID corrector.         */

static double velocityScaleStop = 30;    /**< TODO: comment                  */
static double velocityScaleTurning = 35; /**< TODO: comment                  */

static int16_t zoneA = 4000;          /**< TODO: comment                     */
static int16_t zoneB = 2000;          /**< TODO: comment                     */
static double positionScaleA = 250;   /**< One resolution is 464 pulses      */
static double positionScaleB = 500;   /**< TODO: comment                     */
static double positionScaleC = 1000;  /**< TODO: comment                     */
static double velocityScaleMove = 35; /**< TODO: comment                     */

extern long wheelPosition;      /**< TODO: comment                           */
extern long lastWheelPosition;  /**< TODO: comment                           */
extern long wheelVelocity;      /**< TODO: comment                           */
extern long targetPosition;     /**< TODO: comment                           */

bool steerForward;          /**< TODO: comment                               */
bool steerBackward;         /**< TODO: comment                               */
bool steerStop      = true; /* Stop by default                               */
bool steerLeft;             /**< TODO: comment                               */
bool steerRight;            /**< TODO: comment                               */

/*===========================================================================*/
/* Functions.                                                                */
/*===========================================================================*/

/**
 * @fn      pid
 * @brief   Calcul the command to send to the motors according to last error.
 *
 * @param[in] restAngle TODO: comment
 * @param[in] offset    TODO: comment
 * @param[in] turning   TODO: comment
 */
void pid(double pitch, double restAngle, double offset, double turning) {
  /*
   * Read the Potentiometer to tune the PID:
   * This is done juste once when your want to find the rigth PID parameters
   * for your system.
   */
  /*
  // TODO: Use the chibios ADC read functions.
  Kp = (double)analogRead(A15)/10.0;
  Serial.print(Kp);
  Serial.print("\t");
  Ki = (double)analogRead(A14)/10000.000;
  Serial.print(Ki, 4);
  Serial.print("\t");
  Kd = (double)analogRead(A13)/10.0;
  Serial.println(Kd);
  */

  //==> Steer robot
  if (steerForward) {
    /* Scale down offset at high speed and scale up when reversing */
    offset += (double)wheelVelocity/velocityScaleMove;
    restAngle -= offset;
  }
  else if (steerBackward) {
    /* Scale down offset at high speed and scale up when reversing */
    offset -= (double)wheelVelocity/velocityScaleMove;
    restAngle += offset;
  }
  //==> Brake
  else if (steerStop) {
    long positionError = wheelPosition - targetPosition;
    if (abs(positionError) > zoneA) // Inside zone A
      restAngle -= (double)positionError/positionScaleA;
    else if (abs(positionError) > zoneB) // Inside zone B
      restAngle -= (double)positionError/positionScaleB;
    else // Inside zone C
      restAngle -= (double)positionError/positionScaleC;
    restAngle -= (double)wheelVelocity/velocityScaleStop;
    if (restAngle < 160) // Limit rest Angle
      restAngle = 160;
    else if (restAngle > 200)
      restAngle = 200;
  }

  //==> Update PID values
  error = (restAngle - pitch);
  pTerm = Kp * error;
  iTerm += Ki * error;
  dTerm = Kd * (error - lastError);
  lastError = error;
  PIDValue = pTerm + iTerm + dTerm;

  //==> Steer robot sideways
  if (steerLeft) {
    /* Scale down at high speed */
    turning -= abs((double)wheelVelocity/velocityScaleTurning);
    if (turning < 0)
      turning = 0;
    PIDLeft = PIDValue-turning;
    PIDRight = PIDValue+turning;
  }
  else if (steerRight) {
    /* Scale down at high speed */
    turning -= abs((double)wheelVelocity/velocityScaleTurning);
    if (turning < 0)
      turning = 0;
    PIDLeft = PIDValue+turning;
    PIDRight = PIDValue-turning;
  }
  else {
    PIDLeft = PIDValue;
    PIDRight = PIDValue;
  }

  /* Set the left motor PWM value. */
  if (PIDLeft >= 0)
    moveMotor(MOTOR_L, MOTOR_DIR_F, PIDLeft);
  else
    moveMotor(MOTOR_L, MOTOR_DIR_B, PIDLeft * -1);

  /* Set the rigth motor PWM value. */
  if (PIDRight >= 0)
    moveMotor(MOTOR_R, MOTOR_DIR_F, PIDRight);
  else
    moveMotor(MOTOR_R, MOTOR_DIR_B, PIDRight * -1);
}

/**
 * @fn      pidParametersReset
 * @brief   Reset the PID parameters.
 */
void pidParametersReset(void) {
  pTerm     = 0;
  iTerm     = 0;
  dTerm     = 0;
  lastError = 0;
  targetPosition = wheelPosition;
}

