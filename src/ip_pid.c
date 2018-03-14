
/**
 *
 * @file    ip_pid.c
 *
 * @brief   PID corrector source file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    17 July 2016
 *
 */

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

/* Standard files. */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/*==========================================================================*/
/* Global variables.                                                        */
/*==========================================================================*/

/* Local variables. */
static float lastError;   /**< Store position error.                       */
static float iTerm;       /**< Store integral term.                        */
static float error;       /**< Error between two measurement.              */
static float pTerm;       /**< Proportional error.                        */
static float dTerm;       /**< Derivate error.                             */
static float pidValue;    /**< PID value, sum of all the errors.           */

static float kp;          /**< Proportional parameter of PID corrector.    */
static float ki;          /**< Integral parameter of PID corrector.        */
static float kd;          /**< Derivate parameter of PID corrector.        */

static float velocityScaleStop     = 30; /**< Max velocity of the robot.    */
static float velocityScaleTurning  = 35; /**< Max turning velocity.         */

static int16_t zoneA = 4000;         /**< Area to ajust robot PID.         */
static int16_t zoneB = 2000;         /**< Area to ajust robot PID.         */
static float positionScaleA = 250;   /**< One resolution is 464 pulses.     */
static float positionScaleB = 500;   /**< Max position scale for control.   */
static float positionScaleC = 1000;  /**< Max position scale for control.   */
static float velocityScaleMove = 35; /**< Velocity scale use to move.       */

bool steerForward;          /**< Robot orientation forward.                 */
bool steerBackward;         /**< Robot orientation backward.                */
bool steerStop      = true; /* Stop by default                              */
bool steerLeft;             /**< Robot orientation left.                    */
bool steerRight;            /**< Robot orientation right.                   */

/* Extern variables. */
extern long wheelPosition;
//extern long lastWheelPosition;
extern long wheelVelocity;
extern long targetPosition;

/*==========================================================================*/
/* Functions.                                                               */
/*==========================================================================*/

/**
 * @brief   Initialise the Kp Ki and Kd pid term.
 *
 * @param[in] kpval   Kp value to be set
 * @param[in] kival   Ki value to be set
 * @param[in] kdval   Kd value to be set
*/
void pid_init(float kpval, float kival, float kdval) {

  kp = kpval;
  ki = kival;
  kd = kdval;
}

/**
 * @brief   Calcul the command to send to the motors according to last error.
 *
 * @param[in] pitch       measured angle of the robot
 * @param[in] restAngle   target angle of the robot
 * @param[in] offset      angle we want to add to the target angle
 * @param[in] turning     value use to turn robot over right or left
 * @return    result      the result of the pid calcul
*/
float pid(float pitch, float restAngle, float offset, float turning) {

  float result;

  /* Steer robot forward. */
  if (steerForward) {
    /* Scale down offset at high speed and scale up when reversing. */
    offset    += (float)wheelVelocity/velocityScaleMove;
    restAngle -= offset;
  }

  /* Steer robot backward. */
  else if (steerBackward) {
    /* Scale down offset at high speed and scale up when reversing. */
    offset    -= (float)wheelVelocity/velocityScaleMove;
    restAngle += offset;
  }

  /* Default steer. */
  else if (steerStop) {
    long positionError = wheelPosition - targetPosition;
    if (abs(positionError) > zoneA)
      restAngle -= (float)positionError/positionScaleA;
    else if (abs(positionError) > zoneB)
      restAngle -= (float)positionError/positionScaleB;
    else
      restAngle -= (float)positionError/positionScaleC;
    restAngle -= (float)wheelVelocity/velocityScaleStop;
    if (restAngle < 160)
      restAngle = 160;
    else if (restAngle > 200)
      restAngle = 200;
  }

  /* Update PID values. */
  error     =  (restAngle - pitch);
  pTerm     =  kp * error;
  iTerm     += ki * error;
  dTerm     =  kd * (error - lastError);
  lastError =  error;
  pidValue  =  pTerm + iTerm + dTerm;

  /* Steer robot sideways. */
  if (steerLeft) {
    /* Scale down at high speed. */
    turning -= abs((float)wheelVelocity/velocityScaleTurning);
    if (turning < 0)
      turning = 0;
    result   = pidValue-turning;
  }
  else if (steerRight) {
    /* Scale down at high speed. */
    turning -= abs((float)wheelVelocity/velocityScaleTurning);
    if (turning < 0)
      turning = 0;
    result   = pidValue+turning;
  }
  else {
    result   = pidValue;
  }

  return result;
}

/**
 * @brief   Reset the PID parameters.
 */
void pid_Reset_Parameters(void) {
  pTerm           = 0;
  iTerm           = 0;
  dTerm           = 0;
  lastError       = 0;
  targetPosition  = wheelPosition;
}

