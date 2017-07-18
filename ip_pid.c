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

/* Standard Files.  */
#include <stdlib.h>
#include <stdbool.h>

/* Locale files.    */
#include "ip_motor.h"

/*==========================================================================*/
/* Global variables.                                                        */
/*==========================================================================*/

static double lastError;    /**< Store position error.                      */
static double iTerm;        /**< Store integral term.                       */
static double error;        /**< Error between two measurement.             */
static double pTerm;        /**< Proportionnal error.                       */
static double dTerm;        /**< Derivate error.                            */
static double pidValue;     /**< PID value, sum of all the errors.          */
static double pidLeft;      /**< PID result for Left motor.                 */
static double pidRight;     /**< PID result for Right motor.                */

static double kp = 55.468;  /**< Proportional parameter of PID corrector.   */
static double ki = 0.554;   /**< Integral parameter of PID corrector.       */
static double kd = 42.524;  /**< Derivate parameter of PID corrector.       */

static double velocityScaleStop = 30;     /**< Max velocity of the robot.   */
static double velocityScaleTurning = 35;  /**< Max turning velocity.        */

static int16_t zoneA = 4000;           /**< Area to ajust robot PID.        */
static int16_t zoneB = 2000;           /**< Area to ajust robot PID.        */
static double positionScaleA = 250;    /**< One resolution is 464 pulses.   */
static double positionScaleB = 500;    /**< Max position scale for control. */
static double positionScaleC = 1000;   /**< Max position scale for control. */
static double velocityScaleMove = 35;  /**< Velocity scale use to move.     */

bool steerForward;           /**< Robot oriantation forward.                */
bool steerBackward;          /**< Robot oriantation backward.               */
bool steerStop      = true;  /**< Stop by default.                          */
bool steerLeft;              /**< Robot orientation left.                   */
bool steerRight;             /**< Robot oriantation rigth.                  */

extern long wheelPosition;
extern long lastWheelPosition;
extern long wheelVelocity;
extern long targetPosition;

/*==========================================================================*/
/* Functions.                                                               */
/*==========================================================================*/

/**
 * @brief   Calcul the command to send to the motors according to last error.
 *
 * @param[in] pitch      mesured angle of the robot
 * @param[in] restAngle  target angle of the robot
 * @param[in] offset     angle we want to add to the target angle
 * @param[in] turning    value use to turn robot over rigth or left
 */
void pid(double pitch, double restAngle, double offset, double turning) {

  /* Steer robot forward. */
  if (steerForward) {
    /* Scale down offset at high speed and scale up when reversing */
    offset += (double)wheelVelocity/velocityScaleMove;
    restAngle -= offset;
  }

  /* Steer robot backward. */
  else if (steerBackward) {
    /* Scale down offset at high speed and scale up when reversing */
    offset -= (double)wheelVelocity/velocityScaleMove;
    restAngle += offset;
  }

  /* Default steer. */
  else if (steerStop) {
    uint32_t positionError = wheelPosition - targetPosition;
    if (abs(positionError) > zoneA)                       /* Inside zone A. */
      restAngle -= (double)positionError/positionScaleA;
    else if (abs(positionError) > zoneB)                  /* Inside zone B. */
      restAngle -= (double)positionError/positionScaleB;
    else                                                  /* Inside zone C. */
      restAngle -= (double)positionError/positionScaleC;
    restAngle -= (double)wheelVelocity/velocityScaleStop;
    if (restAngle < 160) // Limit rest Angle
      restAngle = 160;
    else if (restAngle > 200)
      restAngle = 200;
  }

  // TODO: make this function more simple. The real PID is bellow.

  /* Update PID values. */
  error     =   (restAngle - pitch);
  pTerm     =   kp * error;
  iTerm     +=  ki * error;
  dTerm     =   kd * (error - lastError);
  lastError =   error;
  pidValue  =   pTerm + iTerm + dTerm;

  // TODO: Here is the end of the PID controller.

  /* Steer robot sideways. */
  if (steerLeft) {
    /* Scale down at high speed */
    turning -= abs((double)wheelVelocity/velocityScaleTurning);

    if (turning < 0)
      turning = 0;

    pidLeft   = pidValue-turning;
    pidRight  = pidValue+turning;
  }
  else if (steerRight) {
    /* Scale down at high speed */
    turning -= abs((double)wheelVelocity/velocityScaleTurning);

    if (turning < 0)
      turning = 0;

    pidLeft = pidValue+turning;
    pidRight = pidValue-turning;
  }
  else {
    pidLeft = pidValue;
    pidRight = pidValue;
  }

  /* Set the left motor PWM value. */
  if (pidLeft >= 0)
    motorsMove(MOTOR_L, MOTOR_DIR_F, pidLeft);
  else
    motorsMove(MOTOR_L, MOTOR_DIR_B, abs(pidLeft));

  /* Set the rigth motor PWM value. */
  if (pidRight >= 0)
    motorsMove(MOTOR_R, MOTOR_DIR_F, pidRight);
  else
    motorsMove(MOTOR_R, MOTOR_DIR_B, abs(pidRight));
}

/**
 * @brief   Reset the PID parameters.
 */
void pidParametersReset(void) {

  pTerm     = 0;
  iTerm     = 0;
  dTerm     = 0;
  lastError = 0;
  targetPosition = wheelPosition;
}

