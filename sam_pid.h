/**
 * @file    sam_pid.h
 *
 * @brief   pid corrector header file.
 *
 * @author  Theodore Ateba
 *
 * @date    13 July 2016
 *
 * @update  13 July 2016
 *
 */

#ifndef _SAM_PID_H_
#define _SAM_PID_H_

/*===========================================================================*/
/* Include file.                                                             */
/*===========================================================================*/
#include <stdbool.h>

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief PID Modes enumerations.
 */
typedef enum{
  PID_MODE_AUTOMATIC = 0x00,
  PID_MODE_MANUAL    = 0x01
} pid_mode_e;

/**
 * @brief PID Directions eneumerations.
 */
typedef enum{
  PID_DIRECTION_DIRECT  = 0x00,
  PID_DIRECTION_REVERSE = 0x01
} pid_direction_e;

/**
 * @brief PID Controller structure
 */
typedef struct {
  float dispKp;     // * we'll hold on to the tuning parameters in user-entered
  float dispKi;     //   format for display purposes
  float dispKd;     //

  float kp;         // * (P)roportional Tuning Parameter
  float ki;         // * (I)ntegral Tuning Parameter
  float kd;         // * (D)erivative Tuning Parameter

  pid_direction_e controllerDirection;

  float input;    // * Pointers to the Input, Output, and Setpoint variables
  float output;   //   This creates a hard link between the variables and the
  float setpoint; //   PID, freeing the user from having to constantly tell us
                    //   what these values are.  with pointers we'll just know.

  //  unsigned long lastTime;
  float ITerm, lastInput;

  unsigned long SampleTime;
  float outMin, outMax;
  bool inAuto;
} pid_t;

//commonly used functions **************************************************************************

//  constructor.  links the PID to the Input, Output, and
//  Setpoint.  Initial tuning parameters are also set here
void pid_init(pid_t *pid,
    float kp,
    float ki,
    float kd,
    pid_direction_e controllerDirection);

// sets PID to either Manual (0) or Auto (non-0)
void pid_setMode(pid_t *pid, pid_mode_e mode);

// performs the PID calculation.  it should be
// called every time loop() cycles. ON/OFF and
// calculation frequency can be set using SetMode
// SetSampleTime respectively
bool pid_compute(pid_t *pid);

// clamps the output to a specific range. 0-255 by default, but
// it's likely the user will want to change this depending on
// the application
void pid_setOutputLimits(pid_t *pid, float min, float max);

//available but not commonly used functions ********************************************************

// While most users will set the tunings once in the
// constructor, this function gives the user the option
// of changing tunings during runtime for Adaptive control
void pid_setTunings(pid_t *pid, float kp, float ki, float kd);

// Sets the Direction, or "Action" of the controller. DIRECT
// means the output will increase when error is positive. REVERSE
// means the opposite.  it's very unlikely that this will be needed
// once it is set in the constructor.
void pid_setControllerDirection(pid_t *pid, pid_direction_e direction);

// sets the frequency, in Milliseconds, with which
// the PID calculation is performed.  default is 100
void pid_setSampleTime(pid_t *pid, int newSampleTime);

//Display functions ****************************************************************
// These functions query the pid for interal values.
//  they were created mainly for the pid front-end,
// where it's important to know what is actually
//  inside the PID.
float pid_GetKp(pid_t *pid);
float pid_GetKi(pid_t *pid);
float pid_GetKd(pid_t *pid);
pid_mode_e pid_GetMode(pid_t *pid);
pid_direction_e pid_GetDirection(pid_t *pid);

//void pid_initialize(pid_t *pid);
#endif

