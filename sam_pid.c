/**
 * @file sam_pid.c
 *
 * @brief PID controller
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    17 July 2016
 *
 * @update  30 July 2016
 */

// TODO: Correct the general presentation of this file, also sam_pid.h

#include "sam_pid.h"
void pid_initialize(pid_t* pid);

/**
 * @fn    pid_init
 * @brief Initialization of the PID controller.
 *
 * @param[in] pid                 Limits of the PID output.
 * @param[in] kp                  Proportional gain
 * @param[in] ki                  Integral gain
 * @param[in] kd                  Derivative gain
 * @param[in] controllerDirection Direction of the PID controler
 */
void pid_init(pid_t *pid, float kp, float ki, float kd,
    pid_direction_e ControllerDirection){
  pid->input = 0;
  pid->output = 0;
  pid->setpoint = 0;
  pid->ITerm = 0;
  pid->lastInput = 0;
  pid->inAuto = false;

  pid_setOutputLimits(pid, -255, 255);

  //default Controller Sample Time is 0.1 seconds
  pid->SampleTime = 10;

  pid_setControllerDirection(pid, ControllerDirection);
  pid_setTunings(pid, kp, ki, kd);

//  pid->lastTime = millis() - pid->SampleTime;
}
 
 
/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/ 
/**
 * @fn    pid_compute
 * @brief Calcul the output value of the PID controller
 *
 * @param[in] pid The pointer of the PID controlleur
 * @return        The result of the computing operation 
 */
bool pid_compute(pid_t *pid) {
  if (!pid->inAuto) {
    return false;
  }
    //  unsigned long now = millis();
    //  unsigned long timeChange = (now - pid->lastTime);
    //  if (timeChange >= pid->SampleTime) {
    /*Compute all the working error variables*/
    float input = pid->input;
    float error = pid->setpoint - input;
    pid->ITerm += (pid->ki * error);
    if (pid->ITerm > pid->outMax)
      pid->ITerm = pid->outMax;
    else if (pid->ITerm < pid->outMin)
      pid->ITerm = pid->outMin;
    float dInput = (input - pid->lastInput);

    /*Compute PID Output*/
    float output = pid->kp * error + pid->ITerm - pid->kd * dInput;

    if (output > pid->outMax)
      output = pid->outMax;
    else if (output < pid->outMin)
      output = pid->outMin;
    pid->output = output;

    /*Remember some variables for next time*/
    pid->lastInput = input;
    //    pid->lastTime = now;
    return true;
    //  } else {
    //    return false;
    //  }
}


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/ 
/**
 * @fn    pid_setTunings
 * @brief Configure the PID controller
 *
 * @param[in] pid The pointer to the PID controller
 * @param[in] kp  The proportionnal gain
 * @param[in] Ki  The integral gain
 * @param[in] kd  The derivative gain
 */
void pid_setTunings(pid_t *pid, float kp, float ki, float kd){
  if(kp < 0 || ki < 0 || kd < 0){
    return;
  }

  pid->dispKp = kp;
  pid->dispKi = ki;
  pid->dispKd = kd;

  float sampleTimeInSec = ((float) pid->SampleTime) / 1000;
  pid->kp = kp;
  pid->ki = ki * sampleTimeInSec;
  pid->kd = kd / sampleTimeInSec;

  if(pid->controllerDirection == PID_DIRECTION_REVERSE){
    pid->kp = (0 - pid->kp);
    pid->ki = (0 - pid->ki);
    pid->kd = (0 - pid->kd);
  }
}
  
/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
/**
 * @fn    pid_setSampleTime
 * @brief Configure the PID controller sample time interval
 *
 * @param[in] pid           The pointer to the PID controller
 * @param[in] newSampleTime The new sample time to be set
 */
void pid_setSampleTime(pid_t *pid, int newSampleTime){
  if (newSampleTime > 0){
    float ratio = (float) newSampleTime / (float) pid->SampleTime;
    pid->ki *= ratio;
    pid->kd /= ratio;
    pid->SampleTime = (unsigned long) newSampleTime;
  }
}
 
/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
/**
 * @fn    pid_setOutputLimits
 * @brief Configure the high and the low limit for the PID output.
 *
 * @param[in] pid The pointer to the PID controller
 * @param[in] min The output low limit
 * @param[in] max The output high limit
 */
void pid_setOutputLimits(pid_t *pid, float min, float max){
  if(min >= max)
    return;

  pid->outMin = min;
  pid->outMax = max;

  if(pid->inAuto){
    if(pid->output > pid->outMax)
      pid->output = pid->outMax;
    else if (pid->output < pid->outMin)
      pid->output = pid->outMin;

    if (pid->ITerm > pid->outMax)
      pid->ITerm = pid->outMax;
    else if (pid->ITerm < pid->outMin)
      pid->ITerm = pid->outMin;
  }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/ 
/**
 * @fn    pid_setMode
 * @brief Configure the PID mode to manual or automatic
 *
 * @param[in] pid   The pointer to the PID controller
 * @param[in] mode  The configuration mode to set
 */
void pid_setMode(pid_t *pid, pid_mode_e mode){
    bool newAuto = (mode == PID_MODE_AUTOMATIC);
    if(newAuto == !pid->inAuto)/*we just went from manual to auto*/
      pid_initialize(pid);
    pid->inAuto = newAuto;
}
 
/* Initialize()****************************************************************
 *  does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/ 
void pid_initialize(pid_t *pid){
  pid->ITerm = pid->output;
  pid->lastInput = pid->input;
  if (pid->ITerm > pid->outMax)
    pid->ITerm = pid->outMax;
  else if (pid->ITerm < pid->outMin)
    pid->ITerm = pid->outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void pid_setControllerDirection(pid_t *pid, pid_direction_e direction) {
  if (pid->inAuto && direction != pid->controllerDirection) {
    pid->kp = (0 - pid->kp);
    pid->ki = (0 - pid->ki);
    pid->kd = (0 - pid->kd);
  }
  pid->controllerDirection = direction;
}

/**
 * @fn    pid_getKp
 * @brief Get the proportional gain of the PID controller
 *
 * @param[in] pid     Pointer to the PID controller
 * @return    dispKd  The Kp value
 */
float pid_getKp(pid_t *pid){
  return pid->dispKp;
}

/**
 * @fn    pid_getKi
 * @brief Get the integral gain of the PID controller
 *
 * @param[in] pid     Pointer to the PID controller
 * @return    dispKd  The Ki value
 */
float pid_getKi(pid_t *pid){
  return pid->dispKi;
}

/**
 * @fn    pid_getKd
 * @brief Get the derivative gain of the PID controller
 *
 * @param[in] pid     Pointer to the PID controller
 * @return    dispKd  The Kd value
 */
float pid_getKd(pid_t *pid){
  return pid->dispKd;
}

/**
 * @fn    pid_getMode
 * @brief Get the mode of the PID controller
 *
 * @param[in] pid The pointer to the PID controller
 */
pid_mode_e pid_getMode(pid_t *pid){
  return pid->inAuto ? PID_MODE_AUTOMATIC : PID_MODE_MANUAL;
}

/**
 * @fn    pid_getDirection
 * @brief Get the direction of the PID
 *
 * @param[in] pid Pointer to the PID controller
 */
pid_direction_e pid_getDirection(pid_t *pid){
  return pid->controllerDirection;
}
