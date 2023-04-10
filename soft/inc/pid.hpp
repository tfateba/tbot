/*
    TBOT - Copyright (C) 2015...2023 Theodore Ateba

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    pid.h
 * @brief   PID corrector header file.
 *
 * @addtogroup PID
 * @{
 */

#ifndef PID_H
#define PID_H

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

/*==========================================================================*/
/* Enumerations, Structures and macros.                                     */
/*==========================================================================*/

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

class Pid {

  public:
  void init(float kpVal, float kiVal, float kdVal);
  void compute(void);
  void reset(void);

  void  setSetPoint(float val) {setpoint = val;}
  void  setMeasure(float val) {measure = val;}

  float getSetPoint(void) {return setpoint;}
  float getMeasure(void) {return measure;}
  float getOutput(void) {return output;}

  private:

  // Privated attribute
  float setpoint;
  float measure;

  float kp;           /**< Proportional parameter of PID controler.    */
  float ki;           /**< Integral parameter of PID controler.        */
  float kd;           /**< Derivate parameter of PID controler.        */

  float actualError;  /**< Actual error between consigne an measured.  */
  float lastError;    /**< Last error.                                 */

  float pTerm;        /**< Proportional error.                         */
  float iTerm;        /**< Integral error.                             */
  float dTerm;        /**< Derivate error.                             */

  float output;       /**< PID value, sum of all the errors.           */
};

#endif /* PID_H */

/** @} */
