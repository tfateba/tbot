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

/* TODO: Rename the PIDDriver to PIDController. */

/**
 * @brief   PID driver type definition.
 */
// typedef struct {

// float consigne;
//   float measure;

//   float kp;          /**< Proportional parameter of PID controler.    */
//   float ki;          /**< Integral parameter of PID controler.        */
//   float kd;          /**< Derivate parameter of PID controler.        */

//   float actuError;   /**< Actual error between consigne an measured.  */
//   float lastError;   /**< Last error.                                 */

//   float pTerm;       /**< Proportional error.                         */
//   float iTerm;       /**< Integral error.                             */
//  float dTerm;       /**< Derivate error.                             */
//
//  float output;      /**< PID value, sum of all the errors.           */
//} PIDDriver;

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

class Pid {

  public:
  void init(float kpval, float kival, float kdval);
  void compute(void);
  void reset(void);

  void  setSetPoint(float val) {setpoint = val;}
  void  setMeasure(float val) {measure = val;}
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
  
  // Private methods:
  float getSetpoint(void) {return setpoint;}
  
  float getMeasure(void) {return measure;}
  
  float getKp(void) {return kp;}
  void  setKp(float val) {kp = val;}
  float getKi(void) {return ki;}
  void  setKi(float val) {ki = val;}
  float getKd(void) {return kd;}
  void  setKd(float val) {kd = val;}
  float getActualError(void) {return actualError;}
  void  setActualError(float val) {actualError = val;}
  float getLastError(void) {return lastError;}
  void  setLastError(float val) {lastError = val;}
  float getPTerm(void) {return pTerm;}
  void  setPTerm(float val) {pTerm = val;}
  float getITerm(void) {return iTerm;}
  void  setITerm(float val) {iTerm = val;}
  float getDTerm(void) {return dTerm;}
  void  setDTerm(float val) {dTerm = val;}
  
  void  setOutput(float val) {output = val;}
};

#endif /* PID_H */

/** @} */
