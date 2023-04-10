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
 * @file    pid.c
 * @brief   PID corrector source file.
 *
 * @addtogroup PID
 * @{
 */

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

/* Standard files. */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/* Project files. */
#include "pid.hpp"

#ifdef __cplusplus
extern "C" {
#endif

/*==========================================================================*/
/* Global variables.                                                        */
/*==========================================================================*/

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
void Pid::init(float kpval, float kival, float kdval) {

  kp = kpval;
  ki = kival;
  kd = kdval;

  pTerm = 0;
  iTerm = 0;
  dTerm = 0;

  actualError = 0;
  lastError   = 0;
}

/**
 * @brief   Compute the pid error.
 *
 * @param[in] pidp pointer to the pid that need to compute data
 */
void Pid::compute(void) {

  /* Update PID values. */
  actualError = (setpoint - measure);

  pTerm = (kp * actualError);
  
  iTerm = (iTerm +  (iTerm * actualError));

  dTerm = (dTerm * (actualError - lastError));

  lastError = actualError;

  output = (pTerm + iTerm + dTerm);
}

/**
 * @brief   Reset the PID parameters.
 */
void Pid::reset(void) {

  kd = 0;
  ki = 0;
  kd = 0;

  pTerm = 0;
  iTerm = 0;
  dTerm = 0;

  actualError = 0;
  lastError   = 0;
}

#ifdef __cplusplus
}
#endif

/** @} */
