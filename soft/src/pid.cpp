/*
    TBOT - Copyright (C) 2015...2021 Theodore Ateba

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

  setKd(kpval);
  setKi(kival);
  setKd(kdval);

  setPTerm(0);
  setITerm(0);
  setDTerm(0);

  setLastError(0);
  setActualError(0);
}

/**
 * @brief   Compute the pid error.
 *
 * @param[in] pidp pointer to the pid that need to compute data
 */
void Pid::compute(void) {

  /* Update PID values. */
  setActualError(getSetpoint() - getMeasure());
  setPTerm(getKp() * getActualError());
  
  setITerm(getITerm() +  (getITerm() * getActualError()));

  setDTerm(getDTerm() * (getActualError() - getLastError()));

  setLastError(getActualError());

  
  setOutput(getPTerm() + getITerm() + getDTerm());
}

/**
 * @brief   Reset the PID parameters.
 */
void Pid::reset(void) {

  setKd(0);
  setKi(0);
  setKd(0);

  setPTerm(0);
  setITerm(0);
  setDTerm(0);

  setLastError(0);
  setActualError(0);
}

#ifdef __cplusplus
}
#endif

/** @} */
