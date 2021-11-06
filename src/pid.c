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
#include "pid.h"

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
void pidInit(PIDDriver *pidp, float kpval, float kival, float kdval) {

  pidp->kp = kpval;
  pidp->ki = kival;
  pidp->kd = kdval;

  pidp->pTerm      = 0;
  pidp->iTerm      = 0;
  pidp->dTerm      = 0;

  pidp->lastError  = 0;
  pidp->actuError  = 0;
}

/**
 * @brief   Compute the pid error.
 *
 * @param[in] pidp pointer to the pid that need to compute data
 */
void pidCompute(PIDDriver *pidp) {

  /* Update PID values. */
  pidp->actuError =  (pidp->consigne - pidp->measure);
  pidp->pTerm     =  pidp->kp * pidp->actuError;
  pidp->iTerm     += pidp->ki * pidp->actuError;
  pidp->dTerm     =  pidp->kd * (pidp->actuError - pidp->lastError);
  pidp->lastError =  pidp->actuError;
  pidp->output    =  pidp->pTerm + pidp->iTerm + pidp->dTerm;
}

/**
 * @brief   Reset the PID parameters.
 */
void pidResetParameters(PIDDriver *pidp) {
  pidp->pTerm      = 0;
  pidp->iTerm      = 0;
  pidp->dTerm      = 0;
  pidp->lastError  = 0;
}

/** @} */
