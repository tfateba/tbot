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
 * @file    asserv.h
 * @brief   Asservissement header file.
 *
 * @addtogroup ASSERV
 * @{
 */

#ifndef ASSERV_H
#define ASSERV_H

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

/* Project files. */
#include "main.h"

/*==========================================================================*/
/* Enumerations, Structures and macros.                                     */
/*==========================================================================*/

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

/* @todo    Add Constructor and destructor for all cpp classes. */
class Asserv {

    public:
    void startAsserv(Tbot *robot);
};

#endif /* ASSERV_H */

/** @} */
