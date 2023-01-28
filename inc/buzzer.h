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
 * @file    buzzer.h
 * @brief   Buzzer driver header file.
 *
 * @addtogroup BUZZER
 * @{
 */

#ifndef BUZZER_H
#define BUZZER_H

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

/*==========================================================================*/
/* Enumerations, Structures and macros.                                     */
/*==========================================================================*/

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

void buzzerInit(void);
void buzzerSound(void);
void buzzerStopSound(void);

#ifdef __cplusplus
}
#endif

#endif /* BUZZER_H */

/** @} */
