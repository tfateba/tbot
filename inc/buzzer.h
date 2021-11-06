
/**
 *
 * @file    ip_buzzer.h
 *
 * @brief   Buzzer driver header file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    22 August 2017
 *
 */

/*
    IP - Copyright (C) 2015..2018 Theodore Ateba

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

#ifndef IP_BUZZER_H
#define IP_BUZZER_H

/*==========================================================================*/
/* Driver macros.                                                           */
/*==========================================================================*/

/* Pin used to control the buzzer. */
//#define BUZZER_PIN_PORT   IOPORT11
//#define BUZZER_PIN        46

/*==========================================================================*/
/* Functions prototypes.                                                    */
/*==========================================================================*/

void buzzer_init(void);
void buzzer_sound_play(void);
void buzzer_sound_stop(void);

#endif /* IP_BUZZER_H */

