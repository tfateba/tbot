
/**
 *
 * @file    ip_pid.h
 *
 * @brief   PID corrector header file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    13 July 2016
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

#ifndef IP_PID_H
#define IP_PID_H

/*==========================================================================*/
/* Structures of the PID controller.                                        */
/*==========================================================================*/

/**
 * @brief   PID driver data structure.
 */
struct PIDDriver {
  uint8_t id;
  float   kp;
  float   ki;
  float   kd;
  float   result;
};

/**
 * @brief   PID driver type definition.
 */
typedef struct PIDDriver PIDDriver;

/*==========================================================================*/
/* Functions prototypes.                                                    */
/*==========================================================================*/

void pid_init(float kpval, float kival, float kdval);
float pid(float pitch, float restAngle, float offset, float turning);
void pid_reset_parameters(void);

#endif /* IP_PID_H */

