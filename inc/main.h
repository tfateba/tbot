
/**
 *
 * @file    ip_main.h
 *
 * @brief   Main application header file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    17 November 2017
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

#ifndef IP_MAIN_H
#define IP_MAIN_H

/* Project files. */
#include "encoder.h"
#include "motor.h"
#include "mpu6050.h"
#include "pid.h"


struct ROBOTDriver {
  MPU6050Driver imu;

  PIDDriver     lpid;
  PIDDriver     rpid;

  MOTORDriver   lmotor;
  MOTORDriver   rmotor;

  ENCODERDriver lencoder;
  ENCODERDriver rencoder;
};

typedef struct ROBOTDriver ROBOTDriver;

#endif /* IP_MAIN_H */
