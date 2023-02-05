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
 * @file    kalman.h
 * @brief   Kalman filter header file.
 *
 * @addtogroup KALMAN
 * @{
 */

#ifndef KALMAN_H
#define KALMAN_H

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

/*==========================================================================*/
/* Enumerations, Structures and macros.                                     */
/*==========================================================================*/

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

class Kalman {

    public:
    void  init(void);
    float getAngle(float newAngle, float newRate, float dt);
    void  setAngle(float newAngle);
    float getRate(void);
    void  setQangle(float newQ_angle);
    void  setQbias(float newQ_bias);
    void  setRmeasure(float newR_measure);
    float getQangle(void);
    float getQbias(void);
    float getRmeasure(void);
};

#endif /* KALMAN_H */

/** @} */
