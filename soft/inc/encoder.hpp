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
 * @file    encoder.h
 * @brief   Encoder driver header file.
 *
 * @addtogroup ENCODER
 * @{
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "hal.h"
#include "types.hpp"

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

/*==========================================================================*/
/* Enumerations, Structures and macros.                                     */
/*==========================================================================*/

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

class Encoder {
  public:
  
  //bool readLeftStateA(void);
  //bool readLeftStateB(void);

  //bool readRightStateA(void);
  //bool readRightStateB(void);
  
  void incrementCounter(void) {counter++;}
  void decrementCounter(void) {counter--;}
  long getCounter(void) {return counter;}

  void setStateA(bool state) {stateA = state;}
  void setStateB(bool state) {stateB = state;}

  bool getStateA(void);
  bool getStateB(void);

  void init(ENCODERConfig cfg);
  //void getWheelVelocity(void);

  private:
  bool          stateA;
  bool          stateB;
  volatile long counter;
  ENCODERConfig config;
};

#endif /* ENCODER_H */

/** @} */
