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
 * @file    led.hpp
 * @brief   led class header file.
 */

#ifndef LED_H
#define LED_H

#include "hal.h"
#include "types.hpp"

/**
 * @brief Small LED C++ clas
*/
class Led {

public:
  void init(ioportid_t port, iopadid_t pad);
  void toggle(void);
  void on(void);
  void off(void);

private:
  LED led;
};

#endif /* LED_H */