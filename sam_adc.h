/*
    PLAY Embedded demos - Copyright (C) 2014-2016 Rocco Marco Guglielmi

    This file is part of PLAY Embedded demos.

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
#ifndef __SAM_ADC_H_
#define __SAM_ADC_H_

#include "ch.h"
#include "hal.h"

/*
 * Retrieve the integer part of value
 */
int32_t ftomod(float value);

/*
 * Retrieve the decimal part of value
 */
uint32_t ftodp(float value);

void adc_init(void);

float adc0_read(void);
float adc1_read(void);
#endif /* __SAM_ADC_H_ */
