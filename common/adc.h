/*

  adc.c

  BSD 3-Clause License

  Copyright (c) 2023, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef _ADC_H
#define _ADC_H

#include <stdint.h>

/*
  generic ADC init for the STM32G0xx
    - setup RCC for ADC
    - Do ADC calibration
    - Wakeup VREFINT
    - Wakeup temperature sensor
    
    Warning: This function will call delay_micro_seconds()
      --> This requires proper SysTick init
*/
void adc_init(void);


/*
  execute single converion and return the value from the specified channel 

  ch 0..7 ==  ADC_IN0..7  == PA0..PA7
  ch 8 == ADC_IN8 == PB0
  ch 9 == ADC_IN9 == PB1
  ch 10 == ADC_IN10 == PB2
  ch 11 == ADC_IN11 == PB7 / PB10
  ch 12: temperture sensor
  ch 13: vrefint
  ch 14: vbat (not available)
  ch 15 == ADC_IN15 == PB11 / PA11 [PA9]
  ch 16 == ADC_IN16 == PB12 / PA12 [PA10]
  ch 17 == ADC_IN17 == PA13
  ch 18 == ADC_IN18 == PA14 (BOOT0)
  
  GPIO pins should be in analog mode (which is reset default)
*/
uint16_t adc_get_value(uint8_t ch);


#endif /* _ADC_H */

