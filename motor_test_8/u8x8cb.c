/* 

  u8x8cb.c

  Copyright (c) 2021, olikraus@gmail.com
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

#include "stm32g0xx.h"
#include "delay.h"
#include "u8g2.h"


uint8_t u8x8_gpio_and_delay_stm32g0(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  switch(msg)
  {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
      /* only support for software I2C*/
    
      RCC->IOPENR |= RCC_IOPENR_GPIOAEN;		/* Enable clock for GPIO Port A */
      __NOP();
      __NOP();

      GPIOA->MODER &= ~GPIO_MODER_MODE1;	/* clear mode for PA1 */
      //GPIOA->MODER |= GPIO_MODER_MODE1_0;	/* Output mode for PA1 */
      GPIOA->OTYPER &= ~GPIO_OTYPER_OT1;	/* no open drain for PA1 */
      GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED1;	/* low speed for PA1 */
      GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD1;	/* no pullup/pulldown for PA1 */
      //GPIOA->BSRR = GPIO_BSRR_BS_1;		/* atomic set PA1 */
    
      GPIOA->MODER &= ~GPIO_MODER_MODE2;	/* clear mode for PA2 */
      //GPIOA->MODER |= GPIO_MODER_MODE2_0;	/* Output mode for PA2 */
      GPIOA->OTYPER &= ~GPIO_OTYPER_OT2;	/* no open drain for PA2 */
      GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED2;	/* low speed for PA2 */
      GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD2;	/* no pullup/pulldown for PA2 */
      //GPIOA->BSRR = GPIO_BSRR_BS_2;		/* atomic set PA2 */
        
      break;
    case U8X8_MSG_DELAY_NANO:
      /* not required for SW I2C */
      break;
    
    case U8X8_MSG_DELAY_10MICRO:
      /* not used at the moment */
      break;
    
    case U8X8_MSG_DELAY_100NANO:
      /* not used at the moment */
      break;
   
    case U8X8_MSG_DELAY_MILLI:
      delay_micro_seconds(arg_int*1000UL);
      break;
    case U8X8_MSG_DELAY_I2C:
      /* arg_int is 1 or 4: 100KHz (5us) or 400KHz (1.25us) */
      delay_micro_seconds(arg_int<=2?5:1);
      break;
    
    case U8X8_MSG_GPIO_I2C_DATA:
      
      if ( arg_int == 0 )
      {
	GPIOA->MODER &= ~GPIO_MODER_MODE1;	/* clear mode for PA1 */
	GPIOA->MODER |= GPIO_MODER_MODE1_0;	/* Output mode for PA1 */
	GPIOA->BSRR = GPIO_BSRR_BR1;		/* atomic clr PA1 */
      }
      else
      {
	//GPIOA->BSRR = GPIO_BSRR_BS_9;		/* atomic set PA1 */
	GPIOA->MODER &= ~GPIO_MODER_MODE1;	/* clear mode for PA1: input mode */
      }
      break;
    case U8X8_MSG_GPIO_I2C_CLOCK:
      
      if ( arg_int == 0 )
      {
	GPIOA->MODER &= ~GPIO_MODER_MODE2;	/* clear mode for PA2 */
	GPIOA->MODER |= GPIO_MODER_MODE2_0;	/* Output mode for PA2 */
	GPIOA->BSRR = GPIO_BSRR_BR2;		/* atomic clr PA2 */
      }
      else
      {
	//GPIOA->BSRR = GPIO_BSRR_BS_2;		/* atomic set PA2 */
	// input mode
	GPIOA->MODER &= ~GPIO_MODER_MODE2;	/* clear mode for PA10: input mode */
      }
      break;
/*
    case U8X8_MSG_GPIO_MENU_SELECT:
      u8x8_SetGPIOResult(u8x8, Chip_GPIO_GetPinState(LPC_GPIO, KEY_SELECT_PORT, KEY_SELECT_PIN));
      break;
    case U8X8_MSG_GPIO_MENU_NEXT:
      u8x8_SetGPIOResult(u8x8, Chip_GPIO_GetPinState(LPC_GPIO, KEY_NEXT_PORT, KEY_NEXT_PIN));
      break;
    case U8X8_MSG_GPIO_MENU_PREV:
      u8x8_SetGPIOResult(u8x8, Chip_GPIO_GetPinState(LPC_GPIO, KEY_PREV_PORT, KEY_PREV_PIN));
      break;
    
    case U8X8_MSG_GPIO_MENU_HOME:
      u8x8_SetGPIOResult(u8x8, Chip_GPIO_GetPinState(LPC_GPIO, KEY_HOME_PORT, KEY_HOME_PIN));
      break;
*/
    default:
      u8x8_SetGPIOResult(u8x8, 1);
      break;
  }
  return 1;
}
