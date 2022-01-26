/* 

  U8x8 test with the STM32G031

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


  Default clock is HSI16 (16 MHz)
    
  Ensure that -DUSER_VECT_TAB_ADDRESS is set during compilation, otherwise
  interrupts will not work after the "go" commant of the flashware USART upload.
  
  Problem is, that SCB->VTOR doesn't point to FLASH_BASE, so manually
  assigning SCB->VTOR = FLASH_BASE;	will also fix this.

  If -DUSER_VECT_TAB_ADDRESS is defined, then the SCB->VTOR is set in
  SystemInit(void) which is called by the reset handler (.s file).
  
  SCL @PC14
  SDA @PF2

*/

#include "stm32g0xx.h"
#include "delay.h"
#include "u8x8.h"

uint8_t u8x8_gpio_and_delay_stm32g0(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  switch(msg)
  {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
      /* only support for software I2C*/
    
      RCC->IOPENR |= RCC_IOPENR_GPIOAEN;		/* Enable clock for GPIO Port A */
      RCC->IOPENR |= RCC_IOPENR_GPIOBEN;		/* Enable clock for GPIO Port B */
      RCC->IOPENR |= RCC_IOPENR_GPIOCEN;		/* Enable clock for GPIO Port C */
      RCC->IOPENR |= RCC_IOPENR_GPIOFEN;		/* Enable clock for GPIO Port F */
      __NOP();
      __NOP();
    
      GPIOA->AFR[0] = 0;
      GPIOA->AFR[1] = 0;
      GPIOB->AFR[0] = 0;
      GPIOB->AFR[1] = 0;
    
      /* clear all alternative functions on Port C & F */
      //GPIOC->AFR[0] = 0;
      //GPIOC->AFR[1] = 0;
      //GPIOF->AFR[0] = 0;
      //GPIOF->AFR[1] = 0;

      GPIOB->MODER &= ~GPIO_MODER_MODE9;	/* clear mode for PB9 */
      //GPIOB->MODER |= GPIO_MODER_MODE9_0;	/* Output mode for PB9 */
      GPIOB->OTYPER &= ~GPIO_OTYPER_OT9;	/* no open drain for PB9 */
      GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED9;	/* low speed for PB9 */
      GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD9;	/* no pullup/pulldown for PB9 */
      GPIOB->PUPDR |= GPIO_PUPDR_PUPD9_0;	/* pullup */
      //GPIOB->BSRR = GPIO_BSRR_BS_9;		/* atomic set PB9 */

      //GPIOC->MODER &= ~GPIO_MODER_MODE14;	/* clear mode for PC14 */
      //GPIOC->OTYPER &= ~GPIO_OTYPER_OT14;	/* no open drain for PC14 */
      //GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED14;	/* low speed for PC14 */
      //GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD14;	/* no pullup/pulldown for PC14 */

      GPIOA->MODER &= ~GPIO_MODER_MODE2;	/* clear mode for PA2 */
      //GPIOA->MODER |= GPIO_MODER_MODE2_0;	/* Output mode for PA2 */
      GPIOA->OTYPER &= ~GPIO_OTYPER_OT2;	/* no open drain for PA2 */
      GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED2;	/* low speed for PA2 */
      GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD2;	/* no pullup/pulldown for PA2 */
      GPIOB->PUPDR |= GPIO_PUPDR_PUPD2_0;	/* pullup */
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
      delay_micro_seconds(10);
      break;
    
    case U8X8_MSG_GPIO_I2C_CLOCK:
      if ( arg_int == 0 )
      {
	GPIOB->MODER &= ~GPIO_MODER_MODE9;	/* clear mode for PB9 */
	GPIOB->MODER |= GPIO_MODER_MODE9_0;	/* Output mode for PB9 */
	GPIOB->BSRR = GPIO_BSRR_BR9;		/* atomic clr PB9 */
      }
      else
      {
	//GPIOB->MODER &= ~GPIO_MODER_MODE9;	/* clear mode for PB9 */
	//GPIOB->MODER |= GPIO_MODER_MODE9_0;	/* Output mode for PB9 */
	//GPIOB->BSRR = GPIO_BSRR_BS9;		/* atomic set PB9 */
	GPIOB->MODER &= ~GPIO_MODER_MODE9;	/* clear mode for PB9: input mode */
      }
      break;
    case U8X8_MSG_GPIO_I2C_DATA:
      if ( arg_int == 0 )
      {
	GPIOA->MODER &= ~GPIO_MODER_MODE2;	/* clear mode for PF2 */
	GPIOA->MODER |= GPIO_MODER_MODE2_0;	/* Output mode for PF2 */
	GPIOA->BSRR = GPIO_BSRR_BR2;		/* atomic clr PF2 */
      }
      else
      {
	//GPIOA->MODER &= ~GPIO_MODER_MODE2;	/* clear mode for PF2 */
	//GPIOA->MODER |= GPIO_MODER_MODE2_0;	/* Output mode for PF2 */
	//GPIOA->BSRR = GPIO_BSRR_BS2;		/* atomic set PF2 */
	// input mode
	GPIOA->MODER &= ~GPIO_MODER_MODE2;	/* clear mode for PF2: input mode */
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


u8x8_t u8x8;
volatile unsigned long SysTickCount = 0;

void __attribute__ ((interrupt, used)) SysTick_Handler(void)
{
  SysTickCount++;  
  
  if ( SysTickCount & 1 )
    GPIOA->BSRR = GPIO_BSRR_BS13;		/* atomic set PA13 */
  else
    GPIOA->BSRR = GPIO_BSRR_BR13;		/* atomic clr PA13 */
}

int main()
{
  SystemCoreClockUpdate();
  
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;		/* Enable clock for GPIO Port A */
  __NOP();
  __NOP();

  
  //GPIOA->AFR[0] &= ~(0xf << (3*4));       /* clear alternative function */
  //GPIOA->AFR[0] = 0;       /* clear alternative function */
  
  GPIOA->MODER &= ~GPIO_MODER_MODE13;	/* clear mode for PA13 */
  GPIOA->MODER |= GPIO_MODER_MODE13_0;	/* Output mode for PA13 */
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT13;	/* no Push/Pull for PA13 */
  GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED13;	/* low speed for PA13 */
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD13;	/* no pullup/pulldown for PA13 */
  GPIOA->BSRR = GPIO_BSRR_BR13;		/* atomic clr PA13 */
  
  if ( (FLASH->OPTR & FLASH_OPTR_NRST_MODE_1) != 0 && (FLASH->OPTR & FLASH_OPTR_NRST_MODE_0) == 0 )
  {
    SysTick->LOAD = 16000*500 - 1;        // Blink with 1 Hz 
  }
  else
  {
    SysTick->LOAD = 16000*100 - 1;        // Blink with 5 Hz 
  }
  SysTick->VAL = 0;
  SysTick->CTRL = 7;   /* enable, generate interrupt (SysTick_Handler), do not divide by 2 */

  
  u8x8_Setup(&u8x8, u8x8_d_ssd1306_128x64_noname, u8x8_cad_ssd13xx_i2c, u8x8_byte_sw_i2c, u8x8_gpio_and_delay_stm32g0);
  u8x8_InitDisplay(&u8x8);
  u8x8_ClearDisplay(&u8x8);
  u8x8_SetPowerSave(&u8x8, 0);
  u8x8_SetFont(&u8x8, u8x8_font_amstrad_cpc_extended_r);  
  for(;;)
    u8x8_DrawString(&u8x8, 0,0, "Hello World!");
  
}
