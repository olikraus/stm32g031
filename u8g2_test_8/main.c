/* 

  U8g2 test with the STM32G031

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
  
  SCL @PA2
  SDA @PA1

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
    
    case U8X8_MSG_GPIO_I2C_CLOCK:
      
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
    case U8X8_MSG_GPIO_I2C_DATA:
      
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


u8g2_t u8g2;
volatile unsigned long SysTickCount = 0;


void __attribute__ ((interrupt, used)) SysTick_Handler(void)
{
  SysTickCount++;  
  
  if ( SysTickCount & 1 )
    GPIOA->BSRR = GPIO_BSRR_BS3;		/* atomic set PA3 */
  else
    GPIOA->BSRR = GPIO_BSRR_BR3;		/* atomic clr PA3 */
}


void initDisplay(void)
{
  /* setup display */
  u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_sw_i2c, u8x8_gpio_and_delay_stm32g0);
  u8g2_InitDisplay(&u8g2);
  u8g2_SetPowerSave(&u8g2, 0);
  u8g2_SetFont(&u8g2, u8g2_font_6x12_tf);
  u8g2_ClearBuffer(&u8g2);
  u8g2_DrawStr(&u8g2, 0,12, "STM32G031");
  u8g2_DrawStr(&u8g2, 0,24, u8x8_u8toa(SystemCoreClock/1000000, 2));
  u8g2_DrawStr(&u8g2, 20,24, "MHz");
  u8g2_SendBuffer(&u8g2);
}

int main()
{
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;		/* Enable clock for GPIO Port A */
  __NOP();
  __NOP();

  
  GPIOA->AFR[0] &= ~(0xf << (3*4));       /* clear alternative function */
  //GPIOA->AFR[0] = 0;       /* clear alternative function */
  
  GPIOA->MODER &= ~GPIO_MODER_MODE3;	/* clear mode for PA3 */
  GPIOA->MODER |= GPIO_MODER_MODE3_0;	/* Output mode for PA3 */
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT3;	/* no Push/Pull for PA3 */
  GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED3;	/* low speed for PA3 */
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD3;	/* no pullup/pulldown for PA3 */
  GPIOA->BSRR = GPIO_BSRR_BR3;		/* atomic clr PA3 */
  

  SysTick->LOAD = 16000*500 - 1;        // Blink with 1 Hz 
  SysTick->VAL = 0;
  SysTick->CTRL = 7;   /* enable, generate interrupt (SysTick_Handler), do not divide by 2 */
  
  initDisplay();
  
  for(;;)
    ;
}
