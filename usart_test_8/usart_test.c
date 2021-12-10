/* 

  USART Test for the STM32G031

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

*/

#include "stm32g0xx.h"
#include "delay.h"

volatile unsigned long SysTickCount = 0;

void __attribute__ ((interrupt, used)) SysTick_Handler(void)
{
  SysTickCount++;  
  
#ifdef xxx
  if ( SysTickCount & 1 )
    GPIOA->BSRR = GPIO_BSRR_BS3;		/* atomic set PA3 */
  else
    GPIOA->BSRR = GPIO_BSRR_BR3;		/* atomic clr PA3 */
#endif
}

void usart1_write_byte(uint8_t b)
{
  while ( (USART1->ISR & USART_ISR_TXE_TXFNF) == 0 )
      ;
  USART1->TDR = b;
  while ( (USART1->ISR & USART_ISR_TC) == 0 )
      ;
}


int main()
{
  
  SystemCoreClockUpdate();
  
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

    
  RCC->APBENR2 |= RCC_APBENR2_USART1EN;
  RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;
  __NOP();
  __NOP();
  
  RCC->CCIPR &= ~RCC_CCIPR_USART1SEL;		// clear clock selection
  RCC->CCIPR |= RCC_CCIPR_USART1SEL_0;	// select system clock --> 16 MHz

  SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_RMP;       // remap to A9
  SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA12_RMP;      // remap to A10

  GPIOA->MODER &= ~GPIO_MODER_MODE9;  // clear mode  
  GPIOA->MODER |= GPIO_MODER_MODE9_1;  // enable alternate functions
  GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL9;		// clear alternate function
  GPIOA->AFR[1] |= 1 << GPIO_AFRH_AFSEL9_Pos ;		// AF1: USART pins

  GPIOA->MODER &= ~GPIO_MODER_MODE10;  // clear mode  
  GPIOA->MODER |= GPIO_MODER_MODE10_1;  // enable alternate functions
  GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL10;		// clear alternate function
  GPIOA->AFR[1] |= 1 << GPIO_AFRH_AFSEL10_Pos ;		// AF1: USART pins


  USART1->BRR = 278; 	/* 16000000/57600= 277.7 with 16x oversampling */ ;
  //USART1->BRR = 32000000U / 9600;
  USART1->CR1 = USART_CR1_TE | USART_CR1_RE;	/* default 8-N-1 configuration, transmit & receive enable */
  USART1->CR2 = 0;
  USART1->CR3 = 0;
  USART1->PRESC = 0;
  USART1->CR1 |= USART_CR1_UE;	/* enable usart */
  

  SysTick->LOAD = 16000*500 - 1;        // Blink with 1 Hz 
  SysTick->VAL = 0;
  SysTick->CTRL = 7;   /* enable, generate interrupt (SysTick_Handler), do not divide by 2 */
    
  for(;;)
  {
    GPIOA->BSRR = GPIO_BSRR_BS3;		/* atomic set PA3 */
    usart1_write_byte('a');
    delay_micro_seconds(100000);
    GPIOA->BSRR = GPIO_BSRR_BR3;		/* atomic clr PA3 */
    usart1_write_byte('b');
    usart1_write_byte('\n');
    delay_micro_seconds(100000);
    
    
  }
}
