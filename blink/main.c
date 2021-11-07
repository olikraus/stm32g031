/* 
  LED blink project for the STM32L072 
  
  Assumes LED at Pin 18, GPIO PA8
  
  Ensure that -DUSER_VECT_TAB_ADDRESS is set during compilation, otherwise
  interrupts will not work after the "go" commant of the flashware USART upload.
  
  Problem is, that SCB->VTOR doesn't point to FLASH_BASE, so manually
  assigning SCB->VTOR = FLASH_BASE;	will also fix this.

*/

#include "stm32l0xx.h"

volatile unsigned long SysTickCount = 0;

void __attribute__ ((interrupt, used)) SysTick_Handler(void)
{
  SysTickCount++;  
  
  if ( SysTickCount & 1 )
    GPIOA->BSRR = GPIO_BSRR_BS_8;		/* atomic set PA8 */
  else
    GPIOA->BSRR = GPIO_BSRR_BR_8;		/* atomic clr PA8 */
}

int main()
{
  RCC->IOPENR |= RCC_IOPENR_IOPAEN;		/* Enable clock for GPIO Port A */
  __NOP();
  __NOP();

  GPIOA->MODER &= ~GPIO_MODER_MODE8;	/* clear mode for PA8 */
  GPIOA->MODER |= GPIO_MODER_MODE8_0;	/* Output mode for PA8 */
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT_8;	/* no Push/Pull for PA8 */
  GPIOA->OSPEEDR &= ~GPIO_OSPEEDER_OSPEED8;	/* low speed for PA8 */
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD8;	/* no pullup/pulldown for PA8 */
  GPIOA->BSRR = GPIO_BSRR_BR_8;		/* atomic clr PA8 */


  
  SysTick->LOAD = 2000*500 - 1;
  SysTick->VAL = 0;
  SysTick->CTRL = 7;   /* enable, generate interrupt (SysTick_Handler), do not divide by 2 */
    
  for(;;)
    ;
}
