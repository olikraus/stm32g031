/* 
  LED blink project for the STM32G031
  
  Default clock is HSI16 (16 MHz)
  
  
  Ensure that -DUSER_VECT_TAB_ADDRESS is set during compilation, otherwise
  interrupts will not work after the "go" commant of the flashware USART upload.
  
  Problem is, that SCB->VTOR doesn't point to FLASH_BASE, so manually
  assigning SCB->VTOR = FLASH_BASE;	will also fix this.

  If -DUSER_VECT_TAB_ADDRESS is defined, then the SCB->VTOR is set in
  SystemInit(void) which is called by the reset handler (.s file).

*/

#include "stm32g0xx.h"

volatile unsigned long SysTickCount = 0;

void __attribute__ ((interrupt, used)) SysTick_Handler(void)
{
  SysTickCount++;  
  
  if ( SysTickCount & 1 )
    GPIOA->BSRR = GPIO_BSRR_BS3;		/* atomic set PA13 */
  else
    GPIOA->BSRR = GPIO_BSRR_BR3;		/* atomic clr PA13 */
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
  
  SysTick->CTRL = 0;

  if ( (FLASH->OPTR & FLASH_OPTR_nBOOT_SEL) == 0 )
  {
    /* nBOOT_SEL is already cleared..., do nothing */
    
    SysTick->LOAD = 1600*500 - 1;        // Blink with 10 Hz 
    SysTick->VAL = 0;
    SysTick->CTRL = 7;   /* enable, generate interrupt (SysTick_Handler), do not divide by 2 */  

    for(;;)
      ;
    
  }

  /* Clear the LOCK bit in FLASH->CR (precondition for option byte flash) */
  FLASH->KEYR = 0x45670123;
  FLASH->KEYR = 0xCDEF89AB;
  /* Clear the OPTLOCK bit in FLASH->CR */
  FLASH->OPTKEYR = 0x08192A3B;
  FLASH->OPTKEYR = 0x4C5D6E7F;
  /* Enable legacy mode (BOOT0 bit defined by BOOT0 pin) */
  /* by clearing the nBOOT_SELection bit */
  FLASH->OPTR &= ~FLASH_OPTR_nBOOT_SEL;
  //FLASH->OPTR |= FLASH_OPTR_nBOOT_SEL;
  /* check if there is any flash operation */
  while( (FLASH->SR & FLASH_SR_BSY1) != 0 )
    ;
  /* start the option byte flash */
  FLASH->CR |= FLASH_CR_OPTSTRT;
  /* wait until flashing is done */
  while( (FLASH->SR & FLASH_SR_BSY1) != 0 )
    ;  

  /* do a busy delay, for about one second, again check for the BSY1 flag to avoid compiler loop optimization */
  for( unsigned long i = 0; i < 4000000; i++ )
    if ( (FLASH->SR & FLASH_SR_BSY1) != 0 )
      break;
  
  /* load the new value and do a system reset */
  FLASH->CR |= FLASH_CR_OBL_LAUNCH;
  /* we will never arrive here */
  for(;;)
    ;

}
