/*
  u8x8cb.c
  
  STM32G031
  
  SCL @PB9/PC14 (Pin 1)
  SDA @PF2/PA2 (Pin 4 aka Reset Pin)
  
*/

#include "stm32g031xx.h"
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
	GPIOB->MODER &= ~GPIO_MODER_MODE9;	/* clear mode for PB9: input mode */
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
	GPIOA->MODER &= ~GPIO_MODER_MODE2;	/* clear mode for PA2: input mode */
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
