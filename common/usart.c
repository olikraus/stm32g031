
/*

*/


#include <stddef.h>
#include <usart.h>
#include <utoa.h>


/*
void __attribute__ ((interrupt)) UART0_Handler(void)
{
  if ( (usart0_struct_ptr->usart->STAT & RXRDY) != 0 )
  {
    rb_add(&(usart0_struct_ptr->rb), usart0_struct_ptr->usart->RXDAT);
  }
}
*/

void usart_init(usart_t *usart, uint32_t baud, int is_rx)
{
  USART_TypeDef *u = usart->usart;
 
  u->BRR = (SystemCoreClock+baud/2)/baud; 	/* assume 16x oversampling */ ;
  //u->CR1 = USART_CR1_TE ;						/* default 8-N-1 configuration, transmit enable */
  u->CR1 = USART_CR1_TE | USART_CR1_RE;	/* default 8-N-1 configuration, transmit & receive enable */
  
  //u->BRR = 138; 	/* 16000000/115200 with 16x oversampling */ ;
  u->BRR = 278; 	/* 16000000/57600= 277.7 with 16x oversampling */ ;
  //u->BRR = 32000000U / 9600;
  u->CR2 = 0;
  u->CR3 = 0;
  u->PRESC = 0;
  //u->CR1 |= USART_CR1_UE;	/* enable usart */
  
  
  if ( is_rx )
  {
    u->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE; /* receive enable  */
  }
  u->CR1 |= USART_CR1_UE;	/* enable usart */
  
}



void usart_write_byte(usart_t *usart, uint8_t data)
{
  /*
  while( (usart->usart->ISR & USART_ISR_TC) == 0 )
    ;
  usart->usart->TDR = data;
  */
  
  while ( (usart->usart->ISR & USART_ISR_TXE_TXFNF) == 0 )
      ;
  usart->usart->TDR = data;
  while ( (usart->usart->ISR & USART_ISR_TC) == 0 )
      ;
  
  
}

void usart_write_bits(usart_t *usart, uint32_t bits, int cnt)
{
  uint32_t mask = 1<<(cnt-1);
  while( cnt > 0 )
  {
    if ( bits & mask )
    {
      usart_write_byte(usart, '1');
    }
    else
    {
      usart_write_byte(usart, '0');
    }
    mask >>= 1;
    cnt--;
  }
}

void usart_write_string(usart_t *usart, const char *s)
{
  if ( s == NULL )
    return;
  while( *s != '\0' )
    usart_write_byte(usart, *s++);
}

void usart_write_u16(usart_t *usart, uint16_t v)
{
  usart_write_string(usart, u16toa(v));
}

void usart_write_u32(usart_t *usart, uint32_t v)
{
  usart_write_string(usart, u32toa(v));
}

int usart_read_byte(usart_t *usart)
{
  return rb_get(&(usart->rb));
}



/*=================================================*/
/* USART1 */


usart_t usart1_data;

void usart1_init(uint32_t baud, uint8_t *rx_buf, uint16_t rx_len)
{
  rb_init(&(usart1_data.rb), rx_buf, rx_len);
  

  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;		/* Enable clock for GPIO Port A */
  RCC->APBENR2 |= RCC_APBENR2_USART1EN;
  RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;
  __NOP();
  __NOP();


  //RCC->APBRSTR2 |= RCC_APBRSTR2_USART1RST;
  __NOP();								/* let us wait for some time */
  __NOP();								/* let us wait for some time */
  //RCC->APBRSTR2 &= ~RCC_APBRSTR2_USART1RST;
  __NOP();								/* let us wait for some time */
  __NOP();								/* let us wait for some time */

  /*
  RCC->APBENR2 |= RCC_APBENR2_USART1EN;
  RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;
  __NOP();
  __NOP();
*/
  
  RCC->CCIPR &= ~RCC_CCIPR_USART1SEL;		// clear clock selection
  //RCC->CCIPR |= RCC_CCIPR_USART1SEL_0;	// select system clock --> 16 MHz
  RCC->CCIPR |= RCC_CCIPR_USART1SEL_1;	// HSI16 --> 16 MHz

  RCC->CCIPR &= ~RCC_CCIPR_USART1SEL;		// clear clock selection
  RCC->CCIPR |= RCC_CCIPR_USART1SEL_0;	// select system clock --> 16 MHz

  SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_RMP;       // remap to A9 (TX)
  SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA12_RMP;      // remap to A10

  GPIOA->MODER &= ~GPIO_MODER_MODE9;  // clear mode  
  GPIOA->MODER |= GPIO_MODER_MODE9_1;  // enable alternate functions  
  //GPIOA->OTYPER &= ~GPIO_OTYPER_OT9;	/* no Push/Pull for PA9 */
  //GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED9;	/* low speed for PA9 */
  //GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD9;	/* no pullup/pulldown for PA9 */
  
  GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL9;		// clear alternate function
  GPIOA->AFR[1] |= 1 << GPIO_AFRH_AFSEL9_Pos ;		// AF1: USART pins
  

  GPIOA->MODER &= ~GPIO_MODER_MODE10;  // clear mode  
  GPIOA->MODER |= GPIO_MODER_MODE10_1;  // enable alternate functions
  GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL10;		// clear alternate function
  GPIOA->AFR[1] |= 1 << GPIO_AFRH_AFSEL10_Pos ;		// AF1: USART pins
  
  usart1_data.usart = USART1;
  
  usart_init(&usart1_data, baud, rx_len != 0);
  
  if (  rx_len != 0 ) 
    NVIC_EnableIRQ(USART1_IRQn);

  
}

void usart1_write_byte(uint8_t byte)
{
  usart_write_byte(&usart1_data, byte);
}

void usart1_write_bits(uint32_t bits, int cnt)
{
  uint32_t mask = 1<<(cnt-1);
  while( cnt > 0 )
  {
    if ( bits & mask )
    {
      usart1_write_byte('1');
    }
    else
    {
      usart1_write_byte('0');
    }
    mask >>= 1;
    cnt--;
  }
}


void usart1_write_string(const char *s)
{
  usart_write_string(&usart1_data, s);
}

void usart1_write_u16(uint16_t v)
{
  usart_write_u16(&usart1_data, v);
}

void usart1_write_u32(uint32_t v)
{
  usart_write_u32(&usart1_data, v);
}

int usart1_read_byte(void)
{
  return usart_read_byte(&usart1_data);
}

void __attribute__ ((interrupt, used)) USART1_IRQHandler()
{
  if ( (usart1_data.usart->ISR & USART_CR1_RXNEIE_RXFNEIE) != 0 )
  {
    rb_add(&(usart1_data.rb), usart1_data.usart->RDR);
  }
}
