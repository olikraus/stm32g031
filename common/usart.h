/*
USART_TypeDef
*/



#include <stm32g0xx.h>
#include <rb.h>

#ifndef _USART_H
#define _USART_H

struct usart_struct
{
  USART_TypeDef *usart;
  rb_t rb;
};
typedef struct usart_struct usart_t;

void usart1_init(uint32_t baud, uint8_t *rx_buf, uint16_t rx_len);
void usart1_write_byte(uint8_t byte);
void usart1_write_bits(uint32_t bits, int cnt);
void usart1_write_string(const char *s);
void usart1_write_u16(uint16_t v);
void usart1_write_u32(uint32_t v);
int usart1_read_byte(void);

#endif /* _USART_H */