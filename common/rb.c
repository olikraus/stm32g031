/*

  rb.c
  
  Ring Buffer Implementation
  
  BSD 3-Clause License

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



#include <rb.h>
#include <cmsis_compiler.h>

/*====================================================*/
/* ring buffer implementation */


/* ring buffer */

/* 
  Prototype:
    void rb_clear(rb_t *rb)

  Description:
    Clear the data in the ring buffer.

  Precondition:
    rb_init() must be called on the "rb" argument.

  Parameter:
    rb: 	Address of a (initialized) buffer data structure

*/

void rb_clear(rb_t *rb)
{
  rb->start = 0;
  rb->end = 0;
}

/* 
  Prototype:
    void rb_init(rb_t *rb, uint8_t *buf, uint16_t len)

  Description:
    Prepares a ring (first in first out) buffer. 

  Parameter:
    rb: 	Address of a (uninitialized) buffer data structure
    buf:	Memory location large enough for "len" bytes
    len:	Size of the ring buffer. 

  Example:

    rb_t usart_rx_ring_buffer;
    uint8_t usart_rx_buf[32];
    ...
    rb_init(&usart_rx_ring_buffer, usart_rx_buf, 32);

*/
void rb_init(rb_t *rb, uint8_t *buf, uint16_t len)
{
  rb->ptr = buf;
  rb->len = len;
  rb_clear(rb);
}

uint16_t rb_next_val(rb_t *rb, uint16_t val)
{
  val++;
  if ( val >= rb->len )
    val = 0;
  return val;
}

/* 
  Prototype:
    int rb_add(rb_t *rb, uint8_t data)

  Description:
    Add a byte to the ring buffer.

  Precondition:
    rb_init() must be called on the "rb" argument.

  Parameter:
    rb: 	Address of a (initialized) buffer data structure
    data:	8 bit value, which should be stored in the ring buffer

  Return:
    0:	Data not stored, ring buffer is full
    1:	all ok
    
*/
int rb_add(rb_t *rb, uint8_t data)
{
  uint32_t primask;
  uint16_t end;
  
  primask = __get_PRIMASK();	/* get the interrupt status */
  __disable_irq();			/* disable IRQs, this will modify PRIMASK */
  
  end = rb_next_val(rb, rb->end);
  if ( end == rb->start )
    return __set_PRIMASK(primask), 0; /* restore interrupt state & return error */
  rb->ptr[rb->end] = data;
  rb->end = end;
  __set_PRIMASK(primask);	/* restore interrupt state */
  return 1;
}

/* 
  Prototype:
    int rb_get(rb_t *rb)

  Description:
    Get data out of the ring buffer and remove this data item from the ring buffer.

  Precondition:
    rb_init() must be called on the "rb" argument.

  Parameter:
    rb: 	Address of a (initialized) buffer data structure

  Return:
    -1 if there is no data available, otherwise the data which was added before.
    
*/
int rb_get(rb_t *rb)
{
  uint32_t primask;
  int data;
  if ( rb->end == rb->start )
    return -1;
  
  primask = __get_PRIMASK();	/* get the interrupt status */
  __disable_irq();			/* disable IRQs, this will modify PRIMASK */
  
  data = rb->ptr[rb->start];
  rb->start = rb_next_val(rb, rb->start);
  __set_PRIMASK(primask);	/* restore interrupt state */

  return data;  
}


