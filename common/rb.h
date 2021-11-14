/*

  rb.h
  
  Ring Buffer 
  
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


#ifndef _RB_H
#define _RB_H


#include <stdint.h>

/*
  ring buffer
*/
struct _rb_struct
{
  uint8_t *ptr;  
  uint16_t start;
  uint16_t end;
  uint16_t len;
};
typedef struct _rb_struct rb_t;


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
void rb_init(rb_t *rb, uint8_t *buf, uint16_t len);

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
int rb_add(rb_t *rb, uint8_t data);

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
int rb_get(rb_t *rb);

#endif