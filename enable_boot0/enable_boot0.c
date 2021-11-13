/* 
  
  enable_boot0.c
  
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


  Background:
    The BOOT0 pin is disabled for STM32G0 devices by default: The bootloader
    will not be executed with a high level applied to BOOT0 pin during reset.
    This means: The bootloader is not executed any more after first successful 
    upload of any code.
    As a result the upload via UART can be only done once for an empty device.
    
  Solution:
    Upload and execute this code as first and initial flash operation: 
    It will enable the BOOT0 pin behavior, so that a high level on BOOT0 pin 
    during reset will force the bootloader to execute.

  Instructions for UART uploads:
  1. Generate hex file from this code
  2. Upload and execute the generated hex file via UART (stm32flash -g 0)
  3. Wait for 1 second: The BOOT0 pin is now activated
  4. Upload your own code
  
  Reference:
  https://community.st.com/s/question/0D50X0000ARQLq2/has-anyone-gotten-the-boot0-pin-to-work-on-an-stm32g071-solved
    
*/

#include "stm32g0xx.h"

int main()
{
  /* check for the BOOT0 selection to avoid reflashing */  
  if ( (FLASH->OPTR & FLASH_OPTR_nBOOT_SEL) == 0 )
  {
    /* nBOOT_SEL is already cleared..., do nothing */
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
  
  /* check if there is any flash operation */
  while( (FLASH->SR & FLASH_SR_BSY1) != 0 )
    ;
  
  /* start the option byte flash */
  FLASH->CR |= FLASH_CR_OPTSTRT;
  /* wait until flashing is done */
  while( (FLASH->SR & FLASH_SR_BSY1) != 0 )
    ;  

  /* do a busy delay, for about one second, check BSY1 flag to avoid compiler loop optimization */
  for( unsigned long i = 0; i < 2000000; i++ )
    if ( (FLASH->SR & FLASH_SR_BSY1) != 0 )
      break;
  
  /* load the new value and do a system reset */
  /* this will behave like a goto to the begin of this main procedure */
  FLASH->CR |= FLASH_CR_OBL_LAUNCH;
    
  /* we will never arrive here */
  for(;;)
    ;
}
