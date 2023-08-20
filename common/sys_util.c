/* 

  sys_util.c

  Copyright (c) 2023, olikraus@gmail.com
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

#include "stm32g0xx.h"
#include "sys_util.h"

/*
  Set system clock to 64MHz (max speed)
  Enable PLL P, Q and R output.
    P: 64MHz
    Q: 128MHz
    R: 64MHz
  Calls SystemCoreClockUpdate() at the end to update the SystemCoreClock variable.
*/
void set_64mhz_sysclk(void)
{
    
  /* test if the current clock source is something else than HSI */
  /* 0 means HSISYS clock source */
  if ((RCC->CFGR & RCC_CFGR_SWS) != 0) 
  {
    /* HSI is not the system clock, so enable HSI and change sys clock to HSI */
    
    /* clear all bits of the HSI divider: divide by 1, so HSISYS clock == HSI clock */
    RCC->CR &= (uint32_t) (~RCC_CR_HSIDIV);
    
    /* enable HSI */
    RCC->CR |= RCC_CR_HSION;    
    /* wait until HSI becomes ready */
    while ( (RCC->CR & RCC_CR_HSIRDY) == 0 )
      ;      
 
    /* then use the HSI clock */
    /* clear all bits, which means: Use HSISYS for SYSCLK */
    RCC->CFGR &= (uint32_t) (~RCC_CFGR_SW);     
    /* wait until HSI clock is used */
    while ((RCC->CFGR & RCC_CFGR_SWS) != 0)
      ;
  }
  else
  {
      /* HSIDIV might not be 0, but this doesn't matter for the PLL below */
  }
  
  /* At this point the HSI runs with 16 MHz */
  
  /* disable PLL */
  RCC->CR &= (uint32_t)(~RCC_CR_PLLON);
  /* wait until PLL is inactive */
  while((RCC->CR & RCC_CR_PLLRDY) != 0)
    ;

  /* set latency to 2 wait states */
  FLASH->ACR &= (uint32_t) (~FLASH_ACR_LATENCY);
  FLASH->ACR |= FLASH_ACR_LATENCY_1;
  
  /* wait until flash latency has been assigned */
  while( (FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_1 )
    ;
  
  /* enable instruction prefech */
  FLASH->ACR |= FLASH_ACR_PRFTEN;

  /* enable instruction cache */
  FLASH->ACR |= FLASH_ACR_ICEN;
  
  /*
    PLL VCO Programming:
      fVCO = fPLLIN × (N / M)
      fPLLP = fVCO / P          I2S and ADC clock sources
      fPLLQ = fVCO / Q          High speed clock for TIM1
      fPLLR = fVCO / R          Used for the system clock

      Sideconditions:
        Valid ranges: N=8..86, M=1..8 P=2..32 Q=2..8 R=2..8
        fVCO = 64..344 MHz
        fPLLIN / M = 2.66..16MHz
      To drive TIM1 with 128MHz we need fPLLQ at 128MHz, which means fVCO >= 256 MHz
      With M=2: fPLLIN / M = 8MHz
      for fVCO=256MHz, N must be 32
      
      ADC max asyic clock is 122MHz
      For this we could use M=3, N=45 --> fVCO=240MHz --> with P=2 --> fPLLP=120MHz --> Sysclk=60MHz
      or:  M=4, N=61 --> fVCO=244MHz --> with P=2 --> fPLLP=122MHz --> Sysclk=61MHz
  */
  
  RCC->PLLCFGR = (3<<RCC_PLLCFGR_PLLR_Pos)  // SYSCLK: fVCO division by 4 --> 64MHz
                            | RCC_PLLCFGR_PLLREN // enable fPLLR output
                            | (1<<RCC_PLLCFGR_PLLQ_Pos) // fVCO div by 2 --> 128 MHz
                            | RCC_PLLCFGR_PLLQEN // enable fPLLQ
                            | (3<<RCC_PLLCFGR_PLLP_Pos) // fVCO div by 4 --> 64 MHz 
                            | RCC_PLLCFGR_PLLPEN // enable fPLLP
                            | (32<<RCC_PLLCFGR_PLLN_Pos)
                            | (1<<RCC_PLLCFGR_PLLM_Pos)  // value 1 means divide by 2
                            | RCC_PLLCFGR_PLLSRC_1 // "10" selet HSI16
                            ;
                          
  /* enable PLL */
  RCC->CR |= RCC_CR_PLLON; 
  
  /* wait until the PLL is ready */
  while ((RCC->CR & RCC_CR_PLLRDY) == 0)
    ;

  /* use the PLLRCLK as clock source for SYSCLK */
  /* Note: All bits in SW are already cleared, so just set the second bit: "010" for PLL source */
  RCC->CFGR |= (uint32_t) (RCC_CFGR_SW_1); 
  /* wait until the PLL source is active */
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_1) 
    ;
  
  /* recalculate SystemCoreClock variable */
  SystemCoreClockUpdate();  
}

