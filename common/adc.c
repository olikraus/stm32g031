/*

  adc.c

  BSD 3-Clause License

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
#include "delay.h"

/*
  generic ADC init for the STM32G0xx
    - setup RCC for ADC
    - Do ADC calibration
    - Wakeup VREFINT
    - Wakeup temperature sensor
    
    this function will call delay_micro_seconds()
    
    configuration:
      - PCLK/2 sync clock for ADC --> 32MHz (close to the max 35 MHz)
      - No oversampler
      - 12 Bit resolution
    
*/
void adc_init(void)
{
  short i;
  //__disable_irq();
  
  
    /* DMA Clock Enable */
  
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;
  __NOP();								/* let us wait for some time */
  __NOP();								/* let us wait for some time */  

  RCC->AHBRSTR |= RCC_AHBRSTR_DMA1RST;
  __NOP();								/* let us wait for some time */
  __NOP();								/* let us wait for some time */  
  RCC->AHBRSTR &= ~RCC_AHBRSTR_DMA1RST;
  __NOP();								/* let us wait for some time */
  __NOP();								/* let us wait for some time */  
  

  /* ADC Clock Enable */ 
  
  RCC->APBENR2 |= RCC_APBENR2_ADCEN;	/* enable ADC clock */
  __NOP();								/* let us wait for some time */
  __NOP();								/* let us wait for some time */  
  
  /* ADC Reset */
  
  RCC->APBRSTR2 |= RCC_APBRSTR2_ADCRST;
  __NOP();								/* let us wait for some time */
  __NOP();								/* let us wait for some time */
  RCC->APBRSTR2 &= ~RCC_APBRSTR2_ADCRST;
  __NOP();								/* let us wait for some time */
  __NOP();								/* let us wait for some time */

  
  /* ADC Basic Setup */
  
  ADC1->IER = 0;						/* do not allow any interrupts */
  ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE;	// clear the clock mode, which would the select the async clock source from RCC, which is the 64MHz system clock by default
  /*
    PCLK is 64MHz  
    Instead of the async clock, select PCLK/2. --> ADC clock = 32 MHz (which is close to the max 35 MHz mentioned in the datasheet)
  */
  ADC1->CFGR2 |= ADC_CFGR2_CKMODE_0; // select PCLK/2 --> 32MHz ADC clock
  

  /* oversampler */
  ADC1->CFGR2 &= ~ADC_CFGR2_OVSE; // disable oversampler
  ADC1->CFGR2 &= ~ADC_CFGR2_OVSS;  // clear oversampling shift
  ADC1->CFGR2 &= ~ADC_CFGR2_OVSR;  // clear oversampling ratio
  
  //ADC1->CFGR2 |= ADC_CFGR2_OVSE         // enable oversampler
  //  | (3 << ADC_CFGR2_OVSS_Pos)         // bit shift cnt
  //  | (2 << ADC_CFGR2_OVSR_Pos);         // 1: 4x, 2: 8x, 3: 16x
  
  ADC1->CR |= ADC_CR_ADVREGEN;				/* enable ADC voltage regulator, 20us wait time required */

  delay_micro_seconds(20);


  /* ADC Clock prescaler */
  ADC->CCR &= ~ADC_CCR_PRESC;           // clear prescalar (ADC clock not divided)
  //ADC->CCR |= ADC_CCR_PRESC_2;                  /* divide by 0100=8 */
  
  
  ADC->CCR |= ADC_CCR_VREFEN; 			/* Wake-up the VREFINT */  
  ADC->CCR |= ADC_CCR_TSEN; 			/* Wake-up the temperature sensor */  
  //ADC->CCR |= ADC_CCR_VBATEN; 			/* Wake-up VBAT sensor */  

  __NOP();								/* let us wait for some time */
  __NOP();								/* let us wait for some time */

  /* CALIBRATION */
  
  if ((ADC1->CR & ADC_CR_ADEN) != 0) /* clear ADEN flag if required */
  {
    ADC1->CR &= (uint32_t)(ADC_CR_ADDIS);
  }
  
  ADC1->CR |= ADC_CR_ADCAL; 				/* start calibration */
  
  while ((ADC1->ISR & ADC_ISR_EOCAL) == 0) 	/* wait for clibration finished */
  {
  }
  ADC1->ISR |= ADC_ISR_EOCAL; 			/* clear the status flag, by writing 1 to it */
  
  
  for( i = 0; i < 64; i++ )
    __NOP();								/* not sure why, but some nop's are required here, at least 8 of them with 16MHz */

  /* ENABLE ADC */

  /* minimal sampling time for both sampling values */  
  /* 
    1.5 clk sampling time + 12.5 conversion time for 12 bit --> 14 adc clocks @ 32MHz --> 0.44us --> 2.285.714 samples per second
    --> 1000Hz --> 2285 samples
  */
  
  ADC1->SMPR &= ~ADC_SMPR_SMP1;
  ADC1->SMPR &= ~ADC_SMPR_SMP2;

  /*
    000: 1.5 ADC clock cycles
    001: 3.5 ADC clock cycles
    010: 7.5 ADC clock cycles
    011: 12.5 ADC clock cycles
    100: 19.5 ADC clock cycles
    101: 39.5 ADC clock cycles
    110: 79.5 ADC clock cycles
    111: 160.5 ADC clock cycles
  */
  //ADC1->SMPR |= ADC_SMPR_SMP1_0 | ADC_SMPR_SMP1_1 | ADC_SMPR_SMP1_2; /* Select a sampling mode of 111 (very slow)*/
  //ADC1->SMPR |= ADC_SMPR_SMP2_0 | ADC_SMPR_SMP2_1 | ADC_SMPR_SMP2_2; /* Select a sampling mode of 111 (very slow)*/

  /* sampling time needs to be at least 12.5 ADC clock cycles for good results if the channels are changing */
  /* minimal sampling time for both sampling values */  
  /* 
    12.5 clk sampling time + 12.5 conversion time for 12 bit --> 25 adc clocks @ 32MHz --> 0.78us --> 1.282.051 samples per second
  */
  ADC1->SMPR |= ADC_SMPR_SMP1_0 | ADC_SMPR_SMP1_1;
  ADC1->SMPR |= ADC_SMPR_SMP2_0 | ADC_SMPR_SMP2_1;
  
  
  /* ADC result configuration */
  ADC1->CFGR1 &= ~ADC_CFGR1_ALIGN;		/* right alignment */
  ADC1->CFGR1 &= ~ADC_CFGR1_RES;		/* 12 bit resolution */

  
  
  ADC1->ISR |= ADC_ISR_ADRDY; 			/* clear ready flag */
  ADC1->CR |= ADC_CR_ADEN; 			/* enable ADC */
  while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* wait for ADC */
  {
  }
  
}


/*
  execute single converion and return the value from the specified channel 
  adc_init() will be called if the adc is not enabled.

  ch 0..7 ==  ADC_IN0..7  == PA0..PA7
  ch 8 == ADC_IN8 == PB0
  ch 9 == ADC_IN9 == PB1
  ch 10 == ADC_IN10 == PB2
  ch 11 == ADC_IN11 == PB7 / PB10
  ch 12: temperture sensor
  ch 13: vrefint (1212mV)
  ch 14: vbat (not available)
  ch 15 == ADC_IN15 == PB11 / PA11 [PA9]
  ch 16 == ADC_IN16 == PB12 / PA12 [PA10]
  ch 17 == ADC_IN17 == PA13
  ch 18 == ADC_IN18 == PA14 (BOOT0)

  GPIO pins should be in analog mode (which is reset default)
*/
uint16_t adc_get_value(uint8_t ch)
{
  uint32_t dummyread __attribute__((unused));
  unsigned short timeout = 20000;
  
  if ( (ADC1->CR & ADC_CR_ADEN)==0 )
  {
    adc_init();
    return 0;
  }

  /* stop any pending conversion */
  ADC1->CR |= ADC_CR_ADSTP;
  while( ADC1->CR & ADC_CR_ADSTP )      // wait until stop is executed
      ;

  /* looks like DR is somehow double buffered: do a read access to the data to remove any pending value */
  dummyread = ADC1->DR;

  /* CONFIGURE ADC */

  ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;              // disable DMA
  ADC1->CFGR1 &= ~ADC_CFGR1_CONT;               // disable continues mode
  
  //DMA1_Channel1->CCR = 0;       // clear DMA channel register
  //DMA1_Channel1->CNDTR = 0;                                        /* buffer size, number of ADC scans --> array length */
  //DMAMUX1_Channel0->CCR = 0;    // clear DMA multiplexer
  
  ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN;	/* software enabled conversion start */
  //ADC1->CFGR1 &= ~ADC_CFGR1_ALIGN;		/* right alignment */
  //ADC1->CFGR1 &= ~ADC_CFGR1_RES;		/* 12 bit resolution */
  ADC1->CHSELR = 1<<ch; 				/* Select channel */
  //ADC1->SMPR |= ADC_SMPR_SMP1_0 | ADC_SMPR_SMP1_1 | ADC_SMPR_SMP1_2; /* Select a sampling mode of 111 (very slow)*/
  //ADC1->SMPR |= ADC_SMPR_SMP2_0 | ADC_SMPR_SMP2_1 | ADC_SMPR_SMP2_2; /* Select a sampling mode of 111 (very slow)*/

  //ADC1->SMPR &= ~ADC_SMPR_SMP1;
  //ADC1->SMPR &= ~ADC_SMPR_SMP2;
  //ADC1->SMPR |= ADC_SMPR_SMP1_2; /* Select a sampling mode of 100 (19.6 ADC cycles)*/
  //ADC1->SMPR |= ADC_SMPR_SMP2_2; /* Select a sampling mode of 100 (19.6 ADC cycles)*/

  /* DO CONVERSION */

  ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversion */
  while ((ADC1->ISR & ADC_ISR_EOC) == 0) /* wait end of conversion */
  {
    if ( timeout == 0 )
      return 0;
    timeout--;
  }
  return ADC1->DR;						/* get ADC result and clear the ISR_EOC flag */
}

/*
  read multiple ADC values
*/
void adc_get_multiple_values(uint16_t *adr, uint16_t cnt, uint8_t ch)
{
  
  /* stop any pending conversion */
  ADC1->CR |= ADC_CR_ADSTP;
  while( ADC1->CR & ADC_CR_ADSTP )      // wait until stop is executed
      ;
  
  /* configure adc */

  ADC1->CFGR1 &= ~ADC_CFGR1_DMACFG;              // disable DMA circular mode --> one shot mode
  ADC1->CFGR1 |= ADC_CFGR1_DMAEN;              // enable DMA
  ADC1->CFGR1 |= ADC_CFGR1_CONT;               // enable continues mode (because we need to read 'cnt' values
  
  ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN;	// software enabled conversion start 
  //ADC1->CFGR1 |= ADC_CFGR1_EXTEN_0;     // HW trigger, rising edge
  //ADC1->CFGR1 |= ADC_CFGR1_EXTEN_1;     // HW trigger, falling edge
  //ADC1->CFGR1 &= ~ADC_CFGR1_EXTSEL;     // HW trigger input is TRG0 ("000"), which is TRGO2 from TIM1
  
  ADC1->CHSELR = 1<<ch; 				/* Select channel */

  /* configure DMA */
  
  DMA1_Channel1->CCR = 0;       // clear DMA channel register
  
  DMA1_Channel1->CNDTR = cnt;                                        /* buffer size, number of ADC scans --> array length */
  DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);                     /* source value */
  DMA1_Channel1->CMAR = (uint32_t)adr;                   /* destination memory */

  DMA1_Channel1->CCR |= DMA_CCR_PL;		/* highest prio */   
  DMA1_Channel1->CCR |= DMA_CCR_MINC;		/* increment memory */   
  DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0;		/* 01: 16 Bit access */   
  DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0;		/* 01: 16 Bit access */   

  /* 
    DMA MUX channel 0 connected to DMA channel 1!
    
    see code comment here:
    https://vivonomicon.com/2019/07/05/bare-metal-stm32-programming-part-9-dma-megamix/
    --> mux channel0 connects to dma1
  */
  DMAMUX1_Channel0->CCR = 0;
  DMAMUX1_Channel0->CCR =  5<<DMAMUX_CxCR_DMAREQ_ID_Pos;   /* 5=ADC */

  DMA1_Channel1->CCR |= DMA_CCR_EN;                /* enable */
  

  ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversion */

  //delay_micro_seconds(cnt);     // test delay, conversion time is actually lesser than one 1us per sample
}


/*
  read from multiple channels 
  results are stored in 'adr'

  'channels' must contain a '1' bit for each requested channel
  ch 0..7 ==  ADC_IN0..7  == PA0..PA7
  ch 8 == ADC_IN8 == PB0
  ch 9 == ADC_IN9 == PB1
  ch 10 == ADC_IN10 == PB2
  ch 11 == ADC_IN11 == PB7 / PB10
  ch 12: temperture sensor
  ch 13: vrefint (1212mV)
  ch 14: vbat (not available)
  ch 15 == ADC_IN15 == PB11 / PA11 [PA9]
  ch 16 == ADC_IN16 == PB12 / PA12 [PA10]
  ch 17 == ADC_IN17 == PA13
  ch 18 == ADC_IN18 == PA14 (BOOT0)

  Size of 'adr' must be the number of '1' in 'channels'

*/
void adc_get_channel_values(uint32_t channels, uint16_t *adr)
{
  uint32_t dummyread __attribute__((unused));
  uint32_t c;
  uint16_t cnt;
  unsigned short timeout = 20000;

  cnt = 0;
  c = channels;
  if ( c == 0 )
    return;             /* nothing todo */
  
  /* calculate the number of bits 1 channels */
  while( c > 0 )
  {
    if ( c & 1 ) 
      cnt++;
      c >>= 1;
  }
  
  /* stop any pending conversion */
  ADC1->CR |= ADC_CR_ADSTP;
  while( ADC1->CR & ADC_CR_ADSTP )      // wait until stop is executed
      ;

  /* looks like DR is somehow double buffered: do a read access to the data to remove any pending value */
  dummyread = ADC1->DR;

  /* CONFIGURE ADC */

  ADC1->CFGR1 &= ~ADC_CFGR1_DMACFG;              // disable DMA circular mode --> one shot mode
  ADC1->CFGR1 |= ADC_CFGR1_DMAEN;              // enable DMA
  ADC1->CFGR1 &= ~ADC_CFGR1_CONT;               // disable continues mode
  
  //DMA1_Channel1->CCR = 0;       // clear DMA channel register
  //DMA1_Channel1->CNDTR = 0;                                        /* buffer size, number of ADC scans --> array length */
  //DMAMUX1_Channel0->CCR = 0;    // clear DMA multiplexer
  
  ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN;	/* software enabled conversion start */
  //ADC1->CFGR1 &= ~ADC_CFGR1_ALIGN;		/* right alignment */
  //ADC1->CFGR1 &= ~ADC_CFGR1_RES;		/* 12 bit resolution */
  ADC1->CHSELR = channels; 				/* Select channels */
  //ADC1->SMPR |= ADC_SMPR_#_0 | ADC_SMPR_SMP1_1 | ADC_SMPR_SMP1_2; /* Select a sampling mode of 111 (very slow)*/
  //ADC1->SMPR |= ADC_SMPR_SMP2_0 | ADC_SMPR_SMP2_1 | ADC_SMPR_SMP2_2; /* Select a sampling mode of 111 (very slow)*/

  //ADC1->SMPR &= ~ADC_SMPR_SMP1;
  //ADC1->SMPR &= ~ADC_SMPR_SMP2;
  //ADC1->SMPR |= ADC_SMPR_SMP1_2; /* Select a sampling mode of 100 (19.6 ADC cycles)*/
  //ADC1->SMPR |= ADC_SMPR_SMP2_2; /* Select a sampling mode of 100 (19.6 ADC cycles)*/

  /* configure DMA */
  
  DMA1_Channel1->CCR = 0;       // clear DMA channel register
  
  DMA1_Channel1->CNDTR = cnt;                                        /* buffer size, number of ADC scans --> array length */
  DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);                     /* source value */
  DMA1_Channel1->CMAR = (uint32_t)adr;                   /* destination memory */

  DMA1_Channel1->CCR |= DMA_CCR_PL;		/* highest prio */   
  DMA1_Channel1->CCR |= DMA_CCR_MINC;		/* increment memory */   
  DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0;		/* 01: 16 Bit access */   
  DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0;		/* 01: 16 Bit access */   

  /* 
    DMA MUX channel 0 connected to DMA channel 1!
    
    see code comment here:
    https://vivonomicon.com/2019/07/05/bare-metal-stm32-programming-part-9-dma-megamix/
    --> mux channel0 connects to dma1
  */
  DMAMUX1_Channel0->CCR = 0;
  DMAMUX1_Channel0->CCR =  5<<DMAMUX_CxCR_DMAREQ_ID_Pos;   /* 5=ADC */

  DMA1_Channel1->CCR |= DMA_CCR_EN;                /* enable */
  

  /* DO CONVERSION */

  ADC1->ISR |= ADC_ISR_EOS;             /* clear the end of sequence bit */
  ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversion */
  while ((ADC1->ISR & ADC_ISR_EOS) == 0) /* wait end of sequence conversion */
  {
    if ( timeout == 0 )
      return 0;
    timeout--;
  }
  
  
}


