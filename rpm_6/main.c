/* 

  RPM calculation with STM32G031
  
  Hall sensor at PA13


  Configuration is 57600 8-N-1. Only "\n" is sent. Receiving terminal should add \r
  (e.g. use add CR function in "minicom")
  

  Linux:
    stty -F /dev/ttyUSB0 sane 57600 && cat /dev/ttyUSB0
    or stty -F /dev/ttyUSB0 sane 57600 igncr  && cat /dev/ttyUSB0
    screen /dev/ttyUSB0  57600 (terminate with "C-a k" or "C-a \")
    minicom -D /dev/ttyUSB0  -b 57600 (terminate with "C-a x", change CR mode: "C-a u", disable HW control flow!)
    
  Baud Rates: 57600 115200

  Ensure that -DUSER_VECT_TAB_ADDRESS is set during compilation, otherwise
  interrupts will not work after the "go" commant of the flashware USART upload.
  
  Problem is, that SCB->VTOR doesn't point to FLASH_BASE, so manually
  assigning SCB->VTOR = FLASH_BASE;	will also fix this.


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

#include "stm32g0xx.h"
#include "delay.h"
#include "usart.h"

/*=======================================================================*/
/* Global variables and constants */
/*=======================================================================*/


/* After which time the sys tick handler is called again. */
/* 1/SYS_TICK_HANDLER_MS is the frequency of the sys tick handler call */
/* Should be fixed to 200Hz / 50ms */
#define SYS_TICK_HANDLER_MS 1

/* Number of calls to the SysTick handler  */
volatile unsigned long SysTickCount = 0;

/* state variable: defines which task to call */
volatile uint16_t SysTickSchedulerCount = 0;

/* processor cycles between calls to systick handler, should be ca. 800000 (=16MHz/20Hz) */
uint32_t SysTickClockUsage = 0;

/* duration of the systick handler */
uint32_t SysTickClockPeriod = 0;


/* number of ADC sources, which will be scanned and copied to ADCRawValues */
#define ADC_SRC_CNT 4

/* Raw values from the ADC1 data register, copyied to this array by DMA channel 1 */
uint16_t ADCRawValues[ADC_SRC_CNT] __attribute__ ((aligned (4)));


/* number of ADC1 end of sequence events since startup */
uint32_t ADCEOSCnt = 0;

/* systick value of the last start of the ADC (used for ADCDuration) [systicks] */
uint32_t ADCStartTime = 0;

/* the duration of the ADC converion sequence [systicks], value shoud be ca. n*2000, where n is the sequence length */ 
/* ADCDuration seems to be 150 clocks longer if AUTOFF is active */
uint32_t ADCDuration = 0;

/* value of ADC1->ISR inside the ADC interrupt after EOS, seems to be %0000000000001010 (independent from AUTOFF flag) */
uint32_t ADCISRAfterEOS = 0;

/* a flag, which indicates that the ADC is ready */
int16_t isADCSetupDone = 0;

/* a flag, which indicates whether the background ADC sequence conversion is active */
int16_t ADCScanActive = 0;

/* the voltage applied to the microcontroller (equal to the reference voltage of the ADC) [mV] */
uint16_t supplyVoltage = 0;

/* hall raw value */
uint16_t hallRaw = 0;

/* hall min and max values, both are directly taken from hallRaw */
uint16_t hallRawMin = 0xffff;
uint16_t hallRawMax = 0;

/* stable min/max/average/diff values, assigned in slow task */
uint16_t hallMin = 0xffff;
uint16_t hallMax = 0;
uint16_t hallAvg = 0;   /* = (max + min) */
uint16_t hallDiff = 0;  /* = max - min */
uint16_t hallDiff4 = 0; /* = hallDiff/4 */

/* rpm calculation state: 
  0=not started 
  1=wait for >hallAvg+hallDiff
  2 = wait for < hallAvg+hallDiff
  3 = wait for < hallAvg- hallDiff
*/
uint16_t rpmState = 0;

/* count the length of a revolution in fast user task ticks (=1ms) */
uint16_t rpmCount = 0;

/* number of slow task ticks for one revolution (max val of  rpmCount) */
uint16_t rpmTicksPerRevolution = 0;

/* raw speed is derived from rpmTicksPerRevolution 
  revolutions per second = frequency = 1000/rpmTicksPerRevolution
  revolutions per minute  = 60000/rpmTicksPerRevolution
*/
uint16_t freqRaw = 0;
uint16_t rpmRaw = 0;


/*=======================================================================*/
/* Utility Procedures */
/*=======================================================================*/

/*
  Measure the number or processor clock cycles:
  uint32_t start;

  start = SysTick->VAL;
  ...
  getProcessorClockDelta(start); // retun the number of processor cycles since start was recorded

  Limitation: The number of cycles between start and getProcessorClockDelta(start) must not exceed Systick->LOAD


*/
uint32_t getProcessorClockDelta(uint32_t start_value)
{
  uint32_t current_value = SysTick->VAL;
  /* SysTick->VAL is decremented, so the simple case is current_value < start_value */
  if ( current_value < start_value )
    return start_value-current_value;
  /* reload happend since start_value */
  return SysTick->LOAD - current_value + start_value;
}

/* low pass filter with 8 bit resolution, p = 0..255 */
#define LOW_PASS_BITS 8
int32_t low_pass(int32_t *a, int32_t x, int32_t p)
{
  int32_t n;
  //n = ((1<<LOW_PASS_BITS)-p) * (*a) + p * x + (1<<(LOW_PASS_BITS-1));
  n = ((1<<LOW_PASS_BITS)-p) * (*a) + p * x ;
  n >>= LOW_PASS_BITS;
  *a = n;
  return n;
}



/*=======================================================================*/
/* ADC and DMA */
/*=======================================================================*/

/*
  Called at the end of conversion
*/
void __attribute__ ((interrupt, used)) ADC1_IRQHandler(void)
{
  /* Check for "end of sequence" */
  if ( (ADC1->ISR & ADC_ISR_EOS) != 0 )
  {
    ADCDuration = getProcessorClockDelta(ADCStartTime); /* calculate the duration of the sequence conversion. */
    ADCISRAfterEOS = ADC1->ISR;
    ADC1->ISR |= ADC_ISR_EOS;   /* clear the eos event */
    ADCEOSCnt++;
    ADCScanActive = 0;
  }
}



/*
  Start ADC conversion sequence in background.
  DMA will copy the data to ADCRawValues array.
  ADC1_IRQHandler will be called at end of sequence.
*/
void startADC(void)
{
  DMA1_Channel1->CCR = 0; /* disable DMA */
  DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);
  DMA1_Channel1->CMAR = (uint32_t)&(ADCRawValues[0]);
  DMA1_Channel1->CNDTR = ADC_SRC_CNT;
  DMA1_Channel1->CCR = DMA_CCR_PL               /* highest piority */
    | DMA_CCR_MSIZE_0                           /* 32 bit transfer memory size */
    | DMA_CCR_PSIZE_0                           /* 32 bit peripheral size */
    | DMA_CCR_MINC                              /* increment memory address after each transfer */
    | DMA_CCR_CIRC                              /* repeat with inital memory address */
          /* DIR=0: transfer from CPAR to CMAR */
    ;

  /* 
    DMA MUX channel 0 connected to DMA channel 1!
    
    see code comment here:
    https://vivonomicon.com/2019/07/05/bare-metal-stm32-programming-part-9-dma-megamix/
    --> mux channel0 connects to dma1
  */
  DMAMUX1_Channel0->CCR = 0;
  DMAMUX1_Channel0->CCR =  5<<DMAMUX_CxCR_DMAREQ_ID_Pos;   /* 5=ADC */

  DMA1_Channel1->CCR |= DMA_CCR_EN;  /* enable DMA */ 

  ADCScanActive = 1;
  ADCStartTime = SysTick->VAL;          /* store start time of the background sampling activity */
  ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversion */
}


/* requires proper setup of the systick timer, because a 20ms delay is needed here */
void initADC(void)
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

  /* Allow ADC interrupts */

  NVIC_EnableIRQ(ADC1_IRQn);

  /* ADC Basic Setup */
  
  ADC1->IER = 0;						/* do not allow any interrupts */
  ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE;	/* select HSI16 clock */
  
  /* oversampler */
  ADC1->CFGR2 &= ~ADC_CFGR2_OVSS;
  ADC1->CFGR2 &= ~ADC_CFGR2_OVSR;
  
  ADC1->CFGR2 |= ADC_CFGR2_OVSE
    | (3 << ADC_CFGR2_OVSS_Pos)         // bit shift
    | (2 << ADC_CFGR2_OVSR_Pos);         // 1: 4x, 2: 8x, 3: 16x
  
  ADC1->CR |= ADC_CR_ADVREGEN;				/* enable ADC voltage regulator, 20us wait time required */
  delay_micro_seconds(20);


  /* ADC Clock prescaler */
  ADC->CCR &= ~ADC_CCR_PRESC;
  ADC->CCR |= ADC_CCR_PRESC_2;                  /* divide by 0100=8 */
  
  
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

  for( i = 0; i < 48; i++ )                             /* post calibration delay */
    __NOP();								/* not sure why, but some nop's are required here, at least 8 of them with 16MHz */

    /* CONFIGURE ADC */

  ADC1->SMPR &= ~ADC_SMPR_SMP1;
  ADC1->SMPR &= ~ADC_SMPR_SMP2;
  ADC1->SMPR |= ADC_SMPR_SMP1_2; /* Select a sampling mode of 100 (19.6 ADC cycles)*/
  ADC1->SMPR |= ADC_SMPR_SMP2_2; /* Select a sampling mode of 100 (19.6 ADC cycles)*/

  ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN;	/* software enabled conversion start */
  ADC1->CFGR1 &= ~ADC_CFGR1_ALIGN;		/* right alignment */
  ADC1->CFGR1 &= ~ADC_CFGR1_RES;		/* 12 bit resolution */
  
  /* CONFIGURE SEQUENCER */
  
  ADC1->ISR &= ~ADC_ISR_CCRDY;          /* clear the channel config flag */

/*
  ch 12: temperture sensor
  ch 13: vrefint
  ch 14: vbat
*/
  ADC1->CFGR1 &= ~ADC_CFGR1_CHSELRMOD;  /* "not fully configurable" mode */
  ADC1->CFGR1 &= ~ADC_CFGR1_SCANDIR;    /* forward scan */
  ADC1->CFGR1 &= ~ADC_CFGR1_DISCEN;     /* disable discontinues mode */
  //ADC1->CFGR1 |= ADC_CFGR1_CONT;        /* continues mode */
  ADC1->CFGR1 &= ~ADC_CFGR1_CONT;        /* disable continues mode: excute the sequence only once  */
  ADC1->CFGR1 |= ADC_CFGR1_AUTOFF;     /* enable auto off feature, according to the datasheet, the ADRDY flag will not be raised if this feature is enabled */
  
  ADC1->CHSELR = 
    ADC_CHSELR_CHSEL5 |                 /* external temperature sensor */
    ADC_CHSELR_CHSEL12 |                /* internal temperature sensor */
    ADC_CHSELR_CHSEL13 |                /* internal reference voltage (bandgap) */
    ADC_CHSELR_CHSEL17;                 /* PA13 */
  
  while((ADC1->ISR&ADC_ISR_CCRDY) == 0) /* wait until channel config is applied */
  {
  }

  
  /* DMA CONFIGURATION */
  
  /* How to handle OVR BIT???? */
  ADC1->CFGR1 |= ADC_CFGR1_DMACFG;      /* DMA continues mode (probably doesn't matter, because ADC is in single mode) */
  ADC1->CFGR1 |= ADC_CFGR1_DMAEN;      /* DMA enable */
  
  /* DMA CHANNEL SETUP */

#ifdef NOT_REQUIRED_WILL_BE_DONE_WITH_ADC_START
  
  DMA1_Channel1->CCR = 0; /* ensure to disable DMA */
  DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);
  DMA1_Channel1->CMAR = (uint32_t)&(ADCRawValues[0]);
  DMA1_Channel1->CNDTR = ADC_SRC_CNT;
  DMA1_Channel1->CCR = DMA_CCR_PL               /* highest piority */
    | DMA_CCR_MSIZE_0                           /* 32 bit transfer memory size */
    | DMA_CCR_PSIZE_0                           /* 32 bit peripheral size */
    | DMA_CCR_MINC                              /* increment memory address after each transfer */
    | DMA_CCR_CIRC                              /* repeat with inital memory address */
          /* DIR=0: transfer from CPAR to CMAR */
    ;

  /* 
    which DMAMUX channel is connect to which DMA channel???
    Is mux channel 0 connected to dma channel 1 ???
    
    see code comment here:
    https://vivonomicon.com/2019/07/05/bare-metal-stm32-programming-part-9-dma-megamix/
    --> mux channel0 connects to dma1
  */
  DMAMUX1_Channel0->CCR = 0;
  DMAMUX1_Channel0->CCR =  5<<DMAMUX_CxCR_DMAREQ_ID_Pos;   /* 5=ADC */

  /* enable DMA */  
  DMA1_Channel1->CCR |= DMA_CCR_EN;
#endif
  
  /* ENABLE ADC */

  ADC1->IER |= ADC_IER_EOSIE;                    /* Enable "end of sequence" interrupt event */
  ADC1->ISR |= ADC_ISR_ADRDY; 			/* clear ready flag */
  ADC1->CR |= ADC_CR_ADEN; 			/* enable ADC */
  
  /* according to the datasheet, the ADRDY flag is not set if the AUTOFF flag is active. */
  /* Instead power on sequence is done automatically. */ 
  //while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* wait for ADC */
  //{
  //}
  
  /* let's wait for some time instead of waiting for the ADRDY flag */
  delay_micro_seconds(20);


  //ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversion */
  
  isADCSetupDone = 1;    /* mark ADC as ready */
  
  
}




/*=======================================================================*/
/* Tasks */
/*=======================================================================*/




void user_fast_task(void)
{
  if ( isADCSetupDone )
  {
    uint16_t refint;  // bandgap has 1200 mV, so it should be 2^12/3 > 1000
    //uint16_t temp_adc;
    //static int32_t temp10_z = 0;
  
    /* calculate the reference voltage of the ADC */
    refint = ADCRawValues[2];  // bandgap has 1200 mV, so it should be 4000/3 > 1000
    if ( refint < 100 ) 
      refint=100;
    supplyVoltage = (4095UL*1212UL)/refint;
    
    /* Calculate the min/max values of the hall sensor */
    /* The slow task will copy the min max values to hallMin and hallMax */
    hallRaw = ADCRawValues[3];
    if ( hallRawMin > hallRaw )
      hallRawMin = hallRaw;
    if ( hallRawMax < hallRaw )
      hallRawMax = hallRaw;
    
    /* use hallMin and hallMax, which are drived by hallRawMin/Max but are updated only in the slow task */
    
    if ( rpmCount < 65000 )
      rpmCount++;
    
    switch(rpmState)
    {
      case 0:   /* rpm calculation not yet started Will be actived in slow task */
        break;
      case 1:
        if ( hallRaw > hallAvg + hallDiff4 )
          rpmState = 2;
        break;
      case 2:
        if ( hallRaw <= hallAvg + hallDiff4 )
        {
          rpmState = 3;
        }
        break;
      case 3:
        if ( hallRaw <= hallAvg - hallDiff4 )
        {
          rpmTicksPerRevolution = rpmCount;
          if ( rpmTicksPerRevolution > 0 && rpmTicksPerRevolution <= 60000 )
          {
            freqRaw = 1000/rpmTicksPerRevolution;
            rpmRaw = 60000/rpmTicksPerRevolution;
          }
          else 
          {
            rpmRaw = 0;
          }
          rpmCount = 0;
          rpmState = 1;
        }
        break;
    }
    
    /* restart ADC */
    startADC();
  }
}

void user_slow_task(void)
{
  hallMin = hallRawMin;
  hallMax = hallRawMax;
  hallRawMin = 0xffff;
  hallRawMax = 0;
  hallAvg = (hallMax+hallMin)/2;
  hallDiff = hallMax-hallMin;
  hallDiff4 = hallDiff/4;
  if ( hallDiff4 < 5 )
  {
    freqRaw = 0;
    rpmRaw = 0;
    rpmState = 0;
  }
  else
  {
    if ( hallDiff4 == 0 )
      hallDiff4 = 1;
    if ( rpmState == 0 )  /* if the rpm calculation is not yet started, then start it */
      rpmState = 1;
  }
}

void __attribute__ ((interrupt, used)) SysTick_Handler(void)
{
  static uint32_t start_value;
  
  SysTickClockPeriod = getProcessorClockDelta(start_value);
  start_value = SysTick->VAL;

  SysTickCount++;
  SysTickSchedulerCount++;

  user_fast_task();
  
  if ( (SysTickSchedulerCount & 0x3ff) == 0x3ff )
    user_slow_task();
    
    
  
  SysTickClockUsage = getProcessorClockDelta(start_value);

}

void initSysTick(void)
{
  
  SysTick->LOAD = (SystemCoreClock/1000 * SYS_TICK_HANDLER_MS) -1;
  
  //SysTick->LOAD = 2000*500 *16- 1;
  SysTick->VAL = 0;
  SysTick->CTRL = 7;   /* enable, generate interrupt (SysTick_Handler), do not divide by 2 */
}




/*=======================================================================*/
/* main */
/*=======================================================================*/

static uint8_t usart_buf[32];



int main()
{
  
  //int32_t temp10_z = 0;
  SystemCoreClockUpdate();
  
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;		/* Enable clock for GPIO Port A */
  __NOP();
  __NOP();

  //GPIOA->AFR[0] &= ~(0xf << (3*4));       /* clear alternative function */
  GPIOA->AFR[0] = 0;       /* clear alternative function */
  GPIOA->AFR[1] = 0;       /* clear alternative function */
  
  
  //GPIOA->MODER &= ~GPIO_MODER_MODE3;	/* clear mode for PA3 */
  //GPIOA->MODER |= GPIO_MODER_MODE3_0;	/* Output mode for PA3 */
  //GPIOA->OTYPER &= ~GPIO_OTYPER_OT3;	/* no Push/Pull for PA3 */
  //GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED3;	/* low speed for PA3 */
  //GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD3;	/* no pullup/pulldown for PA3 */
  //GPIOA->BSRR = GPIO_BSRR_BR3;		/* atomic clr PA3 */

  //GPIOA->MODER &= ~GPIO_MODER_MODE5;	/* clear mode for PA5 */
  GPIOA->MODER |= GPIO_MODER_MODE5;	/* analog mode for PA5 */
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD5;	/* no pullup/pulldown for PA5 */
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT5;	/* no Push/Pull for PA5 */

  GPIOA->MODER |= GPIO_MODER_MODE13;	/* analog mode for PA13 */
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD13;	/* no pullup/pulldown for PA13 */
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT13;	/* no Push/Pull for PA13 */

  initSysTick();


  usart1_init(57600, usart_buf, sizeof(usart_buf));


  initADC();            // requires a call to initSysTick(), will set the isADCSetupDone flag



  for(;;)
  {

    
    //usart1_write_string(" ADCEOSCnt=");
    //usart1_write_u32(ADCEOSCnt);

    usart1_write_string(" SysTickClockPeriod=");
    usart1_write_u32(SysTickClockPeriod);
    
    usart1_write_string(" SysTickClockUsage=");
    usart1_write_u32(SysTickClockUsage);
    
    
    
    usart1_write_string(" ADCDuration=");
    usart1_write_u32(ADCDuration);

    

    usart1_write_string(" int temp [raw]=");
    usart1_write_u16(ADCRawValues[1]);

    //usart1_write_string(" band gap [raw]=");
    //usart1_write_u16(ADCRawValues[2]);

    usart1_write_string(" supply [mV]=");
    usart1_write_u16(supplyVoltage);

    usart1_write_string(" hall raw=");
    usart1_write_u16(ADCRawValues[3]);

    usart1_write_string(" hall min/max=");
    usart1_write_u16(hallMin);
    usart1_write_string("/");
    usart1_write_u16(hallMax);


    usart1_write_string("\n");
    
    usart1_write_string(" hall avg/diff=");
    usart1_write_u16(hallAvg);
    usart1_write_string("/");
    usart1_write_u16(hallDiff);

    usart1_write_string(" rpmState=");
    usart1_write_u16(rpmState);

    usart1_write_string(" rpmTicksPerRevolution=");
    usart1_write_u16(rpmTicksPerRevolution);

    usart1_write_string(" freqRaw=");
    usart1_write_u16(freqRaw);
    
    usart1_write_string(" rpmRaw=");
    usart1_write_u16(rpmRaw);
    
    usart1_write_string("\n");
    
    delay_micro_seconds(1000000);
  }
}
