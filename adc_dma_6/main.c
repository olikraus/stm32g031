/* 

  ADC with DMA for the STM32G031  Project


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
#define ADC_SRC_CNT 3

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

/* the voltage provided by the LMT84 temperature sensore [mV] */
//uint16_t LMT84Voltage = 0;

/* external temperature in 1/10 degree Celsius */
//uint16_t LMT84RawTemperature = 0;

/* external filtered temperature in 1/10 degree Celsius */
//uint16_t LMT84Temperature = 0;

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



/* temperature is returned in 1/10 degree Celsius, 231 = 23.1 degree Celsius */
short getLMT84LinTemp(unsigned short millivolt)
{
  /* 1088 --> -10 --> -100 */
  /* 760 --> 50 --> 500 */
  /*
    V-V1 = (V2-V1)/(T2-T1)*(T-T1)
    T-T1 = (V-V1)*(T2-T1)/(V2-V1)
  
    T2-T1 = 60
    V2-V1 = 760 - 1088 = -328
  
    T = (V - 1088) * 60 / (-328) + (-10) = (1088 - V1)*60 / 328 - 10
    T = (V - 1088) * 60 / (-328) + (-10) = (1088 - V1)*15 / 82 -10
    *10
    T' = (V - 1088) * 60 / (-328) + (-10) = (1088 - V1)*150/ 82 -100
    T' = (V - 1088) * 60 / (-328) + (-10) = (1088 - V1)*75/ 41 -100

    (1088-millivolt)*75  <= 24600 --> 16 bit
  */
  if ( millivolt > 1088 )
    millivolt = 1088;
  if ( millivolt < 760 )
    millivolt = 760;
  return (((1088-millivolt)*75)/41)-100;  
}

/*=======================================================================*/
/* ADC and DMA */
/*=======================================================================*/

/*
  Start ADC conversion sequence in background.
  DMA will copy the data to ADCRawValues array.
  ADC1_IRQHandler will be called at end of sequence.
*/
void startADC(void)
{
  ADCStartTime = SysTick->VAL;          /* store start time of the background sampling activity */
  
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
  ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversion */
}

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
    ADC_CHSELR_CHSEL13 ;                /* internal reference voltage (bandgap) */
  
  while((ADC1->ISR&ADC_ISR_CCRDY) == 0) /* wait until channel config is applied */
  {
  }

  
  /* DMA CONFIGURATION */
  
  /* How to handle OVR BIT???? */
  ADC1->CFGR1 |= ADC_CFGR1_DMACFG;      /* DMA continues mode */
  ADC1->CFGR1 |= ADC_CFGR1_DMAEN;      /* DMA enable */
  
  /* DMA CHANNEL SETUP */
   
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




void user_task(void)
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
    
    /* calculate the voltage, which is sent by the LMT84 temperature sensor */
    //temp_adc = ADCRawValues[0];
    
    /*
    LMT84Voltage = ((unsigned long)temp_adc*(unsigned long)supplyVoltage)>>12;
    LMT84RawTemperature = getLMT84LinTemp(LMT84Voltage);
    LMT84Temperature = low_pass(&temp10_z, LMT84RawTemperature, 50);
    */
    
    /* restart ADC */
    startADC();
  }
  
}


void __attribute__ ((interrupt, used)) SysTick_Handler(void)
{
  static uint32_t start_value;
  
  SysTickClockPeriod = getProcessorClockDelta(start_value);
  start_value = SysTick->VAL;

  SysTickCount++;
  SysTickSchedulerCount++;

  user_task();
  
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

  GPIOA->AFR[0] &= ~(0xf << (3*4));       /* clear alternative function */
  //GPIOA->AFR[0] = 0;       /* clear alternative function */
  
  GPIOA->MODER &= ~GPIO_MODER_MODE3;	/* clear mode for PA3 */
  GPIOA->MODER |= GPIO_MODER_MODE3_0;	/* Output mode for PA3 */
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT3;	/* no Push/Pull for PA3 */
  GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED3;	/* low speed for PA3 */
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD3;	/* no pullup/pulldown for PA3 */
  GPIOA->BSRR = GPIO_BSRR_BR3;		/* atomic clr PA3 */

  //GPIOA->MODER &= ~GPIO_MODER_MODE5;	/* clear mode for PA5 */
  GPIOA->MODER |= GPIO_MODER_MODE5;	/* analog mode for PA5 */
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD5;	/* no pullup/pulldown for PA5 */
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT5;	/* no Push/Pull for PA5 */

  initSysTick();


  usart1_init(57600, usart_buf, sizeof(usart_buf));


  initADC();            // requires a call to initSysTick(), will set the isADCSetupDone flag



  for(;;)
  {
    //unsigned short refint;
    //unsigned short supply;
    //unsigned short temp_adc;
    //unsigned short temp_millivolt; 
    //short temp10;
    //uint32_t start_clock;
    //uint32_t delta_clock;

    //usart1_write_string("ADCISRAfterEOS=");
    //usart1_write_bits( ADCISRAfterEOS, 16);

    
    usart1_write_string(" ADCEOSCnt=");
    usart1_write_u32(ADCEOSCnt);

    usart1_write_string(" SysTickClockPeriod=");
    usart1_write_u32(SysTickClockPeriod);
    
    usart1_write_string(" SysTickClockUsage=");
    usart1_write_u32(SysTickClockUsage);
    
    
    
    usart1_write_string(" ADCDuration=");
    usart1_write_u32(ADCDuration);

    

    usart1_write_string(" int temp [raw]=");
    usart1_write_u16(ADCRawValues[1]);

    usart1_write_string(" band gap [raw]=");
    usart1_write_u16(ADCRawValues[2]);

    usart1_write_string(" supply [mV]=");
    usart1_write_u16(supplyVoltage);

  /*
    usart1_write_string("  ");
    usart1_write_u16(LMT84RawTemperature);
    
    
    usart1_write_string("  LMT84Temperature [1/10]=");
    usart1_write_u16(LMT84Temperature);
    */
    
    usart1_write_string("\n");
    
    
    
    delay_micro_seconds(1000000);
  }
}
