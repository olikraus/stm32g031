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

/* 
  Number of calls to the SysTick handler 
*/
volatile unsigned long SysTickCount = 0;

/* state variable: defines which task to call */
volatile uint16_t SysTickSchedulerCount = 0;

/* processor cycles between calls to systick handler, should be ca. 800000 (=16MHz/20Hz) */
uint32_t SysTickClockUsage = 0;

/* duration of the systick handler */
uint32_t SysTickClockPeriod = 0;

/* a flag, which indicates that the ADC is ready */
int16_t isADC = 0;

/* the voltage applied to the microcontroller (equal to the reference voltage of the ADC) [mV] */
uint16_t supplyVoltage = 0;

/* the voltage provided by the LMT84 temperature sensore [mV] */
uint16_t LMT84Voltage = 0;

/* external temperature in 1/10 degree Celsius */
uint16_t LMT84RawTemperature = 0;

/* external filtered temperature in 1/10 degree Celsius */
uint16_t LMT84Temperature = 0;

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

#define SYS_TICK_HANDLER_MS 50
void initSysTick(void)
{
  
  SysTick->LOAD = (SystemCoreClock/1000 * SYS_TICK_HANDLER_MS) -1;
  
  //SysTick->LOAD = 2000*500 *16- 1;
  SysTick->VAL = 0;
  SysTick->CTRL = 7;   /* enable, generate interrupt (SysTick_Handler), do not divide by 2 */
}


short lmt84_temp_millivolt[201][2] = {
{-50,1299},
{-49,1294},
{-48,1289},
{-47,1284},
{-46,1278},
{-45,1273},
{-44,1268},
{-43,1263},
{-42,1257},
{-41,1252},
{-40,1247},
{-39,1242},
{-38,1236},
{-37,1231},
{-36,1226},
{-35,1221},
{-34,1215},
{-33,1210},
{-32,1205},
{-31,1200},
{-30,1194},
{-29,1189},
{-28,1184},
{-27,1178},
{-26,1173},
{-25,1168},
{-24,1162},
{-23,1157},
{-22,1152},
{-21,1146},
{-20,1141},
{-19,1136},
{-18,1130},
{-17,1125},
{-16,1120},
{-15,1114},
{-14,1109},
{-13,1104},
{-12,1098},
{-11,1093},
{-10,1088},
{-9,1082},
{-8,1077},
{-7,1072},
{-6,1066},
{-5,1061},
{-4,1055},
{-3,1050},
{-2,1044},
{-1,1039},
{0,1034},
{1,1028},
{2,1023},
{3,1017},
{4,1012},
{5,1007},
{6,1001},
{7,996},
{8,990},
{9,985},
{10,980},
{11,974},
{12,969},
{13,963},
{14,958},
{15,952},
{16,947},
{17,941},
{18,936},
{19,931},
{20,925},
{21,920},
{22,914},
{23,909},
{24,903},
{25,898},
{26,892},
{27,887},
{28,882},
{29,876},
{30,871},
{31,865},
{32,860},
{33,854},
{34,849},
{35,843},
{36,838},
{37,832},
{38,827},
{39,821},
{40,816},
{41,810},
{42,804},
{43,799},
{44,793},
{45,788},
{46,782},
{47,777},
{48,771},
{49,766},
{50,760},
{51,754},
{52,749},
{53,743},
{54,738},
{55,732},
{56,726},
{57,721},
{58,715},
{59,710},
{60,704},
{61,698},
{62,693},
{63,687},
{64,681},
{65,676},
{66,670},
{67,664},
{68,659},
{69,653},
{70,647},
{71,642},
{72,636},
{73,630},
{74,625},
{75,619},
{76,613},
{77,608},
{78,602},
{79,596},
{80,591},
{81,585},
{82,579},
{83,574},
{84,568},
{85,562},
{86,557},
{87,551},
{88,545},
{89,539},
{90,534},
{91,528},
{92,522},
{93,517},
{94,511},
{95,505},
{96,499},
{97,494},
{98,488},
{99,482},
{100,476},
{101,471},
{102,465},
{103,459},
{104,453},
{105,448},
{106,442},
{107,436},
{108,430},
{109,425},
{110,419},
{111,413},
{112,407},
{113,401},
{114,396},
{115,390},
{116,384},
{117,378},
{118,372},
{119,367},
{120,361},
{121,355},
{122,349},
{123,343},
{124,337},
{125,332},
{126,326},
{127,320},
{128,314},
{129,308},
{130,302},
{131,296},
{132,291},
{133,285},
{134,279},
{135,273},
{136,267},
{137,261},
{138,255},
{139,249},
{140,243},
{141,237},
{142,231},
{143,225},
{144,219},
{145,213},
{146,207},
{147,201},
{148,195},
{149,189},
{150,183},
};


short getLMT84Temperature(unsigned short millivolt)
{
  int i = 0;
  for( i = 0; i < 200; i++ )
  {
    if ( millivolt >= lmt84_temp_millivolt[i][1] )
      return lmt84_temp_millivolt[i][0];
  }
  return 150;
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

#define ADC_SRC_CNT 3

uint16_t adcRawValues[ADC_SRC_CNT] __attribute__ ((aligned (4)));


void startADC(void)
{
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
  
  ADC1->CFGR1 &= ~ADC_CFGR1_CHSELRMOD;  /* "not fully configurable" mode */
  ADC1->CFGR1 &= ~ADC_CFGR1_SCANDIR;    /* forward scan */
  ADC1->CHSELR = 
    ADC_CHSELR_CHSEL5 |                 /* external temperature sensor */
    ADC_CHSELR_CHSEL12 |                /* internal temperature sensor */
    ADC_CHSELR_CHSEL13 ;                /* internal reference voltage (bandgap) */
  
  while((ADC1->ISR&ADC_ISR_CCRDY) == 0) /* wait until channel config is applied */
  {
  }

  ADC1->CFGR1 &= ~ADC_CFGR1_DISCEN;     /* disable discontinues mode */
  ADC1->CFGR1 |= ADC_CFGR1_CONT;        /* continues mode */
  
  /* DMA CONFIGURATION */
  
  /* How to handle OVR BIT???? */
  ADC1->CFGR1 |= ADC_CFGR1_DMACFG;      /* DMA continues mode */
  ADC1->CFGR1 |= ADC_CFGR1_DMAEN;      /* DMA enable */
  
  /* DMA CHANNEL SETUP */
   
  DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);
  DMA1_Channel1->CMAR = (uint32_t)&(adcRawValues[0]);
  DMA1_Channel1->CNDTR = ADC_SRC_CNT;
  DMA1_Channel1->CCR = 0; /* is it required to clear everything first? */
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


  /*
  DMA1
  DMA1_Channel1
  DMAMUX1
  DMAMUX1_Channel0
  DMAMUX1_RequestGenerator0
  DMAMUX1_ChannelStatus
  DMAMUX1_RequestGenStatus
  */
  
  DMA1_Channel1->CCR |= DMA_CCR_EN;
  
  /* ENABLE ADC */
  
  ADC1->ISR |= ADC_ISR_ADRDY; 			/* clear ready flag */
  ADC1->CR |= ADC_CR_ADEN; 			/* enable ADC */
  while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* wait for ADC */
  {
  }

  ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversion */
  
  isADC = 1;    /* mark ADC as ready */
  
}


/*
  ch 12: temperture sensor
  ch 13: vrefint
  ch 14: vbat
*/
uint16_t x_getADC(uint8_t ch)
{
  unsigned short timeout = 20000;
  //return 0;
  if ( (ADC1->CR & ADC_CR_ADEN)==0 )
  {
    initADC();
    return 0;
  }
  
  /* CONFIGURE ADC */

  ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN;	/* software enabled conversion start */
  ADC1->CFGR1 &= ~ADC_CFGR1_ALIGN;		/* right alignment */
  ADC1->CFGR1 &= ~ADC_CFGR1_RES;		/* 12 bit resolution */
  ADC1->CHSELR = 1<<ch; 				/* Select channel */
  //ADC1->SMPR |= ADC_SMPR_SMP1_0 | ADC_SMPR_SMP1_1 | ADC_SMPR_SMP1_2; /* Select a sampling mode of 111 (very slow)*/
  //ADC1->SMPR |= ADC_SMPR_SMP2_0 | ADC_SMPR_SMP2_1 | ADC_SMPR_SMP2_2; /* Select a sampling mode of 111 (very slow)*/

  ADC1->SMPR &= ~ADC_SMPR_SMP1;
  ADC1->SMPR &= ~ADC_SMPR_SMP2;
  ADC1->SMPR |= ADC_SMPR_SMP1_2; /* Select a sampling mode of 100 (19.6 ADC cycles)*/
  ADC1->SMPR |= ADC_SMPR_SMP2_2; /* Select a sampling mode of 100 (19.6 ADC cycles)*/

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






void task50ms(void)
{
}

void task100ms_0(void)
{
}

void task100ms_1(void)
{
    uint16_t refint;  // bandgap has 1200 mV, so it should be 4000/3 > 1000
    uint16_t temp_adc;
    static int32_t temp10_z = 0;
  
    /* calculate the reference voltage of the ADC */
    refint = adcRawValues[2];  // bandgap has 1200 mV, so it should be 4000/3 > 1000
    if ( refint < 100 ) 
      refint=100;
    supplyVoltage = (4095UL*1212UL)/refint;
    
    /* calculate the voltage, which is sent by the LMT84 temperature sensor */
    temp_adc = adcRawValues[0];
    LMT84Voltage = ((unsigned long)temp_adc*(unsigned long)supplyVoltage)>>12;


    LMT84RawTemperature = getLMT84LinTemp(LMT84Voltage);

    LMT84Temperature = low_pass(&temp10_z, LMT84RawTemperature, 50);
    
    
}


void task1000ms(void)
{
    SysTickSchedulerCount = 0;
    if ( SysTickCount & 1 )
      GPIOA->BSRR = GPIO_BSRR_BS3;		//atomic set PA3 
    else
      GPIOA->BSRR = GPIO_BSRR_BR3;		// atomic clr PA3
}

void __attribute__ ((interrupt, used)) SysTick_Handler(void)
{
  static uint32_t start_value;
  
  SysTickClockPeriod = getProcessorClockDelta(start_value);
  start_value = SysTick->VAL;

  SysTickCount++;
  SysTickSchedulerCount++;
  task50ms();
  if ( (SysTickSchedulerCount & 1) == 0 )
  {
    task100ms_0();
  }
  else
  {
    task100ms_1();
  }

  if ( SysTickSchedulerCount > 20 )
  {
    task1000ms();
  }
  
  SysTickClockUsage = getProcessorClockDelta(start_value);

}





static uint8_t usart_buf[32];

int main()
{
  
  int32_t temp10_z = 0;
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

  // SystemCoreClock
  //SysTick->LOAD = 2000*500 *16- 1;
  //SysTick->VAL = 0;
  //SysTick->CTRL = 7;   /* enable, generate interrupt (SysTick_Handler), do not divide by 2 */


  initSysTick();


  usart1_init(57600, usart_buf, sizeof(usart_buf));


  initADC();            // requires a call to initSysTick()



  for(;;)
  {
    unsigned short refint;
    unsigned short supply;
    unsigned short temp_adc;
    unsigned short temp_millivolt; 
    short temp10;
    //uint32_t start_clock;
    //uint32_t delta_clock;

    //usart1_write_string("CCR=");
    //usart1_write_bits( DMA1_Channel1->CCR, 32);
    //ADC_CHSELR_CHSEL5 |                 /* external temperature sensor */
    //ADC_CHSELR_CHSEL12 |                /* internal temperature sensor */
    //ADC_CHSELR_CHSEL13 |                /* internal reference voltage (bandgap) */

    
    usart1_write_string(" SysTickClockPeriod=");
    usart1_write_u32(SysTickClockPeriod);

    usart1_write_string(" SysTickClockUsage=");
    usart1_write_u32(SysTickClockUsage);

    

    usart1_write_string(" adc temp=");
    usart1_write_u16(adcRawValues[1]);
    


    usart1_write_string(" LMT84Voltage=");
    usart1_write_u16(LMT84Voltage);
    
    usart1_write_string("  ");
    usart1_write_u16(LMT84RawTemperature);
    
    
    usart1_write_string("  LMT84 Temperature [1/10]=");
    usart1_write_u16(LMT84Temperature);
    
    usart1_write_string("\n");
    
    
    
    delay_micro_seconds(1000000);
  }
}
