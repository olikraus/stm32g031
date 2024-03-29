/* 

  ADC test for the STM32G031  Project

  Configuration is 115200 8-N-1. Only "\n" is sent. Receiving terminal should add \r
  (e.g. use add CR function in "minicom")
  

  Linux:
    stty -F /dev/ttyUSB0 sane 115200 && cat /dev/ttyUSB0
    or stty -F /dev/ttyUSB0 sane 115200 igncr  && cat /dev/ttyUSB0
    screen /dev/ttyUSB0  115200 (terminate with "C-a k" or "C-a \")
    minicom -D /dev/ttyUSB0  -b 115200 (terminate with "C-a x", change CR mode: "C-a u", disable HW control flow!)
    

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


void __attribute__ ((interrupt, used)) SysTick_Handler(void)
{
  SysTickCount++;  
  
  if ( SysTickCount & 1 )
    GPIOA->BSRR = GPIO_BSRR_BS3;		//atomic set PA3 
  else
    GPIOA->BSRR = GPIO_BSRR_BR3;		// atomic clr PA3
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

void initADC(void)
{
  short i;
  //__disable_irq();
  
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
  
  
  for( i = 0; i < 48; i++ )
    __NOP();								/* not sure why, but some nop's are required here, at least 8 of them with 16MHz */

  /* ENABLE ADC */
  
  ADC1->ISR |= ADC_ISR_ADRDY; 			/* clear ready flag */
  ADC1->CR |= ADC_CR_ADEN; 			/* enable ADC */
  while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* wait for ADC */
  {
  }
  
  
}


/*
  ch 12: temperture sensor
  ch 13: vrefint
  ch 14: vbat
*/
uint16_t getADC(uint8_t ch)
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

  SysTick->LOAD = 2000*500 *16- 1;
  SysTick->VAL = 0;
  SysTick->CTRL = 7;   /* enable, generate interrupt (SysTick_Handler), do not divide by 2 */
 

  usart1_init(57600, usart_buf, sizeof(usart_buf));


  initADC();



  for(;;)
  {
    unsigned short refint;
    unsigned short supply;
    unsigned short temp_adc;
    unsigned short temp_millivolt; 
    short temp10;
    uint32_t start_clock;
    uint32_t delta_clock;

    //usart1_write_string("opt=");
    //usart1_write_bits( SYSCFG->CFGR1 , 32);


    usart1_write_string(" adc temp=");
    usart1_write_u16(getADC(12));
    

    start_clock = SysTick->VAL;
    refint = getADC(13);
    delta_clock = getProcessorClockDelta(start_clock);
    usart1_write_string(" refint=");
    usart1_write_u16(refint);
    usart1_write_string(" adc_clocks=");
    usart1_write_u16(delta_clock);


    if ( refint < 100 ) 
      refint=100;
    supply = (4095UL*1212UL)/refint;
    
    usart1_write_string(" supply=");
    usart1_write_u16(supply);
    //usart1_write_string(" PA4=");
    //usart1_write_u16(getADC(4));
    usart1_write_string(" PA5=");
    temp_adc = getADC(5);
    usart1_write_u16(temp_adc);

    temp_millivolt = ((unsigned long)temp_adc*(unsigned long)supply)>>12;
    usart1_write_string(" volt=");
    usart1_write_u16(temp_millivolt);

    usart1_write_string(" temperature=");
    usart1_write_u16(getLMT84Temperature(temp_millivolt));
    usart1_write_string("  ");
    
    temp10 = getLMT84LinTemp(temp_millivolt);
    usart1_write_u16(temp10);
    usart1_write_string("  ");
    temp10 = low_pass(&temp10_z, temp10, 50);
    usart1_write_u16(temp10);
    usart1_write_string("\n");
    
    delay_micro_seconds(1000000);
  }
}
