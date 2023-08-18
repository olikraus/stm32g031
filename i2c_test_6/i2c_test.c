
#include "stm32g031xx.h"
#include "delay.h"
#include "sys_util.h"
#include "u8x8.h"

/*=======================================================================*/
/* external functions */
uint8_t u8x8_gpio_and_delay_stm32g0(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);


/*=======================================================================*/
/* global variables */
u8x8_t u8x8;                    // u8x8 object
uint8_t u8x8_x, u8x8_y;         // current position on the screen

volatile unsigned long SysTickCount = 0;

/*=======================================================================*/

void __attribute__ ((interrupt, used)) SysTick_Handler(void)
{
  SysTickCount++;  
}


/*
  Enable several power regions: PWR, GPIOA

  This must be executed after each reset.
*/
void startUp(void)
{  
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;		/* Enable clock for GPIO Port A */
  __NOP();
  __NOP();
  
  //RCC->APB1ENR |= RCC_APB1ENR_PWREN;	/* enable power interface (PWR) */
  //PWR->CR |= PWR_CR_DBP;				/* activate write access to RCC->CSR and RTC */  
  
  SysTick->LOAD = (SystemCoreClock/1000)*50 - 1;   /* 50ms task */
  SysTick->VAL = 0;
  SysTick->CTRL = 7;   /* enable, generate interrupt (SysTick_Handler), do not divide by 2 */      
}

/*=======================================================================*/
/* u8x8 display procedures */

void initDisplay(void)
{
  u8x8_Setup(&u8x8, u8x8_d_ssd1306_128x64_noname, u8x8_cad_ssd13xx_i2c, u8x8_byte_sw_i2c, u8x8_gpio_and_delay_stm32g0);
  u8x8_InitDisplay(&u8x8);
  u8x8_ClearDisplay(&u8x8);
  u8x8_SetPowerSave(&u8x8, 0);
  u8x8_SetFont(&u8x8, u8x8_font_amstrad_cpc_extended_r);  
  u8x8_x = 0;
  u8x8_y = 0;  
}


void outChar(uint8_t c)
{
  if ( u8x8_x >= u8x8_GetCols(&u8x8) )
  {
    u8x8_x = 0;
    u8x8_y++;
  }
  u8x8_DrawGlyph(&u8x8, u8x8_x, u8x8_y, c);
  u8x8_x++;
}

void outStr(const char *s)
{
  while( *s )
    outChar(*s++);
}

void outHexHalfByte(uint8_t b)
{
  b &= 0x0f;
  if ( b < 10 )
    outChar(b+'0');
  else
    outChar(b+'a'-10);
}

void outHex8(uint8_t b)
{
  outHexHalfByte(b >> 4);
  outHexHalfByte(b);
}

void outHex16(uint16_t v)
{
  outHex8(v>>8);
  outHex8(v);
}

void outHex32(uint32_t v)
{
  outHex16(v>>16);
  outHex16(v);
}

void setRow(uint8_t r)
{
  u8x8_x = 0;
  u8x8_y = r;
}


/*==============================================*/
volatile unsigned char i2c_mem[256];     /* contains data, which read or written */
volatile unsigned char i2c_idx = 0;                  /* the current index into i2c_mem */
volatile unsigned char i2c_is_write_idx;                  /* write state... 1: write I2C value to index, 0: write I2C value to memory */

volatile uint16_t i2c_total_irq_cnt = 0;
volatile uint16_t i2c_TXIS_cnt = 0;
volatile uint16_t i2c_RXNE_cnt = 0;


void i2c_mem_reset_write(void)
{
  i2c_is_write_idx = 1;  
}

void i2c_mem_init(void)
{
  i2c_idx = 0;
  i2c_mem_reset_write();
}

void i2c_mem_set_index(unsigned char value)
{
  i2c_idx = value;
  i2c_is_write_idx = 0;
}

void i2c_mem_write_via_index(unsigned char value)
{
  i2c_mem[i2c_idx++] = value;
}

unsigned char i2c_mem_read(void)
{
  i2c_mem_reset_write();
  i2c_idx++;
  return i2c_mem[i2c_idx];
}

void i2c_mem_write(unsigned char value)
{
  if ( i2c_is_write_idx != 0 )
  {
    i2c_mem_set_index(value);
  }
  else
  {
    i2c_is_write_idx = 0;
    i2c_mem_write_via_index(value);
  }
}



/* address: I2C address multiplied by 2 */
/* Pins PA9 (SCL) and PA10 (SDA) */
void i2c_hw_init(unsigned char address)
{
  
  RCC->APBENR1 |= RCC_APBENR1_I2C1EN;		/* Enable clock for I2C */
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;		/* Enable clock for GPIO Port A */
  
    __NOP();                                                          /* extra delay for clock stabilization required? */
    __NOP();


  /* configure io */
  GPIOA->MODER &= ~GPIO_MODER_MODE9;	/* clear mode for PA9 */  
  GPIOA->MODER |= GPIO_MODER_MODE9_1;  /* alt fn */
  GPIOA->OTYPER |= GPIO_OTYPER_OT9;    /* open drain */
  GPIOA->AFR[1] &= ~(15<<4);            /* Clear Alternate Function PA9 */
  GPIOA->AFR[1] |= 6<<4;                   /* AF6: I2C1 SCL Alternate Function PA9 */
  
  GPIOA->MODER &= ~GPIO_MODER_MODE10;	/* clear mode for PA10 */  
  GPIOA->MODER |= GPIO_MODER_MODE10_1;  /* alt fn */
  GPIOA->OTYPER |= GPIO_OTYPER_OT10;    /* open drain */
  GPIOA->AFR[1] &= ~(15<<8);            /* Clear Alternate Function PA10 */
  GPIOA->AFR[1] |= 6<<8;            /* AF6: I2C1 SDA Alternate Function PA10 */
  
  
  RCC->CCIPR &= ~RCC_CCIPR_I2C1SEL;                      /* write 00 to the I2C clk selection register */
  RCC->CCIPR |= RCC_CCIPR_I2C1SEL_1;                      /* select HSI16 clock (10) for I2C1 */


  /* I2C init flow chart: Clear PE (Peripheral Enable) bit so that we can change the I2C1 configuration */
  
  I2C1->CR1 &= ~I2C_CR1_PE;             


  /* I2C init flow chart: Configure filter */
  
  I2C1->CR1 &= ~I2C_CR1_ANFOFF;          /* 0: Analog Filter enabled, suppres spikes < 50ns */
  I2C1->CR1 &= ~I2C_CR1_DNF;            /* disable digital noise filter */


  /* I2C init flow chart: Configure timing */
  /*
    standard mode 100kHz configuration (see datasheet table 169)
    HSI16 = I2CCLK = 16 MHz
    PRESC = 3           bits 28..31
    SCLDEL = 0x04       bits 20..23
    SDADEL = 0x02       bits 16..19
    SCLH = 0x0f         bits 8..15
    SCLL = 0x13         bits 0..7
  */
  I2C1->TIMINGR = 0x30420f13;

  /* I2C init flow chart: Configure NOSTRECH */
  
  I2C1->CR1 |= I2C_CR1_NOSTRETCH;

  /* I2C init flow chart: Enable I2C */
  
  I2C1->CR1 |= I2C_CR1_PE;

  /* disable OAR1 for reconfiguration */
  I2C1->OAR1 &= ~I2C_OAR1_OA1EN;
  
  /* assign the addres (and also set 7-bit address mode */
  I2C1->OAR1 = address;
  
  /* enable interrupts */
  I2C1->CR1 |= I2C_CR1_STOPIE;
  I2C1->CR1 |= I2C_CR1_NACKIE;
  //I2C1->CR1 |= I2C_CR1_ADDRIE;
  I2C1->CR1 |= I2C_CR1_RXIE;
  I2C1->CR1 |= I2C_CR1_TXIE;

  I2C1->CR1 |= I2C_CR1_ERRIE;

  /* load first value into TXDR register */
  I2C1->TXDR = i2c_mem[i2c_idx];

  /* enabel OAR1 */
  I2C1->OAR1 |= I2C_OAR1_OA1EN;

  /* enable IRQ in NVIC */
  NVIC_SetPriority(I2C1_IRQn, 0);
  NVIC_EnableIRQ(I2C1_IRQn);
}

void i2c_init()
{
  i2c_mem_init();
  i2c_hw_init(7*2);
}


void __attribute__ ((interrupt, used)) I2C1_IRQHandler(void)
{
  unsigned long isr = I2C1->ISR;

  i2c_total_irq_cnt ++;
  
  if ( isr & I2C_ISR_TXIS )     // Transmit Buffer Interrupt Status. Clear: Write to TXDR
  {
    I2C1->TXDR = i2c_mem_read();
    i2c_TXIS_cnt++;
  }
  else if ( isr & I2C_ISR_RXNE )        // Recevie Buffer not empty
  {
    i2c_mem_write(I2C1->RXDR);  // Write the received byte to memory: It can be the adr index or the data into memory
    I2C1->ISR |= I2C_ISR_TXE;           // allow overwriting the TCDR with new data
    I2C1->TXDR = i2c_mem[i2c_idx];  // In case master changes to read from client, put the next memory value into the TX register
    i2c_RXNE_cnt++;
  }
  else if ( isr & I2C_ISR_STOPF )
  {
    I2C1->ICR = I2C_ICR_STOPCF;
    I2C1->ISR |= I2C_ISR_TXE;           // allow overwriting the TCDR with new data
    I2C1->TXDR = i2c_mem[i2c_idx];
    i2c_mem_reset_write();
  }
  else if ( isr & I2C_ISR_NACKF )
  {
    I2C1->ICR = I2C_ICR_NACKCF;
    I2C1->ISR |= I2C_ISR_TXE;           // allow overwriting the TCDR with new data
    I2C1->TXDR = i2c_mem[i2c_idx];
    i2c_mem_reset_write();
  }
  else if ( isr & I2C_ISR_ADDR )
  {
    /* not required, the addr match interrupt is not enabled */
    I2C1->ICR = I2C_ICR_ADDRCF;
    I2C1->ISR |= I2C_ISR_TXE;           // allow overwriting the TCDR with new data
    I2C1->TXDR = i2c_mem[i2c_idx];
    i2c_mem_reset_write();
  }
 
  /* if at any time the addr match is set, clear the flag */
  /* not sure, whether this is required */
  if ( isr & I2C_ISR_ADDR )
  {
    I2C1->ICR = I2C_ICR_ADDRCF;
  }
  
  /* check & clear also for some other flags */
  if ( isr & I2C_ISR_BERR )
  {
    I2C1->ICR = I2C_ICR_BERRCF;
    i2c_mem_reset_write();
  }
  if ( isr & I2C_ISR_ARLO )
  {
    I2C1->ICR = I2C_ICR_ARLOCF;
    i2c_mem_reset_write();
  }
  if ( isr & I2C_ISR_OVR )
  {
    I2C1->ICR = I2C_ICR_OVRCF;
    i2c_mem_reset_write();
  }
    
}


/*==============================================*/

int main()
{
  SystemCoreClockUpdate();
  set64MHzSysClk();
  startUp();
  initDisplay();          /* aktivate display */
  i2c_init();

  __enable_irq();
  
  setRow(0); outStr("x Hello World!"); 
  

  
  for(;;)
  {
    i2c_mem[0] = SysTickCount & 255;
    i2c_mem[1] = (SysTickCount>>8) & 255;
    
    setRow(2); outHex32(SysTickCount); 
    setRow(3); outHex16(i2c_total_irq_cnt);
    setRow(4); outHex16(i2c_TXIS_cnt); outStr(" "); outHex16(i2c_RXNE_cnt);
    setRow(5); outStr("I2C_ISR:"); outHex32(I2C1->ISR);
    setRow(6); outStr("idx:    "); outHex8(i2c_idx);
    setRow(7); outHex8(i2c_mem[0]); outStr(" "); outHex8(i2c_mem[1]); outStr(" "); outHex8(i2c_mem[2]);
    
  }
  
  
}
