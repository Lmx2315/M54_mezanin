#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"Transf
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_tim.h"
#include "onewire.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#include "system_stm32f10x.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_dma.h"
//#include "systick.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
//-----------------------------------------------------------------------


#define  u8  unsigned char
#define  u16 unsigned short
#define  u32 unsigned int

#define  u8  unsigned char
#define  int8_t   char
#define  int16_t  short


#define  ON  1
#define  OFF 0

#define adress_clock 0x68
#define adress_led   0x25

      GPIO_InitTypeDef  GPIO_InitStructure;  //объявляем структуры ножек GPIO

      USART_InitTypeDef USART1_InitStructure; //объявляем структуры UART
      USART_InitTypeDef USART2_InitStructure;

      ADC_InitTypeDef ADC_init_struct;        //объявляем структуру ADC

  //  TIM_TimeBaseInitTypeDef Timer1;         //объявляем структуру Timer
     



#define PE0_0  GPIOE->BRR = GPIO_Pin_0
#define PE0_1  GPIOE->BSRR= GPIO_Pin_0

#define PE1_0  GPIOE->BRR = GPIO_Pin_1
#define PE1_1  GPIOE->BSRR= GPIO_Pin_1

#define PE2_0  GPIOE->BRR = GPIO_Pin_2
#define PE2_1  GPIOE->BSRR= GPIO_Pin_2

#define PE3_0  GPIOE->BRR = GPIO_Pin_3
#define PE3_1  GPIOE->BSRR= GPIO_Pin_3

#define PE4_0  GPIOE->BRR = GPIO_Pin_4
#define PE4_1  GPIOE->BSRR= GPIO_Pin_4

#define PE5_0  GPIOE->BRR = GPIO_Pin_5
#define PE5_1  GPIOE->BSRR= GPIO_Pin_5

#define PE6_0  GPIOE->BRR = GPIO_Pin_6
#define PE6_1  GPIOE->BSRR= GPIO_Pin_6

#define PE7_0  GPIOE->BRR = GPIO_Pin_7
#define PE7_1  GPIOE->BSRR= GPIO_Pin_7

#define PE8_0  GPIOE->BRR = GPIO_Pin_8
#define PE8_1  GPIOE->BSRR= GPIO_Pin_8

#define PE9_0  GPIOE->BRR = GPIO_Pin_9
#define PE9_1  GPIOE->BSRR= GPIO_Pin_9

#define PE10_0  GPIOE->BRR = GPIO_Pin_10
#define PE10_1  GPIOE->BSRR= GPIO_Pin_10

#define PE11_0  GPIOE->BRR = GPIO_Pin_11
#define PE11_1  GPIOE->BSRR= GPIO_Pin_11

#define PE12_0  GPIOE->BRR = GPIO_Pin_12
#define PE12_1  GPIOE->BSRR= GPIO_Pin_12

#define PE13_0  GPIOE->BRR = GPIO_Pin_13
#define PE13_1  GPIOE->BSRR= GPIO_Pin_13

#define PE14_0  GPIOE->BRR = GPIO_Pin_14
#define PE14_1  GPIOE->BSRR= GPIO_Pin_14

#define PE15_0  GPIOE->BRR = GPIO_Pin_15
#define PE15_1  GPIOE->BSRR= GPIO_Pin_15

#define PC13_0 GPIOC->BRR = GPIO_Pin_13
#define PC13_1 GPIOC->BSRR= GPIO_Pin_13

#define PD8_0  GPIOD->BRR = GPIO_Pin_8
#define PD8_1  GPIOD->BSRR= GPIO_Pin_8

#define PD9_0  GPIOD->BRR = GPIO_Pin_9
#define PD9_1  GPIOD->BSRR= GPIO_Pin_9

#define PD10_0  GPIOD->BRR = GPIO_Pin_10
#define PD10_1  GPIOD->BSRR= GPIO_Pin_10

#define PD11_0  GPIOD->BRR = GPIO_Pin_11
#define PD11_1  GPIOD->BSRR= GPIO_Pin_11

#define PD12_0  GPIOD->BRR = GPIO_Pin_12
#define PD12_1  GPIOD->BSRR= GPIO_Pin_12

#define PD13_0  GPIOD->BRR = GPIO_Pin_13
#define PD13_1  GPIOD->BSRR= GPIO_Pin_13

#define PD14_0 GPIOD->BRR = GPIO_Pin_14
#define PD14_1 GPIOD->BSRR= GPIO_Pin_14

#define PD15_0 GPIOD->BRR = GPIO_Pin_15
#define PD15_1 GPIOD->BSRR= GPIO_Pin_15

#define PC6_0  GPIOC->BRR = GPIO_Pin_6
#define PC6_1  GPIOC->BSRR= GPIO_Pin_6

#define PC7_0  GPIOC->BRR = GPIO_Pin_7
#define PC7_1  GPIOC->BSRR= GPIO_Pin_7

#define PC8_0  GPIOC->BRR = GPIO_Pin_8
#define PC8_1  GPIOC->BSRR= GPIO_Pin_8

#define PC9_0  GPIOC->BRR = GPIO_Pin_9
#define PC9_1  GPIOC->BSRR= GPIO_Pin_9

#define PC10_0 GPIOC->BRR = GPIO_Pin_10
#define PC10_1 GPIOC->BSRR= GPIO_Pin_10

#define PA8_0  GPIOA->BRR  = GPIO_Pin_8
#define PA8_1  GPIOA->BSRR = GPIO_Pin_8

#define PA11_0  GPIOA->BRR  = GPIO_Pin_11
#define PA11_1  GPIOA->BSRR = GPIO_Pin_11   

#define PA12_0  GPIOA->BRR  = GPIO_Pin_12
#define PA12_1  GPIOA->BSRR = GPIO_Pin_12         

#define PA15_0  GPIOA->BRR  = GPIO_Pin_15
#define PA15_1  GPIOA->BSRR = GPIO_Pin_15  

#define PD0_0  GPIOD->BRR  = GPIO_Pin_0
#define PD0_1  GPIOD->BSRR = GPIO_Pin_0 

#define PD2_0  GPIOD->BRR  = GPIO_Pin_2
#define PD2_1  GPIOD->BSRR = GPIO_Pin_2

#define PD3_0  GPIOD->BRR   = GPIO_Pin_3
#define PD3_1  GPIOD->BSRR  = GPIO_Pin_3

#define PD4_0  GPIOD->BRR   = GPIO_Pin_4
#define PD4_1  GPIOD->BSRR  = GPIO_Pin_4

#define PD5_0  GPIOD->BRR   = GPIO_Pin_5
#define PD5_1  GPIOD->BSRR  = GPIO_Pin_5

#define PD6_0  GPIOD->BRR   = GPIO_Pin_6
#define PD6_1  GPIOD->BSRR  = GPIO_Pin_6

#define PD7_0  GPIOD->BRR   = GPIO_Pin_7
#define PD7_1  GPIOD->BSRR  = GPIO_Pin_7

#define PD12_0  GPIOD->BRR   = GPIO_Pin_12
#define PD12_1  GPIOD->BSRR  = GPIO_Pin_12      

#define PD10_0  GPIOD->BRR   = GPIO_Pin_10
#define PD10_1  GPIOD->BSRR  = GPIO_Pin_10 

#define PD11_0  GPIOD->BRR   = GPIO_Pin_11
#define PD11_1  GPIOD->BSRR  = GPIO_Pin_11 

#define PD14_0  GPIOD->BRR   = GPIO_Pin_14
#define PD14_1  GPIOD->BSRR  = GPIO_Pin_14 

#define PD15_0  GPIOD->BRR   = GPIO_Pin_15
#define PD15_1  GPIOD->BSRR  = GPIO_Pin_15 

#define PB8_0  GPIOB->BRR   = GPIO_Pin_8
#define PB8_1  GPIOB->BSRR  = GPIO_Pin_8

#define PB9_0  GPIOB->BRR   = GPIO_Pin_9
#define PB9_1  GPIOB->BSRR  = GPIO_Pin_9

#define PB12_0  GPIOB->BRR   = GPIO_Pin_12
#define PB12_1  GPIOB->BSRR  = GPIO_Pin_12

#define CS_AT25_0  PD8_0
#define CS_AT25_1  PD8_1

#define CS_AT45_0  PA12_0
#define CS_AT45_1  PA12_1

#define A_LE_WIRE_LMK_0  PD2_0
#define A_LE_WIRE_LMK_1  PD2_1

#define B_LE_WIRE_LMK_0  PD3_0
#define B_LE_WIRE_LMK_1  PD3_1

#define C_LE_WIRE_LMK_0  PD4_0
#define C_LE_WIRE_LMK_1  PD4_1

#define A_GOE_0  PD5_0
#define A_GOE_1  PD5_1

#define B_GOE_0  PD6_0
#define B_GOE_1  PD6_1

#define C_GOE_0  PD7_0
#define C_GOE_1  PD7_1

#define LE_A_ATT_0  PB8_0
#define LE_A_ATT_1  PB8_1

#define LE_B_ATT_0  PB9_0
#define LE_B_ATT_1  PB9_1

#define LE_C_ATT_0  PD11_0
#define LE_C_ATT_1  PD11_1

#define LE_D_ATT_0  PD15_0
#define LE_D_ATT_1  PD15_1


#define UPR_A2_0  PA8_0
#define UPR_A2_1  PA8_1

#define SYNC_LMK_0  PC7_0;
#define SYNC_LMK_1  PC7_1;

#define SEL_ETALON_0  PD12_0
#define SEL_ETALON_1  PD12_1

#define CSB_A_ADC_0  PD9_0
#define CSB_A_ADC_1  PD9_1

#define RTS_485_0  PE1_0
#define RTS_485_1  PE1_1

#define RE_485_0  PE0_0
#define RE_485_1  PE0_1

#define DE2RX_RS422_0  PB12_0
#define DE2RX_RS422_1  PB12_1

#define DE1RX_RS422_0  PE7_0
#define DE1RX_RS422_0  PE7_1

#define PWRDN_A_0  PE2_0
#define PWRDN_A_1  PE2_1

#define PWRDN_B_0  PE3_0
#define PWRDN_B_1  PE3_1

#define PWRDN_C_0  PE4_0
#define PWRDN_C_1  PE4_1

#define PWRDN_D_0  PE5_0
#define PWRDN_D_1  PE5_1

#define OK_on   PE7_1
#define OK_off  PE7_0

#define WDG_on   PA11_0
#define WDG_off  PA11_1

#define LED1_on   PE6_1
#define LED1_off  PE6_0

#define DATA_MK0_0   PE8_0
#define DATA_MK0_1   PE8_1

#define DATA_MK1_0   PE9_0
#define DATA_MK1_1   PE9_1

#define DATA_MK2_0   PE10_0
#define DATA_MK2_1   PE10_1

#define DATA_MK3_0   PE11_0
#define DATA_MK3_1   PE11_1

#define DATA_MK4_0   PE12_0
#define DATA_MK4_1   PE12_1

#define DATA_MK5_0   PE13_0
#define DATA_MK5_1   PE13_1

#define DATA_MK6_0   PE14_0
#define DATA_MK6_1   PE14_1

#define DATA_MK7_0   PE15_0
#define DATA_MK7_1   PE15_1

#define FPGA_CS_0 PE14_0
#define FPGA_CS_1 PE14_1

static volatile uint32_t __counter;
static volatile uint32_t led_counter;

unsigned char flag_uart1=0;
unsigned char flag_uart2=0;

unsigned char level_5MHz;  //проверка уровня сигнала CONTR_KL_64MHz
unsigned char level_3V3; //проверка уровня сигнала CONTR_KL_392MHz

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// This flag is set on USART1 Receiver buffer overflow
char  rx_buffer_overflow1;

// This flag is set on USART2 Receiver buffer overflow
char  rx_buffer_overflow2;

// USART1 Receiver buffer
#define RX_BUFFER_SIZE1 256
char          rx_buffer1[RX_BUFFER_SIZE1];
unsigned char rx_wr_index1,rx_rd_index1,rx_counter1;

// USART2 Receiver buffer
#define RX_BUFFER_SIZE2 256
char          rx_buffer2[RX_BUFFER_SIZE1];
unsigned char rx_wr_index2,rx_rd_index2,rx_counter2;

//*********************************************************

unsigned char s1[32];
unsigned l1= 5;
unsigned char s2[32];
unsigned l2=6;
unsigned char s3[32];
unsigned l3=7;
unsigned char sr[128];
unsigned char lsr=0;
unsigned char lk;

unsigned char s4[32]={0xef,0xf0,0xe8,0xed,0xff,0xeb};

unsigned char ok[5];
unsigned int led_tick;

unsigned char  strng[32];//



        char lsym;
        char  sym;
        char flag;
        char packet_flag;
        char    NB;
        char Adress=0x30;  //
        char packet_sum;
        char crc,comanda;
        char InOut[64];
        char In_data[64];
        char Word[64];         //
        char DATA_Word[64];    //
        char crc_ok;
        char packet_ok;
        char ink1; //
        char data_in;
        char index_word=0;
        char index_data_word=0;
        char data_flag=0;


        float time_uart; //
        unsigned char flag_pcf=0;

   char index1=0;
   char lsym1=0;
   char pack_ok1=0;
   char pack_sum1=0;
   char sym1=0;

   char status_foch=0;

   
   float Lvl_3V3;   //
   float Lvl_2v5;
   float Lvl_1v8;
   float Lvl_1v5;
   float Lvl_1v0;
   float Lvl_1v0_MGTVCC;
   float Lvl_1v2_MGTVCC;
   float Lvl_VTTREF;
   float Lvl_VTTDDR;
   
float RF_5MHz;

float Temp_stm32;
float U_stm32;

unsigned short Sys_control_health=0;


   char event1,event1_1;
   char event2,event2_1;

   int time1=0;
   int time2=0;

   int time_led1=0;
   int time_led2=0;

   unsigned int time=0;
   unsigned long time_UPr = 0;
   unsigned long time_Temp_DMA = 0;
   char time_wdg=0;
   char SYS_terminal=1;

   float DBm[330];

   u8 flag_life_block=1;

   u8 flag_step    =0;
   u32 TIMER_SCR=0;
  
unsigned char devices;
unsigned int timer_INIT_FAPCH1=0;

int SysTickDelay;  
int tick_wait_LED_Zahvat=0;

unsigned char flag_ADC=0;
unsigned int  DataConv=0;

unsigned char flag_INT=0;


unsigned char state_INT7 =0;
unsigned char state_INT8 =0;
unsigned char state_INT9 =0;
unsigned char state_INT10=0;
unsigned char state_INT11=0;
unsigned char state_INT12=0;
unsigned char state_INT13=0;
unsigned char state_INT14=0;
unsigned char state_INT15=0;
unsigned char INT_var=0;

u8 FLAG_FPGA_DONE;
u8 FLAG_FPGA_INIT;
u8 FLAG_A_LOCK_DETECT;
u8 FLAG_B_LOCK_DETECT;

u8 RUN_FLAG=0;    //флаг означающий включение кассеты
u8 RUN_FLAG_ON=0; //флаг означающий что включение исполненно

float Lvl_f5MHz_in=0;
float Lvl_3v3=0;


u16 timer_start_init=0; //счётчик времени инициализации системы
u16 timer_Temp=0;      //счётчик интервалов между измерениями температуры
u8  flag_lcd_init=0;     // флаг инициализации LCD
u16 timer_clock_watcher=0;

//---------------------------------------------------------------------------------
 struct reg_lmk03806   // объявляю структуру 
{
	u32 R0;
	u32 R1;
	u32 R2;
	u32 R3;
	u32 R4;
	u32 R5;
	u32 R6;
	u32 R7;
	u32 R8;
	u32 R9;
	u32 R10;
	u32 R11;
	u32 R12;
	u32 R13;
	u32 R14;
	u32 R16;
	u32 R24;
	u32 R26;
	u32 R28;
	u32 R29;
	u32 R30;
	u32 R31;
};
struct reg_lmk03806 B_stz;

struct R0_5_03806
{
	u8  ADR:5;
	u16 DIV:11;
	u8  PD :1;
	u8  RST:1;
};

struct R0_5_03806 R0B,R1B,R2B,R3B,R4B,R5B;

struct R6_8_03806
{
	u8  ADR:5;
	u8  TYPE0:4;
	u8  TYPE1:4;
	u8  TYPE2:4;
	u8  TYPE3:4;
};

struct R6_8_03806 R6B,R7B,R8B;

struct R9_03806
{
	u8  ADR:5;

};
struct R9_03806 R9B;

struct R10_03806
{
	u8  ADR  :5;
	u8  TYPE0:2;
	u8  TYPE1:4;
	u8  EN1  :1;
	u8  EN0  :1;
	u8  MUX1 :1;
	u8  MUX0 :1;
	u8  DIV  :3;
};
struct R10_03806 R10B;

struct R11_03806
{
	u8  ADR  :5;
	u8  NO_SYNC1    :1;
	u8  NO_SYNC3    :1;
	u8  NO_SYNC5    :1;
	u8  NO_SYNC7    :1;
	u8  NO_SYNC9    :1;
	u8  NO_SYNC11   :1;
	u8  SYNC_POL_INV:1;
	u8  SYNC_TYPE   :2;
	u8  EN_PLL_XTAL :1;
};

struct R11_03806 R11B;


struct R12_03806
{
	u8  ADR:5;
	u8 SYNC_PLL_DLD :1;
	u8  LD_TYPE     :3;
	u8  LD_MUX      :5;
};

struct R12_03806 R12B;

struct R13_03806
{
	u8  ADR:5;
	u8  GPout0         :3;
	u8  READBACK_TYPE  :3;
};
struct R13_03806 R13B;

struct R14_03806
{
	u8  ADR:5;
	u8 GPout1 :3;
};
struct R14_03806 R14B;

struct R16_03806
{
	u8  ADR:5;

};
struct R16_03806 R16B;

struct R24_03806
{
	u8  ADR:5;
	u8 PLL_R3_LF :3;
	u8 PLL_R4_LF :3;
	u8 PLL_C3_LF :4;
	u8 PLL_C4_LF :4;
};
struct R24_03806 R24B;

struct R26_03806
{
	u8  ADR:5;
	u16 PLL_DLD_CNT  :14;
	u8 PLL_CP_GAIN   :2;
	u8 EN_PLL_REF_2X :1;
};
struct R26_03806 R26B;

struct R28_03806
{
	u8  ADR:5;
	u16 PLL_R   :12;
};
struct R28_03806 R28B;


struct R29_03806
{
	u8  ADR:5;
	u8  OSCin_FREQ :3;
	u32 PLL_N_CAL  :18;
};
struct R29_03806 R29B;


struct R30_03806
{
	u8  ADR:5;
	u8  PLL_P :3;
	u32 PLL_N :18;
};
struct R30_03806 R30B;

struct R31_03806
{
	u8  ADR:5;
	u8 READBACK_ADDR :5;
	u8 uWire_LOCK    :1;
};
struct R31_03806 R31B;



//------------------------------------------------------------------------------

 struct reg_lmk03000   // объявляю структуру 
{
	u32 R0;
	u32 R1;
	u32 R2;
	u32 R3;
	u32 R4;
	u32 R5;
	u32 R6;
	u32 R7;
	u32 R8;
	u32 R9;
	u32 R11;
	u32 R13;
	u32 R14;
	u32 R15;
};

struct reg_lmk03000 A_stz;

struct R_STRUK
{
	u8  ADR:4;
	u8  DLY:4;
	u8  DIV:8;
	u8  EN :1;
	u8  MUX:1;
	u8  RST:1;
};

struct R11_STRUK
{
	u8  ADR :4;
	u8  DIV4:1;
};

struct R13_STRUK
{
	u8  ADR      :4;
	u8  VCO_C3_C4:4;
	u8  VCO_R3   :3;
	u8  VCO_R4   :3;
	u8  FREQ     :8;
};

struct R14_STRUK
{
	u8   ADR      :4;
	u16  PLL_R    :12;
	u8   MUX      :4;
	u8   PWRDN    :1;
	u8   EN_GLOBAL:1;
	u8   EN_FOUT  :1;
};

struct R15_STRUK
{
	u8    ADR       :4;
	u8   PLL_CP_GAIN:2;
	u8    VCO_DIV   :4;
	u32   PLL_N     :19;
};

struct R_STRUK   R0,R1,R2,R3,R4,R5,R6,R7;
struct R11_STRUK R11;
struct R13_STRUK R13;
struct R14_STRUK R14;
struct R15_STRUK R15;

//-----------прототипы---------------------------

u32 FPGA_SPI (u8 );


//-----------------------------------------------

void init_RCC (void)
{

    // Cогласно документации необходимо разрешить тактирование 
    // AFIO (альтернативные функции линий ввода-вывода), так как регистры 
    // управления мультиплексорами находится в данном модуле:

 /* Initialize Enable the Clock*/
  RCC_APB2PeriphClockCmd(
             RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB
            |RCC_APB2Periph_GPIOC |RCC_APB2Periph_GPIOD |RCC_APB2Periph_GPIOE
            |RCC_APB2Periph_USART1|RCC_APB2Periph_TIM1  |RCC_APB2ENR_AFIOEN  , ENABLE);//|RCC_APB2Periph_AFIO
 // RCC_APB1PeriphClockCmd(
       //      RCC_APB1Periph_USART2             
    //        |RCC_APB1Periph_USART3, ENABLE);

   SysTick_Config(SystemCoreClock /1000);//1ms 

  //  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  //  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  //  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  //  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

}

//------------------------------------------------------------------------  
void Delay( unsigned int Val)  
{  
   SysTickDelay = Val;  
   while (SysTickDelay != 0) {};  
}  
  
//------------------------------------------------------------------------  
u8 flag_Temp_convert;
u8 time_Temp_convert;
void SysTick_Handler(void)  
{  
   if (SysTickDelay != 0)  
   {  
      SysTickDelay--;  
   }  
    TIMER_SCR++; //таймер вывода на экран
	++timer_clock_watcher;
	++timer_Temp;       //счётчик интервалов между измерениями температуры
	++timer_start_init; //счётчик времени инициализации системы
  	++__counter;
    ++time;
    ++time_UPr;
    ++time_wdg;      //таймер вочдога
    ++time_Temp_DMA; //таймер функций термометра
    ++time_led1;
    ++time_led2;
    ++tick_wait_LED_Zahvat; //таймер светодиода индикации захвата
    timer_INIT_FAPCH1++;
    if (flag_Temp_convert==1) ++time_Temp_convert; //считаем время измерения температуры
}  


void init_EXT(void)

{

  EXTI_InitTypeDef EXTI_InitStructure;  //объявляем структуру для внешних прерываний      
 
/*
 GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource11); //прерывание с порта D ножки 11

EXTI_InitStructure.EXTI_Line = EXTI_Line11;
EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  //EXTI_Trigger_Falling
EXTI_InitStructure.EXTI_LineCmd = ENABLE;
EXTI_Init(&EXTI_InitStructure);

 NVIC_SetPriority(EXTI15_10_IRQn, 2);

  NVIC_EnableIRQ(EXTI15_10_IRQn);

*/

EXTI_InitStructure.EXTI_Line    = EXTI_Line7|EXTI_Line8|EXTI_Line9;
EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling ;  // EXTI_Trigger_Rising
EXTI_InitStructure.EXTI_LineCmd = ENABLE;
EXTI_Init(&EXTI_InitStructure);

GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource7|GPIO_PinSource8|GPIO_PinSource9); //прерывание с порта 

  NVIC_SetPriority(EXTI9_5_IRQn, 2);

  NVIC_EnableIRQ(EXTI9_5_IRQn);
 

EXTI_InitStructure.EXTI_Line = EXTI_Line10|EXTI_Line11|EXTI_Line12|EXTI_Line13|EXTI_Line14|EXTI_Line15;
EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
EXTI_InitStructure.EXTI_Trigger =EXTI_Trigger_Falling ;  // EXTI_Trigger_Rising
EXTI_InitStructure.EXTI_LineCmd = ENABLE;
EXTI_Init(&EXTI_InitStructure);

GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource10|GPIO_PinSource11|GPIO_PinSource12|GPIO_PinSource13|GPIO_PinSource14|GPIO_PinSource14); //прерывание с порта D ножки 11

  NVIC_SetPriority(EXTI15_10_IRQn, 2);

  NVIC_EnableIRQ(EXTI15_10_IRQn);
   

}


void EXTI9_5_IRQHandler (void)
{ 
  
          if (EXTI_GetITStatus(EXTI_Line7))
          { 
           
             EXTI_ClearITPendingBit(EXTI_Line7);
        //    sendT("принято прерывание! INT7\r\n");
             
           state_INT7=1;

          }

            if (EXTI_GetITStatus(EXTI_Line8))
          { 
          
             EXTI_ClearITPendingBit(EXTI_Line8);
         //    sendT("принято прерывание! INT8\r\n");   
           state_INT8=1;

          }

            if (EXTI_GetITStatus(EXTI_Line9))
          {   
             EXTI_ClearITPendingBit(EXTI_Line9);
         //   sendT("принято прерывание! INT9\r\n");   
           state_INT9=1;

          }

}




void EXTI15_10_IRQHandler (void)
{ 
  
  if (EXTI_GetITStatus(EXTI_Line11))
  { 
   //  EXTI_ClearFlag(EXTI_Line11);
     EXTI_ClearITPendingBit(EXTI_Line11);
  //   sendT("принято прерывание! INT11\r\n");
     
   state_INT11=1;

  }

    if (EXTI_GetITStatus(EXTI_Line10))
  { 
   //  EXTI_ClearFlag(EXTI_Line10);
     EXTI_ClearITPendingBit(EXTI_Line10);
   //  sendT("принято прерывание! INT10\r\n");
     
   state_INT10=1;

  }

    if (EXTI_GetITStatus(EXTI_Line12))
  { 
   //  EXTI_ClearFlag(EXTI_Line12);
     EXTI_ClearITPendingBit(EXTI_Line12);
   //  sendT("принято прерывание! INT12\r\n");
     
   state_INT12=1;

  }

    if (EXTI_GetITStatus(EXTI_Line13))
  { 
    // EXTI_ClearFlag(EXTI_Line13);
     EXTI_ClearITPendingBit(EXTI_Line13);
   //  sendT("принято прерывание! INT11\r\n");
     
   state_INT13=1;

  }

    if (EXTI_GetITStatus(EXTI_Line14))
  { 
     //EXTI_ClearFlag(EXTI_Line14);
     EXTI_ClearITPendingBit(EXTI_Line14);
   //  sendT("принято прерывание! INT11\r\n");
     
   state_INT14=1;

  }

    if (EXTI_GetITStatus(EXTI_Line15))
  { 
     //EXTI_ClearFlag(EXTI_Line15);
     EXTI_ClearITPendingBit(EXTI_Line15);
   //  sendT("принято прерывание! INT15\r\n");
     
   state_INT15=1;

  }



}




void init_GPIO (void)
{
  
    GPIO_PinRemapConfig( GPIO_Remap_SWJ_Disable, ENABLE );
    AFIO->MAPR|=AFIO_MAPR_SWJ_CFG_JTAGDISABLE; 

    /* Configure the GPIOA  pin выходы*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


     /* Configure the GPIOB  pin выходы*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

     /* Configure the GPIOB  pin входы*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
	     /* Configure the GPIOC  pin выходы*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	     /* Configure the GPIOC  pin входы*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
		
        /* Configure выходы GPIOD pin */
    GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_2|
									GPIO_Pin_3|
									GPIO_Pin_4|
									GPIO_Pin_5|
									GPIO_Pin_6|
									GPIO_Pin_7|
									GPIO_Pin_8|
									GPIO_Pin_9|
									GPIO_Pin_10|
									GPIO_Pin_11|
									GPIO_Pin_12|
									GPIO_Pin_13|
									GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	   /* Configure входы GPIOD pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	

        /* Configure выходы GPIOE pin */
    GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_0|
									GPIO_Pin_1|
									GPIO_Pin_2|
									GPIO_Pin_3|
									GPIO_Pin_4|
									GPIO_Pin_5|
									GPIO_Pin_6|
									GPIO_Pin_7|
								//	GPIO_Pin_8|
								//	GPIO_Pin_9|
								//	GPIO_Pin_10|
								//	GPIO_Pin_11|
									GPIO_Pin_12|
									GPIO_Pin_13|
									GPIO_Pin_14|
									GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	      /* Configure входы GPIOE pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 |
								  GPIO_Pin_9 |
								  GPIO_Pin_10|
								  GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

  
     
 }

/*

void init_TIM (void)
{
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseStructInit(&Timer1);
    Timer1.TIM_Prescaler = 720;
    Timer1.TIM_Period = 250;

    TIM_TimeBaseInit(TIM4, &Timer1); 
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
    NVIC_EnableIRQ(TIM4_IRQn);


}

*/

/*

void TIM4_IRQHandler()
{   
    uint16_t button = 0;
      
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    
}

*/



void init_ADC (void)
{
	//подаём тактовую частоту
    RCC_ADCCLKConfig(RCC_PCLK2_Div4); 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  //конфигурируем ноги АЦП порт А 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;//
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

  //конфигурируем ноги АЦП порт С 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;//
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

 //конфигурируем ноги АЦП порт B 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;//
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
    // ADC1 configuration 
    ADC_init_struct.ADC_Mode = ADC_Mode_Independent;//Работаем не в Dual режиме.
    ADC_init_struct.ADC_ScanConvMode = DISABLE;//Выключаем сканирование.
    ADC_init_struct.ADC_ContinuousConvMode = DISABLE;//Выключаем повторное преобразование по окончании преобразования.
    ADC_init_struct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//Выключаем тригеры.
    ADC_init_struct.ADC_DataAlign = ADC_DataAlign_Right;//выравнивание полученных данных по правому краю.
    ADC_init_struct.ADC_NbrOfChannel = 1;//Число каналов для сканирования.
    ADC_Init(ADC1, &ADC_init_struct);//Инициализация.

    ADC_TempSensorVrefintCmd(ENABLE);//внутренний температурный сенсор  

    // Включаем прерывания по окончании преобразования (EoC - End of Conversion)

    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

    // Включаем общие прерывания от АЦП

    NVIC_EnableIRQ(ADC1_2_IRQn);

    // Включаем АЦП
    ADC_Cmd(ADC1, ENABLE);

    //Делаем калибровку.

    // Enable ADC1 reset calibration register 
    ADC_ResetCalibration(ADC1);
    // Check the end of ADC1 reset calibration register 
    while(ADC_GetResetCalibrationStatus(ADC1));

    // Start ADC1 calibration 
    ADC_StartCalibration(ADC1);
    // Check the end of ADC1 calibration 
    while(ADC_GetCalibrationStatus(ADC1));

    //Для запуска АЦП используем команду 
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);


}



void ADC1_2_IRQHandler(void)//ADC interrupt
{
    

//Для чтения пишем следующее:
ADC1->SR&=~ADC_SR_EOC;//Сбрасываем бит окончания преобразования
DataConv = ADC1->DR;
flag_ADC=1;

}

unsigned long ADC (unsigned long ch) {
   ADC1->SQR3 = ch;
   flag_ADC=0;
   ADC_SoftwareStartConvCmd(ADC1, ENABLE);
   while (flag_ADC!=1);
   return (DataConv);
}


void init_USART1 (void)
{

//Настраиваем UART

  //Настраиваем порт на выход Push/Pull
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
GPIO_Init(GPIOA, &GPIO_InitStructure);

//Настраиваем порт на вход с открытым коллектором 
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
GPIO_Init(GPIOA, &GPIO_InitStructure);

// Включение тактирования UART делается аналогично:

RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

//APB2PeriphClockCmd - Список устройств, которые включаются этой функцией можно найти в файле stm32f10x_rcc.h.

//Собственно сама инициализация:
//Для инициализации с помощью библиотеки StdPeriph как обычно надо..

// конфигурируем USART1
USART1_InitStructure.USART_BaudRate = 256000;
USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
USART1_InitStructure.USART_StopBits = USART_StopBits_1;
USART1_InitStructure.USART_Parity = USART_Parity_No;
USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

 NVIC_SetPriority (USART1_IRQn, 15);
 NVIC_EnableIRQ(USART1_IRQn);

// разрешаем прерывания по приёму 
 USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

 USART_Cmd(USART1, ENABLE);
        
//USART_OverSampling8Cmd(ENABLE); //можно уменьшить частоту семплирования  
//USART_OneBitMethodCmd(ENABLE); //можно уменьшить количество стробирований
//USART_HalfDuplexCmd (ENABLE); // можно выбрать полудуплексный режим.
        
  USART_Init(USART1, &USART1_InitStructure); //инизиализируем

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //инициализируем DMA

  
 
}


#define Bufer_size  2048u     //16384

 volatile  unsigned  int  text_lengh;
					 char text_buffer[Bufer_size];
 volatile  unsigned  char flag_pachka_TXT=0; //флаг показывающий что идёт передача пачки данных


// внутренняя процедура. Записывает указанное число бит
void DMA_UART_Send(uint8_t* DMA_buf,uint16_t num_bytes) 
{
DMA_InitTypeDef DMA_InitStructure;
				
			// DMA на запись
			DMA_DeInit(DMA1_Channel4);
			DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(USART1->DR);
			DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) DMA_buf;
			DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
			DMA_InitStructure.DMA_BufferSize = num_bytes;
			DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
			DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
			DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
			DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
			DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
			DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
			DMA_Init(DMA1_Channel4, &DMA_InitStructure);
			
			NVIC_SetPriority (DMA1_Channel4_IRQn, 14);
			NVIC_EnableIRQ (DMA1_Channel4_IRQn); //Включаем общие прерывания в NVIC.
			
			DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE); 
			// старт цикла отправки
			USART_ClearFlag(USART1,USART_FLAG_TC | USART_FLAG_TXE);
			USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
			
			// включаем DMA
			DMA_Cmd(DMA1_Channel4, ENABLE);
	
}

void DMA1_Channel4_IRQHandler(void)
{
   if (DMA_GetFlagStatus(DMA1_FLAG_TC4))
    {       
             DMA_Cmd(DMA1_Channel5, DISABLE);
     		 DMA_ClearFlag(DMA1_FLAG_TC4);
      		 DMA_ClearITPendingBit(DMA1_IT_TC4);
     
         // отключаем DMA
			DMA_Cmd(DMA1_Channel4, DISABLE);
			USART_DMACmd(USART1, USART_DMAReq_Tx | USART_DMAReq_Rx, DISABLE);
    }
     flag_pachka_TXT = 0;
	 
	 //sendT ("IRQ_DMA!\r");
}


void init_USART2 (void)
{

//Настраиваем UART

  //Настраиваем порт на выход Push/Pull
GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
GPIO_Init(GPIOA, &GPIO_InitStructure);

//Настраиваем порт на вход 
GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
GPIO_Init(GPIOA, &GPIO_InitStructure);

// Включение тактирования UART делается аналогично:

RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

//APB2PeriphClockCmd - Список устройств, которые включаются этой функцией можно найти в файле stm32f10x_rcc.h.

//Собственно сама инициализация:
//Для инициализации с помощью библиотеки StdPeriph как обычно надо..

// конфигурируем USART1
USART2_InitStructure.USART_BaudRate = 256000;
USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
USART2_InitStructure.USART_StopBits = USART_StopBits_1;
USART2_InitStructure.USART_Parity = USART_Parity_No;
USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;


 NVIC_EnableIRQ(USART2_IRQn);

// разрешаем прерывания по приёму 
 USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

 USART_Cmd(USART2, ENABLE);
        
//USART_OverSampling8Cmd(ENABLE); //можно уменьшить частоту семплирования  
//USART_OneBitMethodCmd(ENABLE); //можно уменьшить количество стробирований
//USART_HalfDuplexCmd (ENABLE); // можно выбрать полудуплексный режим.
        
  USART_Init(USART2, &USART2_InitStructure); //инизиализируем


}


void delay_us( u16 time_us )
{
	int i=0;
	while(i<time_us) {i++;}
}


void UART_DMA_TX (void)

{

 unsigned  int l=0;
 unsigned  int i;
 unsigned  int k;
 unsigned char w=0;

if ((flag_pachka_TXT==0)&&(text_lengh>1u))
{
 
    k = text_lengh;

  	DMA_UART_Send(text_buffer,k);

      text_lengh=0u;  //обнуление счётчика буфера 

      flag_pachka_TXT=1; //устанавливаем флаг передачи

  }

}
	
 void Transf(const char* s)  // процедура отправки строки символов в порт 
   {
       unsigned  short l=0;
       unsigned  short i=0;
         
         if ((flag_pachka_TXT==0) )
      {
        
      	l=strlen(s);

        if ((text_lengh+l)>Bufer_size-5) text_lengh=0u;
      		
        for (i=text_lengh;i<(text_lengh+l);i++) text_buffer[i]=s[i-text_lengh];
               
      	text_lengh=text_lengh+l;
      	

        }	
	
  }
   

 void ZTransf(const char* s,unsigned char a)  // процедура отправки строки символов в порт 
   {
     unsigned  short l;
     unsigned  short i;
            
           if ((flag_pachka_TXT==0))
          {      
            
	          l=a;

	           if ((text_lengh+l)>Bufer_size-5) text_lengh=0;
	            
	           for (i=text_lengh;i<(text_lengh+l);i++) text_buffer[i]=s[i-text_lengh];
	           
	          text_lengh=text_lengh+l;

          } 
      
  }


void init_SPI2(void)

{
    GPIO_InitTypeDef gpio_port;
    SPI_InitTypeDef SPIConf;

       // Включаем тактирование нужных модулей
  
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);


      // Configure PB.13(SPI2_SCK) and PB.15(SPI2_MOSI)
  gpio_port.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
  gpio_port.GPIO_Mode = GPIO_Mode_AF_PP;  //Internal pull down
  gpio_port.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &gpio_port);
  // Configure  PB.14(SPI2_MISO)
  gpio_port.GPIO_Pin = GPIO_Pin_14;
  gpio_port.GPIO_Mode = GPIO_Mode_AF_OD;//GPIO_Mode_AF_PP;
  gpio_port.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &gpio_port);

    // указываем, что используем мы только передачу данных
    SPIConf.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    // указываем, что наше устройство - Master
    SPIConf.SPI_Mode = SPI_Mode_Master;
    // передавать будем по 8 бит (=1 байт)
    SPIConf.SPI_DataSize = SPI_DataSize_8b;
    // режим 00
    SPIConf.SPI_CPOL = SPI_CPOL_Low;//SPI_CPOL_Low;////
    SPIConf.SPI_CPHA = SPI_CPHA_1Edge;
    SPIConf.SPI_NSS = SPI_NSS_Soft;
    // установим скорость передачи (опытным путём выяснили, что разницы от изменения этого параметра нет)
    SPIConf.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;//SPI_BaudRatePrescaler_256;
    // передаём данные старшим битом вперёд (т.е. слева направо)
    SPIConf.SPI_FirstBit = SPI_FirstBit_MSB;
    // внесём настройки в SPI
    SPI_Init(SPI2, &SPIConf);
    // включим  SPI2
    SPI_Cmd(SPI2, ENABLE);
    // SS = 1
    SPI_NSSInternalSoftwareConfig(SPI2, SPI_NSSInternalSoft_Set);

}

void init_SPI1(void)

{
    GPIO_InitTypeDef gpio_port;
    SPI_InitTypeDef SPIConf;

       // Включаем тактирование нужных модулей
  
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

 //  GPIO_PinRemapConfig( GPIO_Remap_SPI1, ENABLE );

      // Configure PB.3(SPI1_SCK) and PB.5(SPI1_MOSI)
  gpio_port.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5;
  gpio_port.GPIO_Mode = GPIO_Mode_AF_PP;  //Internal pull down
  gpio_port.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &gpio_port);
  // Configure  PB.4(SPI1_MISO)
  gpio_port.GPIO_Pin = GPIO_Pin_4;
  gpio_port.GPIO_Mode = GPIO_Mode_AF_OD;//GPIO_Mode_AF_PP;
  gpio_port.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &gpio_port);

    // указываем, что используем мы только передачу данных
    SPIConf.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    // указываем, что наше устройство - Master
    SPIConf.SPI_Mode = SPI_Mode_Slave;
    // передавать будем по 8 бит (=1 байт)
    SPIConf.SPI_DataSize = SPI_DataSize_8b;
    // режим 00
    SPIConf.SPI_CPOL = SPI_CPOL_High;//SPI_CPOL_Low;////
    SPIConf.SPI_CPHA = SPI_CPHA_2Edge;
    SPIConf.SPI_NSS = SPI_NSS_Hard;
    // установим скорость передачи (опытным путём выяснили, что разницы от изменения этого параметра нет)
    SPIConf.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    // передаём данные старшим битом вперёд (т.е. слева направо)
    SPIConf.SPI_FirstBit = SPI_FirstBit_MSB;
    // внесём настройки в SPI
    SPI_Init(SPI1, &SPIConf);
    SPI_I2S_ITConfig(SPI1,SPI_I2S_IT_RXNE,ENABLE); //Включаем прерывание по приему байта
    // включим  SPI1
    SPI_Cmd(SPI1, ENABLE);
    NVIC_EnableIRQ(SPI1_IRQn); //Разрешаем прерывания от SPI1
    // SS = 1
   // SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);

}

//Обработчик прерываний от SPI1
void SPI1_IRQHandler (void) 
{
if (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)==SET) 
	{
		// Прерывание вызвано приемом байта ?
		uint8_t data = SPI1->DR; //Читаем то что пришло
		 //Инвертируем состояние светодиодов
		//SPI1->DR = data; //И отправляем обратно то что приняли
	}
}

u8 SPI2_Send(u8 data) 
{
    SPI_I2S_SendData(SPI2, data);  // отправили данные

    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET); // ждём, пока данные не отправятся
     
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);  // ждём пока данные появтся

    u8 received = SPI_I2S_ReceiveData(SPI2);
    
    return received;
}

u8 SPI2_Resiv(void) 
{
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);  // ждём пока данные появтся
    u8 received = SPI_I2S_ReceiveData(SPI2);
    return received;
}


//****************************************************************************
//function функция обмена по  spi                                            //
//argument байт для передачи                                                //
//result   принятый байт                                                    //
//****************************************************************************
u8 spi2_rw (u8 outb) 
{
  while (!(SPI2->SR & 0x01));
  SPI2->DR = outb;           //передать байт
  while (!(SPI2->SR & 0x02));
  return (SPI2->DR);       //вернуть принятый байт
}

u8 cs_spi (u8 a) //функция програмного чипселекта
{
  int z=0;

  if (a==0) 
    {
     // LED3_on;
      while (z<60) z=z+1;
    }

  if (a==1) 
    { 
      while (z<60) z=z+1;
      //LED3_off;
    } 
}
/*

void itoa(int val, int base,  char *bufstr) //
{
    u8 buf[32] = {0};
    int i = 30;
    int j;
    for(; val && i ; --i, val /= base)
        buf[i] = "0123456789abcdef"[val % base];
    i++; j=0;
    while (buf[i]!=0){ bufstr[j]=buf[i]; i++; j++;}
}
*/


void zputc (unsigned long c) 
{
	while (!USART_GetFlagStatus(USART1, USART_SR_TXE)) {} //Проверка завершения передачи предыдущих данных
	USART1->DR = c; //Передача данных
}

void zputc2 (unsigned long c) 
{
  while (!USART_GetFlagStatus(USART2, USART_SR_TXE)) {} //Проверка завершения передачи предыдущих данных
  USART2->DR = c; //Передача данных
}

void USART1_IRQHandler (void) 
{
   unsigned char temp;
   char status,data;

   if ( USART_GetITStatus(USART1, USART_IT_RXNE) ) 
    {
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    };

  //while(!(USART1->SR & USART_SR_RXNE)); //Ждем поступления данных от компьютера    
   data = USART1->DR; //считываем данные

   rx_buffer1[rx_wr_index1++]=data;

     if (rx_wr_index1 == RX_BUFFER_SIZE1) rx_wr_index1=0;
     if (++rx_counter1 == RX_BUFFER_SIZE1)
        {
        rx_counter1=0;
        rx_buffer_overflow1=1;
        }

// NVIC_ICPR1 = (1 << 5);

   temp = USART1->SR;
   temp = USART1->DR;

}

void USART2_IRQHandler (void) 
{
   unsigned char temp;
   char status,data;

   if ( USART_GetITStatus(USART2, USART_IT_RXNE) ) 
    {
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    };

  //while(!(USART1->SR & USART_SR_RXNE)); //Ждем поступления данных от компьютера    
   data = USART2->DR; //считываем данные

   rx_buffer2[rx_wr_index2++]=data;

     if (rx_wr_index2 == RX_BUFFER_SIZE2) rx_wr_index2=0;
     if (++rx_counter2 == RX_BUFFER_SIZE2)
        {
        rx_counter2=0;
        rx_buffer_overflow2=1;
        }
  // NVIC_ICPR1 = (1 << 5);

   temp = USART2->SR;
   temp = USART2->DR;

}




 void zputs(unsigned char *s, unsigned l)
{
  unsigned i;
  for (i=0;i<l;i++)
	  {
        zputc ((s[i]));
      }
}


 void zputs2(unsigned char *s, unsigned l)
{
 unsigned i;
 for (i=0;i<l;i++)
	 {
      zputc2 ((s[i]));
     }
}



unsigned int leng (unsigned char *s)
{
  unsigned  char i=0;
   while ((s[i]!='\0')&&(i<120)) { i++;}
  return i;

}

void sendT (unsigned char * s)
 {
  zputs(s,leng(s));
 }

void sendT2 (unsigned char * s)
 {
  zputs2(s,leng(s));
 }

char getchar1(void)
{
    char data;
    while (rx_counter1==0);
    data=rx_buffer1[rx_rd_index1++];
    if (rx_rd_index1 == RX_BUFFER_SIZE1) rx_rd_index1=0;
    --rx_counter1;
    return data;
}

char getchar2(void)
{
    char data;
    while (rx_counter2==0);
    data=rx_buffer2[rx_rd_index2++];
    if (rx_rd_index2 == RX_BUFFER_SIZE2) rx_rd_index2=0;
    --rx_counter2;
    return data;
}


void sprnt_tof(char* buf,float a)
 {

 }


  // reverse:  переворачиваем строку s на месте 
 void reverse(char s[])
 {
     int i, j;
     char c;
 
     for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
         c = s[i];
         s[i] = s[j];
         s[j] = c;
     }
 }

 /* itoa:  конвертируем n в символы в s */
 void itoa(int n,char s[])
 {
     int i, sign;
 
     if ((sign = n) < 0)  /* записываем знак */
         n = -n;          /* делаем n положительным числом */
     i = 0;
     do {       /* генерируем цифры в обратном порядке */
         s[i++] = n % 10 + '0';   /* берем следующую цифру */
     } while ((n /= 10) > 0);     /* удаляем */
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
 }

void u_out (char s[],u32 a)
{
   Transf (s);
   itoa (a,strng);
   Transf(strng);
   Transf ("\r\n");
}

void x_out (char s[],u32 a)
{
   Transf (s);
   sprintf (strng,"%X",a);
   Transf(strng);
   Transf ("\r\n");
}

void xn_out (char s[],u32 a)
{
   Transf (s);
   sprintf (strng,"%X",a);
   Transf(strng);
   //put_str1 ("\r\n");
}

void f_out (char s[],float a)
{
   Transf (s);
   sprintf (strng,"%f",a);
   Transf(strng);
   Transf ("\r\n");
}

 

void Massiv_dbm(char y)
{

int i;
float e;

float float1;

 DBm[  9]=-25;
 DBm[ 10]=-24;
 DBm[ 11]=-23;
 DBm[ 12]=-22;
 DBm[ 13]=-21;
 DBm[ 15]=-20;
 DBm[ 24]=-15;
 DBm[ 41]=-10;
 DBm[ 70]=-5;
 DBm[130]= 0;
 DBm[230]= 5;
 DBm[330]= 8;

for (i=  0;i<  9;i++) { e=-600+(i- 9)*6  ;   DBm[i]= e;}
for (i=  9;i< 16;i++) { e=-250+(i- 9)*8.3;   DBm[i]= e;}
for (i= 15;i< 25;i++) { e=-200+(i-15)* 5;    DBm[i]= e;}
for (i= 24;i< 42;i++) { e=-150+(i-24)*2.7;   DBm[i]= e;}
for (i= 41;i< 71;i++) { e=-100+(i-41)*1.7;   DBm[i]= e;}
for (i= 70;i<131;i++) { e=-50 +(i- 70)*0.83; DBm[i]= e;}
for (i=130;i<231;i++) { e=-0 +(i-130)*0.5;   DBm[i]= e;}
for (i=230;i<330;i++) { e= 50 +(i-230)*0.29; DBm[i]= e;}

 if (y)     //
   {
     for (i=0;i<330;i++) 
	 {
       DBm[i]= DBm[i]/10;
/*
       Transf("DBm[");
       sprintf(strng,"%d",i);
       Transf(strng);
       Transf("]=");
       sprintf(strng,"%3.1f",float1);
       Transf(strng);
       Transf("\r\n");
*/
      }
    }
}

unsigned int convert_to_volt(int a)

{
int i=0;
int a_min=10;
int a_max=0;


 for (i=0;i<330;i++) 
 {
    if (a>DBm[i]) a_min=i;
    if (a<DBm[i]) a_max=i;
 }

 if (a_max==0) {a_max=330*(a/8);a_min=a_max;}
    
    return ((a_min+a_max)/2);

}

unsigned short  level_control(char y)

{

u16 z=0 ;
int m,m_dbm;
char ok=0;

float l0 =0;
float l1 =0;
float l2 =0;
float l3 =0;
float l4 =0;
float l5 =0;
float l6 =0;
float l7 =0;
float l8 =0;
float l9 =0;
float l10=0;
float l11=0;
float l12=0;
float l13=0;
float l14=0;
float l15=0;
float dT,dtc,Tmk;

float l4_max=0;

float l16=0;
float l17=0;

int i;

 //l0 =(float)(ADC(0)) ;
 l1 =(float)(ADC(1)) ;
 //l2 =(float)(ADC(2)) ;
 //l3 =(float)(ADC(3)) ;
 l4 =(float)(ADC(4)) ;
 l5 =(float)(ADC(5)) ;
 l6 =(float)(ADC(6)) ;
 l7 =(float)(ADC(7)) ;
 l8 =(float)(ADC(8)) ;
 l9 =(float)(ADC(9)) ;
 //l10=(float)(ADC(10));
 //l11=(float)(ADC(11));
 //l12=(float)(ADC(12));
 //l13=(float)(ADC(13));
 l14=(float)(ADC(14));
 l15=(float)(ADC(15));

 l16=(float)ADC(16);
 l17=(float)ADC(17);

 l16 = l16*3.3/4096;
 l17 = l17*3.3/4096;

 dT =1.43-l16;

 dtc=dT/0.043;

 Tmk=25-dtc;

 Temp_stm32=Tmk;
 U_stm32   =l17*1.913;

l15  =l15*3.3/4096;
l14  =l14*3.3/4096;
//l13  =l13*3.3/4096;
//l12  =l12*3.3/4096;
//l11  =l11*3.3/4096;
//l10  =l10*3.3/4096;
l9   =l9 *3.3/4096;
l8   =l8 *3.3/4096;
l7   =l7 *3.3/4096;
l6   =l6 *3.3/4096;
l4   =l4 *3.3/4096;
l5   =l5 *3.3/4096;
//l3   =l3 *3.3/4096;
//l2   =l2 *3.3/4096;
l1   =l1 *3.3/4096;
//l0   =l0 *3.3/4096;

  Lvl_3v3 	=l7*1.913;
  Lvl_2v5 	=l6;
  Lvl_1v8	=l5;
  Lvl_1v5	=l4;
  Lvl_1v0	=l1;
  Lvl_1v0_MGTVCC	=l14;
  Lvl_1v2_MGTVCC	=l15;
  Lvl_VTTREF		=l8;
  Lvl_VTTDDR		=l9;

 if (y == 1)
 { 
  Transf("\r");
  f_out("Lvl_3v3		:",Lvl_3v3);
  f_out("Lvl_2v5		:",Lvl_2v5);
  f_out("Lvl_1v8		:",Lvl_1v8);
  f_out("Lvl_1v5		:",Lvl_1v5);
  f_out("Lvl_1v0		:",Lvl_1v0);
  
  f_out("Lvl_1v0_MGTVCC	:",Lvl_1v0_MGTVCC);
  f_out("Lvl_1v2_MGTVCC	:",Lvl_1v2_MGTVCC); 
  f_out("Lvl_VTTREF	:",Lvl_VTTREF);  
  f_out("Lvl_VTTDDR	:",Lvl_VTTDDR); 
  Transf("\r");
  
  f_out("Temp_stm32,C 	:",Temp_stm32);
  f_out("Temp_stm32, 	:",l16);
  f_out("U_stm32		:",U_stm32);
  Transf("\r");
}
 return z;

}




void sys_info(char a)

{
int i;
  

  //if (a==1)     //
      {

         // Temp_work_START(); //запуск процедуры измерения температуры во всех датчиках

        

          Transf(" \r\n");
          Transf("---------ДБм--------\r\n");

       
          
          Transf("Lvl_f435_ksvn:");
          sprintf(strng,"%4.2f",1);
          Transf(strng);
          Transf("\r\n");
          Transf("\r\n");
          Transf("------------\r\n");


          Transf("----------\r\n");
/*
          sendT("------------\r\n");
          sendT("Temp_stm32:");
          sprintf(strng,"%d",Temp_stm32);
          sendT(strng);
          sendT("\r\n");
          
          sendT("U_stm32   :");
          sprintf(strng,"%d",U_stm32);
          sendT(strng);
          sendT("\r\n");
     

          sendT("Temp1   :");
          sprintf(strng,"%4.2f",Temp_izmerennoe);
          sendT(strng);
          sendT("\r\n");
   */ 
          Transf("-----------\r\n");
          //......................
         /*
          Transf("flag_64 :");
          sprintf(strng,"%d",level_kl64);
          Transf(strng);
          Transf("\r\n");
          
          Transf("flag_392:");
          sprintf(strng,"%d",level_kl392);
          Transf(strng);
          Transf("\r\n");
          
          Transf("flag_435:");
          sprintf(strng,"%d",level_kl435);
          Transf(strng);
          Transf("\r\n");
          
          Transf("flag_160:"); //флаг ноль при абсолютном переключении двух реле с основного канала на резервный.
          sprintf(strng,"%d",level_kl160);
          Transf(strng);
          Transf("\r\n");
*/
          Transf("----------\r\n");
       }
 }

 

unsigned char sys_control(char y, char s)
{
 
}


char init_sys(char a)
{

}


void Control_Level(void)  //контроль флагов узлов усилителей разных частот
{

}


void Control_SYNC_Level(u8 a)  //контроль флагов сигналов синхронизации
{
  FLAG_FPGA_DONE 	= GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13);
  FLAG_FPGA_INIT 	= GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6);
  FLAG_A_LOCK_DETECT= GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8);
  FLAG_B_LOCK_DETECT= GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9);
  
  if (a==1)
  {
	  u_out("FLAG_FPGA_DONE    :",FLAG_FPGA_DONE);
	  u_out("FLAG_FPGA_INIT    :",FLAG_FPGA_INIT);
	  u_out("FLAG_A_LOCK_DETECT:",FLAG_A_LOCK_DETECT);
	  u_out("FLAG_B_LOCK_DETECT:",FLAG_B_LOCK_DETECT);
  }
}



void IO (void)      //

 {
 unsigned char k=0;
 unsigned char i=0;
 unsigned char index=0;
 unsigned char dlsr=0;
 u32 const_fpga=0;
 u32 crc_comp;
  char z;
  char p[1];
  float float1;
  char data_byte;

  i=lsr;

   while ((sr[k]!=0x7e)&&(k!=lsr))
   {
           k=k+1;

   }

   sym1=sr[k];

   p[0]=sym1;


if ((sym1==0x7e)||(time_uart>10))
    {
        time_uart=0;  //
        packet_flag=1;
        index1=0;
        index=k;
        crc=0;
        crc_ok=0;
        packet_ok=0;
        index_word=0;
        index_data_word=1;
        data_flag=0;
        DATA_Word[0]=' ';

    } //

if (packet_flag==1)

{

    while (i>0)   //

    {
       InOut[index1]=sr[index];

      if (InOut[index1]==' ')  ink1=ink1+1;

      if (InOut[index1]==';')  packet_ok=1;

      if (InOut[index1]==':')  data_flag=1;


      if ((index1>2)&&(InOut[2]==' '))

      {

        if ((InOut[index1]!=' ')&&(InOut[index1]!=';')&&(data_flag!=1)) Word[index_word]=InOut[index1]; //

        if ((data_flag==1)&&(InOut[index1]!=' ')&&(InOut[index1]!=';')&&(InOut[index1]!=':'))  DATA_Word[index_data_word]=InOut[index1]; //

        if ((data_flag!=1)&&(InOut[index1]!=':')) index_word=index_word+1; else if ((data_flag==1)&&(InOut[index1]!=':')) index_data_word=index_data_word+1;

        }

        index1=index1+1;
        index=index+1;
        i=i-1;

    }

    i=index1;
  // index = 0;

if (packet_ok==1)
    {

        if (InOut[0]==0x7e)   crc_ok=crc_ok|0x1;   //
        if (InOut[1]==Adress) crc_ok=crc_ok|0x2;   //

    if (crc_ok==0x3)  //
       {  

        if (strcmp(Word,"help")==0)                     { Transf ("принял help\r\n"      ); Menu1();}

        if (strcmp(Word,"состояние")==0)                { Transf ("принял состояние\r\n" ); level_control(1);Control_SYNC_Level(1);}
     
        if (strcmp(Word,"test_INT_ON")==0)              { Transf ("принял test_INT_ON\r\n");                      flag_INT=1;}

        if (strcmp(Word,"test_INT_OFF")==0)             { Transf ("принял test_INT_OFF\r\n");                     flag_INT=0;}
  
        if (strcmp(Word,"FAPCH_A")==0)      { Transf ("принял FAPCH_A\r");   FAPCH_A (0);}

        if (strcmp(Word,"FAPCH_B")==0)      { Transf ("принял FAPCH_B\r");   FAPCH_B (1);}
		
		if (strcmp(Word,"init")==0)         { Transf ("принял init\r");   init_FAPCH (1);}	

		if (strcmp(Word,"ATT_init")==0)     { 
												Transf ("принял ATT_init\r");
												 crc_comp =atoi(DATA_Word);
												   spi_ATT_s1(crc_comp);
												   spi_ATT_s2(crc_comp);
												   spi_ATT_s3(crc_comp);
												   spi_ATT_s4(crc_comp);}

	    if (strcmp(Word,"fpga")==0)     { 
												Transf ("принял fpga\r");
												 crc_comp =atoi(DATA_Word);
												 const_fpga=FPGA_SPI(crc_comp);
												 x_out("fpga:",const_fpga);  
										}
												   	

		
		
		if (strcmp(Word,"rst")==0)         { Transf ("принял rst\r");   DATA_MK5_1; delay_us(100); DATA_MK5_0; }
   
       }
    }

    if ((packet_ok==1)&&(crc_ok==0x1))     //

    {

    //  RTS_485_on;
    //    zputs2 (InOut,index1);
    //    sendT2("\r\n");

    //    zputs (InOut,index1);
    //    sendT ("\r\n");
    //  RTS_485_off;

    }


    if ( packet_ok==1)

        {

        for (i=0;i<index_word       ;i++)      Word[i]     =0x0;
        for (i=0;i<index_data_word+1;i++) DATA_Word[i]     =0x0;
        //for (i=0;i<index1       ;i++)         InOut[i]     =0x0;

        time_uart=0;  //
        packet_flag=0;
        index1=0;
        crc=0;
        crc_ok=0;
        i=0;
        packet_ok=0;
        index_word=0;
        index_data_word=0;
        data_flag=0;
        };

   }



 }

 void WDG_func(void)
 {
  if (time_wdg<50) WDG_off;
  if (time_wdg>50) {WDG_on;   } 
  if (time_wdg>100) {time_wdg=0;} 
 }

   u8 Temp_work (void)
 {

 }

 void UART_conrol (void)
 {
   int i=0;

       if (rx_counter1!=0)

        {
            lsr=rx_counter1;
            for (i=0;i<lsr; i++) sr[i]=getchar1();
            IO ();
        };

           if (rx_counter2!=0)
        {
             lsr=rx_counter2;
             for (i=0;i<lsr; i++) sr[i]=getchar2();
             IO ();

        };

 }

 void INT_control (u8 a)
 {
 /* 
 	u8 FLAG=0;  

 	 FLAG = GPIO_ReadInputDataBit (GPIOE,GPIO_Pin_7); //проверка наличия сигнала вызова с кассеты 607

 	if (FLAG==1) RUN_FLAG=1; else  RUN_FLAG=0;  //флаг означающий включение кассеты

    if (flag_INT==1)

         {
            flag_INT=0;

            if (state_INT7 ==1)  INT_var=INT_var|0x1 ; else INT_var=INT_var&(~0x01);
            if (state_INT8 ==1)  INT_var=INT_var|0x2 ; else INT_var=INT_var&(~0x02);
            if (state_INT9 ==1)  INT_var=INT_var|0x4 ; else INT_var=INT_var&(~0x04);
            if (state_INT10==1)  INT_var=INT_var|0x8 ; else INT_var=INT_var&(~0x08);
            if (state_INT11==1)  INT_var=INT_var|0x10; else INT_var=INT_var&(~0x10);
            if (state_INT12==1)  INT_var=INT_var|0x20; else INT_var=INT_var&(~0x20);
            if (state_INT13==1)  INT_var=INT_var|0x40; else INT_var=INT_var&(~0x40);
            if (state_INT14==1)  INT_var=INT_var|0x80; else INT_var=INT_var&(~0x80);
        //    if (state_INT15==1)  INT_var=INT_var|0x20; else INT_var=INT_var&(~0x20);


                state_INT7 =0;
                state_INT8 =0;
                state_INT9 =0;
                state_INT10=0;
                state_INT11=0;
                state_INT12=0;
                state_INT13=0;
                state_INT14=0;
                state_INT15=0;

                Transf("\r\n");
                Transf("INT:");
                sprintf(strng,"%x",INT_var);
                Transf(strng);
                Transf("\r\n"); 
         }

         if (a==1) 
         {
         	    Transf("\r\n");
                Transf("FLAG с кассеты управления:");
                sprintf(strng,"%x",FLAG);
                Transf(strng);
                Transf("\r\n"); 
         }
*/
 }

 void LED_control (void)
 {

     if (time_led1 == 150)
        {  LED1_on;
         }

        else if (time_led1 == 300)
            {
              time_led1 = 0;

              LED1_off;
            }

      WDG_func(); //функция контроля внешнего вочдога
 }

void spisend (unsigned int d) //32 бита
{
   SPI2_Send((d >> 24)&0xff);
   SPI2_Send((d >> 16)&0xff);
   SPI2_Send((d >>  8)&0xff);
   SPI2_Send((d)      &0xff);
}
//--------------------------------------------
void spi_FAPCH_B (u32 d) 
{
   //Delay(1);  	
   //B_LE_WIRE_LMK_1;
   //Delay(1);
   //B_LE_WIRE_LMK_0;
   //Delay(1);
   spisend (d);  
   Delay(1);  
   B_LE_WIRE_LMK_1;
   Delay(1);
   B_LE_WIRE_LMK_0;   
}

void INIT_REG_FAPCH_B (void)
{
	R0B.RST = 1;
	R0B.DIV = 8;//8
	R0B.PD  = 0x00;
	R0B.ADR = 0x00;

	R1B.RST = 0;
	R1B.DIV = 8;//8
	R1B.PD  = 0x00;
	R1B.ADR = 0x01;

	R2B.RST = 0;
	R2B.DIV = 6;//6
	R2B.PD  = 0x00;
	R2B.ADR = 0x02;

	R3B.RST = 0;
	R3B.DIV = 6;
	R3B.PD  = 0x00;
	R3B.ADR = 0x03;

	R4B.RST = 0;
	R4B.DIV = 6;
	R4B.PD  = 0x00;
	R4B.ADR = 0x04;
	
	R5B.RST = 0;
	R5B.DIV = 25;
	R5B.PD  = 0x00;
	R5B.ADR = 0x05;

//-----------------------
	R6B.TYPE0 = 2;//2 -lvpecl (700 мВ) - выход 0
	R6B.TYPE1 = 2;//выход 1
	R6B.TYPE2 = 1;//выход 2 , 1 - lvds
	R6B.TYPE3 = 1;//выход 3
	R6B.ADR   = 6;

	R7B.TYPE0 = 1;//выход 4,
	R7B.TYPE1 = 5;//выход 5,
	R7B.TYPE2 = 5;//выход 6,
	R7B.TYPE3 = 5;//выход 7,
	R7B.ADR   = 7;
	
	R8B.TYPE0 = 5;//выход 8,
	R8B.TYPE1 = 0;//выход 9,
	R8B.TYPE2 = 0;//выход 10,
	R8B.TYPE3 = 0;//выход 11,
	R8B.ADR   = 8;
	
	R9B.ADR   = 9;
//----------------------
	R10B.TYPE0 = 0;
	R10B.TYPE1 = 0;
	R10B.EN1   = 0;
	R10B.EN0   = 0;
    R10B.MUX1  = 1;
	R10B.MUX0  = 1;
	R10B.DIV   = 7;
	R10B.ADR   = 10;
//----------------------
	R11B.NO_SYNC1 	  = 1;
	R11B.NO_SYNC3 	  = 1;
	R11B.NO_SYNC5 	  = 1;
	R11B.NO_SYNC7 	  = 1;
	R11B.NO_SYNC9 	  = 1;
	R11B.NO_SYNC11 	  = 1;
	R11B.SYNC_POL_INV = 0;//SYNC is active high
	R11B.SYNC_TYPE 	  = 0;
	R11B.EN_PLL_XTAL  = 0;
	R11B.ADR 		  = 11;
//----------------------
	R12B.SYNC_PLL_DLD = 0;
	R12B.LD_TYPE  	  = 3;
	R12B.LD_MUX	  	  = 2;//PLL R ,LD_MUX sets the output value of the Ftest/LD pin.
	R12B.ADR 	  	  = 12;
//----------------------
	R13B.ADR 	      =13;
	R13B.GPout0       = 4;  //
    R13B.READBACK_TYPE= 3;  //Output (push-pull)
  
  
    R14B.ADR       = 14;
    R14B.GPout1    = 4;
	
    R16B.ADR       = 16; //надо программировать!!!
   
    R24B.ADR       = 24;
    R24B.PLL_R3_LF = 0;
    R24B.PLL_R4_LF = 0;
    R24B.PLL_C3_LF = 0;
	R24B.PLL_C4_LF = 0;
	
	R26B.ADR           = 26;
    R26B.PLL_DLD_CNT   = 3;
    R26B.PLL_CP_GAIN   = 2;
    R26B.EN_PLL_REF_2X = 0;
	
	R28B.ADR      = 28;
    R28B.PLL_R    = 1;
	
	R29B.ADR 	     =29;
	R29B.OSCin_FREQ  = 1;  // >63 MHz to 127 MHz
    R29B.PLL_N_CAL   =12;  // 
	
	R30B.ADR 	     =30;
	R30B.PLL_P       = 2;  // 
    R30B.PLL_N       =12;  // 
	
	R31B.ADR 	       =31;
	R31B.READBACK_ADDR = 0;  // READBACK R0
    R31B.uWire_LOCK    = 0;  // 
}


void FAPCH_B (char r)
{
    INIT_REG_FAPCH_B ();	

	B_stz.R0 =0x00000000;
	B_stz.R0 =	     B_stz.R0|
				(R0B.RST<<17)|
				(R0B.PD <<31)|
				(R0B.DIV<< 5)|
				(R0B.ADR<< 0);
	
	spi_FAPCH_B(B_stz.R0);//сбрасываем микросхемы битом RST
	Transf("R0 - RST\r\n " );	
//------------------------------------------------------------	
	B_stz.R0 =0x00000000;
	B_stz.R0 =	     B_stz.R0|
				(0      <<17)|
				(R0B.PD <<31)|
				(R0B.DIV<< 5)|
				(R0B.ADR<< 0);
				
	B_stz.R1 =0x00000000;
	B_stz.R1 =	     B_stz.R1|
				(R1B.RST<<17)|
				(R1B.PD <<31)|
				(R1B.DIV<< 5)|
				(R1B.ADR<< 0);
				
	B_stz.R2 =0x00000000;
	B_stz.R2 =	     B_stz.R2|
				(0      <<17)|
				(R2B.PD <<31)|
				(R2B.DIV<< 5)|
				(R2B.ADR<< 0);
				
	B_stz.R3 =0x00000000;
	B_stz.R3 =	     B_stz.R3|
				(0      <<17)|
				(R3B.PD <<31)|
				(R3B.DIV<< 5)|
				(R3B.ADR<< 0);
	
	B_stz.R4 =0x00000000;
	B_stz.R4 =	     B_stz.R4|
				(0      <<17)|
				(R4B.PD <<31)|
				(R4B.DIV<< 5)|
				(R4B.ADR<< 0);
				
	B_stz.R5 =0x00000000;
	B_stz.R5 =	     B_stz.R5|
				(0      <<17)|
				(R5B.PD <<31)|
				(R5B.DIV<< 5)|
				(R5B.ADR<< 0);
				
	B_stz.R6 =0x00000000;
	B_stz.R6 =	       B_stz.R6|
				(R6B.TYPE0<<16)|
				(R6B.TYPE1<<20)|
				(R6B.TYPE2<<24)|
				(R6B.TYPE3<<28)|
				(R6B.ADR  << 0);
				
	B_stz.R7 =0x00000000;
	B_stz.R7 =	       B_stz.R7|
				(R7B.TYPE0<<16)|
				(R7B.TYPE1<<20)|
				(R7B.TYPE2<<24)|
				(R7B.TYPE3<<28)|
				(R7B.ADR  << 0);

	B_stz.R8 =0x00000000;
	B_stz.R8 =	       B_stz.R8|
				(R8B.TYPE0<<16)|
				(R8B.TYPE1<<20)|
				(R8B.TYPE2<<24)|
				(R8B.TYPE3<<28)|
				(R8B.ADR  << 0);
				
	B_stz.R9 =0x00000000;
	B_stz.R9 =	                B_stz.R9|
				(0x05		       <<28)|
				(0x05		       <<24)|
				(0x05		       <<20)|
				(0x05		       <<16)|
				(0x05		       <<12)|
				(0x05		       << 8)|
				(0x02		       << 5)|
				(R9B.ADR           << 0);
				
	B_stz.R10 =0x00000000;
	B_stz.R10 =	       B_stz.R10|
				(R10B.TYPE0<<24)|
				(R10B.TYPE1<<30)|
				(R10B.EN1  <<23)|
				(R10B.EN0  <<22)|
				(R10B.MUX1 <<21)|
				(R10B.MUX0 <<20)|
				(R10B.DIV  <<16)|
				(0x01      <<28)|
				(0x01      <<14)|
				(R10B.ADR  << 0);			
				

	B_stz.R11 =0x00000000;
	B_stz.R11 =	       		  B_stz.R11|
				(R11B.NO_SYNC1    <<20)|
				(R11B.NO_SYNC3    <<21)|
				(R11B.NO_SYNC5    <<22)|
				(R11B.NO_SYNC7    <<23)|
				(R11B.NO_SYNC9    <<24)|
				(R11B.NO_SYNC11   <<25)|
				(R11B.SYNC_POL_INV<<16)|
				(R11B.SYNC_TYPE   <<12)|
				(R11B.EN_PLL_XTAL << 5)|
				(0x03		      <<28)|
				(0x01		      <<26)|
				(R11B.ADR  << 0);

	B_stz.R12 =0x00000000;
	B_stz.R12 =	             B_stz.R12|
				(R12B.SYNC_PLL_DLD<<23)|
				(R12B.LD_TYPE     <<24)|
				(R12B.LD_MUX      <<27)|
				(0x03		      <<18)|
				(0x03		      << 5)|
				(R12B.ADR         << 0);

	B_stz.R13 =0x00000000;
	B_stz.R13 =	               B_stz.R13|
				(R13B.GPout0       <<16)|
				(R13B.READBACK_TYPE<<24)|
				(0x07		       <<27)|
				(0x01		       <<15)|
				(R13B.ADR          << 0);

	B_stz.R14 =0x00000000;
	B_stz.R14 =	               B_stz.R14|
				(R14B.GPout1       <<24)|
				(R14B.ADR          << 0);	
				
	B_stz.R16 =0x00000000;
	B_stz.R16 =	               B_stz.R16|
				(0x03		       <<30)|
				(0x01		       <<24)|
				(0x01		       <<22)|
				(0x01		       <<20)|
				(0x01		       <<18)|
				(0x01		       <<16)|
				(0x01		       <<10)|
				(R16B.ADR          << 0);

	B_stz.R24 =0x00000000;
	B_stz.R24 =	           B_stz.R24|
				(R24B.PLL_R3_LF<<16)|
				(R24B.PLL_R4_LF<<20)|
				(R24B.PLL_C3_LF<<24)|
				(R24B.PLL_C4_LF<<28)|
				(R24B.ADR      << 0);

	B_stz.R26 =0x00000000;
	B_stz.R26 =	              B_stz.R26|
				(R26B.PLL_DLD_CNT  << 6)|
				(R26B.PLL_CP_GAIN  <<26)|
				(R26B.EN_PLL_REF_2X<<29)|
				(0x02		       <<30)|
				(0x07		       <<23)|
				(0x01		       <<21)|
				(R26B.ADR          << 0);			

	B_stz.R28 =0x00000000;
	B_stz.R28 =	               B_stz.R28|
				(R28B.PLL_R        <<20)|
				(R28B.ADR          << 0);
				
	B_stz.R29 =0x00000000;
	B_stz.R29 =	            B_stz.R29|
				(R29B.OSCin_FREQ<<24)|
				(R29B.PLL_N_CAL << 5)|
				(0x01		    <<23)|
				(R29B.ADR       << 0);
	
	B_stz.R30 =0x00000000;
	B_stz.R30 =	       B_stz.R30|
				(R30B.PLL_P<<24)|
				(R30B.PLL_N<< 5)|
				(R30B.ADR  << 0);
				
	B_stz.R31 =0x00000000;
	B_stz.R31 =	               B_stz.R31|
				(R31B.READBACK_ADDR<<16)|
				(R31B.uWire_LOCK   << 5)|
				(R31B.ADR          << 0);
				
				
//программируем синтезатор B
	spi_FAPCH_B(B_stz.R0);Transf("R0 " );	
	spi_FAPCH_B(B_stz.R1);Transf("R1 " );	
	spi_FAPCH_B(B_stz.R2);Transf("R2 " );	
	spi_FAPCH_B(B_stz.R3);Transf("R3 " );	
	spi_FAPCH_B(B_stz.R4);Transf("R4 " );	
	spi_FAPCH_B(B_stz.R5);Transf("R5 " );	
	spi_FAPCH_B(B_stz.R6);Transf("R6 " );	
	spi_FAPCH_B(B_stz.R7);Transf("R7 " );	
	spi_FAPCH_B(B_stz.R8);Transf("R8 " );
	spi_FAPCH_B(B_stz.R9);Transf("R9 " );
	spi_FAPCH_B(B_stz.R10);Transf("R10 " );	
	spi_FAPCH_B(B_stz.R11);Transf("R11 " );	
	spi_FAPCH_B(B_stz.R12);Transf("R12 " );
	spi_FAPCH_B(B_stz.R13);Transf("R13 " );	
	spi_FAPCH_B(B_stz.R14);Transf("R14 " );	
	spi_FAPCH_B(B_stz.R16);Transf("R16 " );	
	spi_FAPCH_B(B_stz.R24);Transf("R24 " );	
	spi_FAPCH_B(B_stz.R26);Transf("R26 " );	
    spi_FAPCH_B(B_stz.R28);Transf("R28 " );	
	spi_FAPCH_B(B_stz.R29);Transf("R29 " );	
	spi_FAPCH_B(B_stz.R30);Transf("R30 " );	
	spi_FAPCH_B(B_stz.R31);Transf("R31\r\n" );	
	//----------------------------
	SYNC_LMK_0;
	delay_us(100);
	SYNC_LMK_1;
}
//--------------------------------------------------
  

void spi_FAPCH_A (u32 d) 
{
   Delay(1);  	
   A_LE_WIRE_LMK_1;
   Delay(1);
   A_LE_WIRE_LMK_0;
   Delay(1);
   spisend (d);  
   Delay(1);  
   A_LE_WIRE_LMK_1;
   Delay(1);
   A_LE_WIRE_LMK_0;   
  }
  

void INIT_REG_FAPCH_A (void)
{
	R0.RST = 1;
	R0.MUX = 1;
	R0.EN  = 0;
	R0.DIV = 0xff;
	R0.DLY = 0x00;
	R0.ADR = 0x00;

	R1.RST = 0;
	R1.MUX = 1;
	R1.EN  = 1;
	R1.DIV = 100;
	R1.DLY = 0x00;
	R1.ADR = 1;

	R2.RST = 0;
	R2.MUX = 0;
	R2.EN  = 0;
	R2.DIV = 0xff;
	R2.DLY = 0x00;
	R2.ADR = 2;

	R3.RST = 0;
	R3.MUX = 0;
	R3.EN  = 0;
	R3.DIV = 0xff;
	R3.DLY = 0x00;
	R3.ADR = 3;

	R4.RST = 0;
	R4.MUX = 1;
	R4.EN  = 1;
	R4.DIV = 5;//делитель на 5 - 100 МГц
	R4.DLY = 0x00;
	R4.ADR = 4;

	R5.RST = 0;
	R5.MUX = 0;
	R5.EN  = 0;
	R5.DIV = 0xff;
	R5.DLY = 0x00;
	R5.ADR = 5;

	R6.RST = 0;
	R6.MUX = 0;
	R6.EN  = 0;
	R6.DIV = 0xff;
	R6.DLY = 0x00;
	R6.ADR = 6;

	R7.RST = 0;
	R7.MUX = 0;
	R7.EN  = 0;
	R7.DIV = 0xff;
	R7.DLY = 0x00;
	R7.ADR = 7;
//----------------------
	R11.ADR = 11;
	R11.DIV4= 1;

	R13.ADR =13;
	R13.VCO_C3_C4 = 0;  //C3 - 0pf C4 - 10pf
    R13.VCO_R3    = 0;  //R3 - 600R
    R13.VCO_R4    = 0;  //200 Om
    R13.FREQ      = 100;//100 MHz 

    R14.ADR       = 14;
    R14.PLL_R     = 4;
    R14.MUX       = 2;//Digital Lock Detect (Active High) //N Divider Output/2 (9)
    R14.PWRDN     = 0;
    R14.EN_GLOBAL = 1;
    R14.EN_FOUT   = 0;

    R15.ADR         = 15;
    R15.PLL_CP_GAIN = 0;
    R15.VCO_DIV     = 2;
    R15.PLL_N       = 40;
}

void FAPCH_A (char r)
{
    INIT_REG_FAPCH_A ();	

	A_stz.R0 =0x00000000;
	A_stz.R0 =	A_stz.R0|
				(R0.RST<<31)|
				(R0.MUX<<17)|
				(R0.EN <<16)|
				(R0.DIV<< 8)|
				(R0.DLY<< 4)|
				(R0.ADR<<0);
	
	spi_FAPCH_A(A_stz.R0);//сбрасываем микросхемы битом RST
	Transf("посылаем сброс через регистр R0!\r" );
//------------------------------------------------------------	
	A_stz.R0 =0x00000000;
	A_stz.R0 =	A_stz.R0|
				(0x00  <<31)|
				(R0.MUX<<17)|
				(R0.EN <<16)|
				(R0.DIV<< 8)|
				(R0.DLY<< 4)|
				(R0.ADR<<0);
	Transf("R0 " );			
	A_stz.R1 =0x00000000;
	A_stz.R1 =A_stz.R1| 	
				(R1.MUX<<17)|
				(R1.EN <<16)|
				(R1.DIV<< 8)|
				(R1.DLY<< 4)|
				(R1.ADR<<0);
	Transf("R1 " );			
	A_stz.R2 =0x00000000;
	A_stz.R2 =A_stz.R2| 	
				(R2.MUX<<17)|
				(R2.EN <<16)|
				(R2.DIV<< 8)|
				(R2.DLY<< 4)|
				(R2.ADR<<0);
	Transf("R2 " );				
	A_stz.R3 =0x00000000;
	A_stz.R3 =A_stz.R3|
				(R3.MUX<<17)|
				(R3.EN <<16)|
				(R3.DIV<< 8)|
				(R3.DLY<< 4)|
				(R3.ADR<<0);
	Transf("R3 " );	
	A_stz.R4 =0x00000000;
	A_stz.R4 =A_stz.R4|
				(R4.MUX<<17)|
				(R4.EN <<16)|
				(R4.DIV<< 8)|
				(R4.DLY<< 4)|
				(R4.ADR<<0);
	Transf("R4 " );				
	A_stz.R5 =0x00000000;
	A_stz.R5 =A_stz.R5|
				(R5.MUX<<17)|
				(R5.EN <<16)|
				(R5.DIV<< 8)|
				(R5.DLY<< 4)|
				(R5.ADR<<0);
	Transf("R5 " );				
	A_stz.R6 =0x00000000;
	A_stz.R6 =A_stz.R6|
				(R6.MUX<<17)|
				(R6.EN <<16)|
				(R6.DIV<< 8)|
				(R6.DLY<< 4)|
				(R6.ADR<<0);
	Transf("R6 " );				
	A_stz.R7 =0x00000000;
	A_stz.R7 =A_stz.R7|
				(R7.MUX<<17)|
				(R7.EN <<16)|
				(R7.DIV<< 8)|
				(R7.DLY<< 4)|
				(R7.ADR<<0);
	Transf("R7 " );	
	A_stz.R11 =0x00000000;			
	A_stz.R11 =A_stz.R11|
				(0x82    <<16)|
				(R11.DIV4<<15)|
				(R11.ADR << 0);
	Transf("R11 " );	
	A_stz.R13 =0x00000000;
	A_stz.R13 =A_stz.R13|
				(0x0A         <<22)|
				(R13.FREQ     <<14)|
				(R13.VCO_R4   <<11)|
				(R13.VCO_R3   << 8)|
				(R13.VCO_C3_C4<< 4)|
				(R13.ADR      << 0);
	Transf("R13 " );	
	A_stz.R14 =0x00000000;
	A_stz.R14 =A_stz.R14|
				(R14.EN_FOUT  <<28)|
				(R14.EN_GLOBAL<<27)|
				(R14.PWRDN    <<26)|
				(R14.MUX      <<20)|
				(R14.PLL_R    << 8)|
				(R14.ADR      << 0);
	Transf("R14 " );	
	A_stz.R15 =0x00000000;
	A_stz.R15 =A_stz.R15|
				(R15.PLL_N      << 8)|
				(R15.VCO_DIV    <<26)|
				(R15.PLL_CP_GAIN<<30)|
				(R15.ADR        << 0);			
     Transf("R15\r\n" );	
	//программируем синтезатор А
	
	
	spi_FAPCH_A(A_stz.R0);
	spi_FAPCH_A(A_stz.R1);
	spi_FAPCH_A(A_stz.R2);
	spi_FAPCH_A(A_stz.R3);
	spi_FAPCH_A(A_stz.R4);
	spi_FAPCH_A(A_stz.R5);
	spi_FAPCH_A(A_stz.R6);
	spi_FAPCH_A(A_stz.R7);
	spi_FAPCH_A(A_stz.R11);
	spi_FAPCH_A(A_stz.R13);
	spi_FAPCH_A(A_stz.R14);
	spi_FAPCH_A(A_stz.R15);
	//----------------------------
	SYNC_LMK_0;//останавливает формирование клоков на выходе микрухи!!!!
	delay_us(100);
	SYNC_LMK_1;
	
}

void init_FAPCH (u8 a)

{
	if (a==1)
	{
	
	 Transf("\r\n" );
	 Transf("-----------------------------------\r\n" );
	 Transf("Пришла команда активации кассеты!!!\r\n" );
	 Transf("Программирую ФАПЧ_A:\r" );
     FAPCH_A (0);
	 Transf("...\r" );
	 Transf("Выполненно!\r" );
	 Transf("\r\n" );	
	
	 //----------------------------------------------
	 Transf("\r\n" );
	 Transf("-----------------------------------\r\n" );
	 Transf("Программирую ФАПЧ_B:\r" );
     FAPCH_B (1);
	 Transf("...\r" );
	 Transf("Выполненно!\r" );
	 Transf("\r\n" );
	 
	}
}

void spi_ATT_s4 (u8 d) 
{
   Delay(1);  	
   LE_D_ATT_0;
   Delay(1);
   spisend (d);  
   Delay(1);  
   LE_D_ATT_1;
   Delay(1);  
   Transf("..инициализация АТТ4!\r\n");
   u_out("ATT4:",d);
 }

void spi_ATT_s3 (u8 d) 
{
   Delay(1);  	
   LE_C_ATT_0;
   Delay(1);
   spisend (d);  
   Delay(1);  
   LE_C_ATT_1;
   Delay(1);  
   Transf("..инициализация АТТ3!\r\n");
   u_out("ATT3:",d);
 }

void spi_ATT_s2 (u8 d) 
{
   Delay(1);  	
   LE_B_ATT_0;
   Delay(1);
   spisend (d);  
   Delay(1);  
   LE_B_ATT_1;
   Delay(1); 
   Transf("..инициализация АТТ2!\r\n"); 
   u_out("ATT2:",d);
 }

 void spi_ATT_s1 (u8 d) 
{
   Delay(1);  	
   LE_A_ATT_0;
   Delay(1);
   spisend (d);  
   Delay(1);  
   LE_A_ATT_1;
   Delay(1);  
   Transf("..инициализация АТТ1!\r\n");
   u_out("ATT1:",d);
 }


 u8  Zahvat_control(char y)
{ 
 static unsigned int delay_ERROR=0;
 u8 FLAG=0;
 u8 FLAG_HARD_err=0;
 u8 FLAG_SOFT_err=0;
 u8 FLAG_LANE_UP =0;
 u8 FLAG_CHANNEL_UP =0;

	  FLAG = GPIO_ReadInputDataBit (GPIOC,GPIO_Pin_8); //проверка наличия сигнала Захват ФАПЧ - A
	  
	  FLAG_HARD_err    = GPIO_ReadInputDataBit (GPIOE,GPIO_Pin_8 ); //проверка наличия сигнала HARD_err
	  FLAG_SOFT_err    = GPIO_ReadInputDataBit (GPIOE,GPIO_Pin_9 ); //проверка наличия сигнала SOFT_err
	  FLAG_LANE_UP     = GPIO_ReadInputDataBit (GPIOE,GPIO_Pin_10); //проверка наличия сигнала LANE_UP
	  FLAG_CHANNEL_UP  = GPIO_ReadInputDataBit (GPIOE,GPIO_Pin_11); //проверка наличия сигнала CHANNEL_UP
	  
	  if (FLAG==0) delay_ERROR++; else delay_ERROR = 0; //фильтр сигнала фазового детектора
	
    if (delay_ERROR>500)   Sys_control_health++; 
	  
	  if (y)     //индикация по требованию
	  
	  {  
 
	   Transf("\r\n" );
	   Transf("---------------------------------\r\n" );
	   
	  	//if (FLAG)  Transf("Захват ФАПЧ - есть!\r" );  else Transf("нет Захвата ФАПЧ !!!\r" ); // сигнал контроля 
		u_out("FLAG_HARD_err   :",FLAG_HARD_err);
		u_out("FLAG_SOFT_err   :",FLAG_SOFT_err);
		u_out("FLAG_LANE_UP    :",FLAG_LANE_UP);
		u_out("FLAG_CHANNEL_UP :",FLAG_CHANNEL_UP);
	  } 
	
	  return FLAG;
  
  }   
  
void LED_Zahvat(unsigned int a)
{
  unsigned int delay_LED=200;

  static unsigned char l=0;
  static unsigned char  delay_a=0;
  static unsigned char  delay_b=0;
  static unsigned int delay_ERROR=0;

  if (tick_wait_LED_Zahvat>delay_LED)
  {
   if (a >0u) l=0u;    //работает
   if (a==0u) l=~l;    //не готов

   if (a >0u) 
   {
     // OK_on ; //квитанция для кассеты К607
      delay_ERROR=0;
   }
   
   if (a==0u) OK_off;

 
   if (l==0x00) LED1_on;
   if (l==0xff) LED1_off;

   tick_wait_LED_Zahvat = 0;
  
   delay_a++;
   delay_b++;

   if ((Sys_control_health>0)&&(delay_a>4)) 
   {
      Sys_control_health = 0;
      delay_a=0;
   	  //Transf("\r\n");
   }
  }
}

void Avariya (void)
{
  
	if (Sys_control_health>0)  
  {
  OK_off;
  } 	
  else OK_on; //авария

	Sys_control_health = 0;
}


void Menu1(char a)
 
 {
//***************************************************************************

    int i;
  
 
  for (i=0;i<20;i++) Transf("\r");    // очистка терминала
  for (i=0; i<20; i++) Transf ("-");  // вывод приветствия
  Transf("\r");
  Transf("..........Terminal К608.........\r\n");
  Transf("\r");
  Transf("MENU :\r");
  Transf("-------\r");
  Transf("Расшифровка структуры команды:\r");
  Transf("~ - стартовый байт\r");
  Transf("1 - адрес абонента\r");
  Transf("Установить_F0 -  команда\r");
  Transf("=30000000 - данные команды\r");
  Transf(";- конец пачки \r");
  Transf(".............. \r");
  Transf("Адреса: \r");
  Transf("1 - 5У-К614   , синхронизатор : \r");
  Transf("3 - 5У-К612   , синтезатор частоты излучения  \r");
  Transf("4 - 5У-К612-1 , синтезатор частоты гетеродина \r");
  Transf("6 - 5У-К611   , формирователь опорных частот  \r");
  Transf("---------------------------------------------\r\n");
  Transf("IP  :192.168.1.163 - IP адрес    блока\r");
  Transf("PORT:2054          - номер порта блока\r");
  Transf("~1 help; - текущее меню\r");
//  Transf("~1 состояние; - выводит состояние блока\r");
  Transf("-------------------------------------------\r");
  Transf(" Синтезатор  5-У-К612\r");
  Transf("\r");
  Transf("~3 Установить_F0=430000000; в Гц\r");
  Transf("~3 Установить_частоту_F_low:430000000;   (для ЛЧМ)\r");
  Transf("~3 Установить_частоту_F_high:432000000;  (для ЛЧМ)\r");
  Transf("~3 Установить_Амплитуду:15; в ДБм\r");
  Transf("~3 Установить_девиацию_ЛЧМ:2000000; в Гц (для ЛЧМ)\r");
  Transf("~3 Длительность_импульса_Tимп:10;  в мкс (для ЛЧМ)\r");
  Transf("~3 Частота_повторения_Fимп:100000;  в Гц (для ЛЧМ)\r");
  Transf("~3 Установить_ЛЧМ; - включает ЛЧМ сигнал\r");
  Transf("-------------------------------------------\r");
  Transf(" Гетеродин  5-У-К612-1\r");
  Transf("\r");
  Transf("~4 Установить_F0=430000000; в Гц\r");
  Transf("~4 Установить_частоту_F_low:430000000;   (для ЛЧМ)\r");
  Transf("~4 Установить_частоту_F_high:432000000;  (для ЛЧМ)\r");
  Transf("~4 Установить_Амплитуду:15; в ДБм\r");
  Transf("~4 Установить_девиацию_ЛЧМ:2000000; в Гц (для ЛЧМ)\r");
  Transf("~4 Длительность_импульса_Tимп:10;  в мкс (для ЛЧМ)\r");
  Transf("~4 Частота_повторения_Fимп:100000;  в Гц (для ЛЧМ)\r");
  Transf("~4 Установить_ЛЧМ; - включает ЛЧМ сигнал\r");
  Transf("\r");
  Transf("\r");
  Transf("++++++++++++++++++++\r");
  Transf("\r");
  Transf("\r");
  //for (i=0; i<64; i++) zputs ("*",1);  // вывод приветствия
  //for (i=0;i<10;i++) puts("\r",1);  // очистка терминала
  Transf("\r");
  //*******************************************************************************


  
  }

u32 FPGA_SPI (u8 data)
{
   u32 d[4];

   Delay(1);  	
   FPGA_CS_0;
   Delay(1);
   d[0] = SPI2_Send (data);  
   d[1] = SPI2_Send (data); 
   d[2] = SPI2_Send (data); 
   d[3] = SPI2_Send (data); 
   Delay(1);  
   FPGA_CS_1;
   Delay(1);  
   //u_out("SPI_rsv:",d);
   return (u32)((d[0]<<24)+(d[1]<<16)+(d[2]<<8)+(d[3]<<0));
}


int main(void)
{
    int i;
    u8 sch_pcf1=0;
    u8 sch_pcf2=0;
    u8 FLAG_FAPCH_ON=0;

  u8 buf[2];

    init_RCC   ();
    init_GPIO  ();
    init_USART1();
//  init_USART2();
    init_ADC   ();
//  init_TIM   ();
    init_EXT   ();
    init_SPI2   ();

    LE_A_ATT_1;
	LE_B_ATT_1;
	LE_C_ATT_1;
	LE_D_ATT_1;

 //   _enable_irq ();

   Massiv_dbm(1); //расчёт массива ДБ для детектора
   LED1_off;

	Transf("\r\n");
    Transf("------");
    Transf("\r\n");
    Transf("работаем!\r\n");
    Transf("------");
    Transf("\r\n");

u8 flag_start_init=0; //флаг инициализации системы
   timer_start_init=0;
   timer_INIT_FAPCH1=0;
   FLAG_FAPCH_ON=0;

u8 step=0; //шаг работы контроллера

    Menu1(0);
	
	//-----------ИНИЦИАЛИЗАЦИЯ-------------
	PWRDN_A_1;
	PWRDN_B_1;
	PWRDN_C_1;
	PWRDN_D_1;
	
	CS_AT45_1;
	CS_AT25_1;
	
	LE_A_ATT_1;
	LE_B_ATT_1;
	LE_C_ATT_1;
	LE_D_ATT_1;
	
	A_LE_WIRE_LMK_1;
	B_LE_WIRE_LMK_1;
	C_LE_WIRE_LMK_1;
	
	UPR_A2_0;
	SYNC_LMK_0;
	
	A_GOE_1;
	B_GOE_1;
	C_GOE_1;
	
	RE_485_0;
	RTS_485_0;
	
	DE1RX_RS422_0;
	DE2RX_RS422_0;
	
	SEL_ETALON_1;
	FLAG_FAPCH_ON=0;
	
	DATA_MK5_0;

    while(1)
    {

   
    if ((timer_INIT_FAPCH1>1500)&&(FLAG_FAPCH_ON==0))  
    	{
    		init_FAPCH (1);
    		FLAG_FAPCH_ON=1;
    		spi_ATT_s1(0xff);
    		spi_ATT_s2(0xff);
    		spi_ATT_s3(0xff);
    		spi_ATT_s4(0xff);
    	}
    else
    {
      if (FLAG_FAPCH_ON==0)
      {
       //LED1_off;
      }
    }

 //----------------UART------------------
       UART_conrol ();
       UART_DMA_TX (); //отправка по DMA сообщений 
//---------------------------------------------
      Control_SYNC_Level(0); //контроль флагов сигналов синхронизации, делается как можно чащще , а лучше сделать прерывания.
   
   if (TIMER_SCR<5000)  	level_control(0); //измерение уровней детекторов 
	else 
		{
			LED_Zahvat (Zahvat_control(1));
			Transf("\r\n");
			Transf("------");
			Transf("\r\n");
			level_control(1);
			Control_SYNC_Level(1);
			TIMER_SCR=0;
			Transf("\r\n");
			Transf("------");
			Transf("\r\n");
		} 					  
   
   //  INT_control (0); //проверка отработки внешних прерываний
//----------------------------------------------------------------------- 
            
      LED_control (); // управление индикаторными светодиодами
      WDG_func(); //функция контроля внешнего вочдога
//-----------------------------------------------------------------------
      flag_start_init=0;

    }
}

