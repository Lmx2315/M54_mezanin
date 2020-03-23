#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
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
#include "var_and_structur.h"

#define adress_clock 0x68
#define adress_led   0x25

GPIO_InitTypeDef  GPIO_InitStructure;   //объявляем структуры ножек GPIO

USART_InitTypeDef USART1_InitStructure; //объявляем структуры UART1
USART_InitTypeDef USART2_InitStructure; //объявляем структуры UART2
 EXTI_InitTypeDef EXTI_InitStructure;   //объявляем структуру для внешних прерываний   
  ADC_InitTypeDef ADC_init_struct;      //объявляем структуру ADC

  //  TIM_TimeBaseInitTypeDef Timer1;         //объявляем структуру Timer

static volatile uint32_t __counter;
static volatile uint32_t led_counter;

unsigned char flag_uart1=0;
unsigned char flag_uart2=0;

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

unsigned int led_tick;

//----------------------- IO ()  --------------------------------------   
#define buf_IO   32u 
#define buf_Word 32u 
#define buf_DATA_Word 200u 
#define BUFFER_SR 200u

    char    srt[BUFFER_SR+1];
    char  strng[buf_IO];

unsigned int lenght=0;

       char      InOut[BUFFER_SR+1];
       char      Word [buf_Word+1];    //массив командного слова
       char DATA_Word [buf_DATA_Word+1];    //массив слова - данных
       char DATA_Word2[buf_DATA_Word+1];    //массив слова - данных
 
#define MAX_PL 157u

  char Adress=0x30;      // адрес  
  char Master_flag=0x00; // флаг обозначающий мастер кассету 0, 1- вспомогательный синхронизатор.
  
unsigned     int index1=0u;
unsigned     char crc_ok=0u;
unsigned     char packet_ok=0u;
unsigned     char packet_flag=0u;

unsigned     int index_z=0u; 

unsigned     int index_word=0u;
unsigned     int index_data_word=0u;
unsigned     int index_data_word2=0u;
unsigned     int lenght_data=0u;//длинна данных
unsigned     char data_flag=0u;
unsigned     char data_flag2=0u;
unsigned     char FLAG_lenght=0u;//флаг служебного байта - длинный данных
unsigned     int sch_lenght_data=0u;
unsigned     char FLAG_DATA=0u;
		 	 u16 SCH_LENGHT_PACKET=0u;
 
unsigned char FLAG_CW=0u;
unsigned     int crc_input=0u; 
unsigned     int crc_comp=0u;
 
 u32 SCH_7E_PAKET=0;
 u32 SCH_7E_PAKET_MAX=0;
 
 u32 SCH_7k_PAKET=0;
 u32 SCH_7k_PAKET_MAX=0;
 
 u32 SCH_7ok_PAKET=0;
 u32 SCH_7ok_PAKET_MAX=0;


 float time_uart; //
 unsigned char flag_pcf=0;

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
struct reg_lmk03806 A_stz;

struct R0_5_03806
{
  u8  ADR:5;
  u16 DIV:11;
  u8  PD :1;
  u8  RST:1;
};

struct R0_5_03806 R0B,R1B,R2B,R3B,R4B,R5B;
struct R0_5_03806 R0A,R1A,R2A,R3A,R4A,R5A;

struct R6_8_03806
{
  u8  ADR:5;
  u8  TYPE0:4;
  u8  TYPE1:4;
  u8  TYPE2:4;
  u8  TYPE3:4;
};

struct R6_8_03806 R6B,R7B,R8B;
struct R6_8_03806 R6A,R7A,R8A;

struct R9_03806
{
  u8  ADR:5;

};
struct R9_03806 R9B;
struct R9_03806 R9A;

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
struct R10_03806 R10A;

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
struct R11_03806 R11A;

struct R12_03806
{
  u8  ADR:5;
  u8 SYNC_PLL_DLD :1;
  u8  LD_TYPE     :3;
  u8  LD_MUX      :5;
};

struct R12_03806 R12B;
struct R12_03806 R12A;

struct R13_03806
{
  u8  ADR:5;
  u8  GPout0         :3;
  u8  READBACK_TYPE  :3;
};
struct R13_03806 R13B;
struct R13_03806 R13A;

struct R14_03806
{
  u8  ADR:5;
  u8 GPout1 :3;
};
struct R14_03806 R14B;
struct R14_03806 R14A;

struct R16_03806
{
  u8  ADR:5;

};
struct R16_03806 R16B;
struct R16_03806 R16A;

struct R24_03806
{
  u8  ADR:5;
  u8 PLL_R3_LF :3;
  u8 PLL_R4_LF :3;
  u8 PLL_C3_LF :4;
  u8 PLL_C4_LF :4;
};
struct R24_03806 R24B;
struct R24_03806 R24A;

struct R26_03806
{
  u8  ADR:5;
  u16 PLL_DLD_CNT  :14;
  u8 PLL_CP_GAIN   :2;
  u8 EN_PLL_REF_2X :1;
};
struct R26_03806 R26B;
struct R26_03806 R26A;

struct R28_03806
{
  u8  ADR:5;
  u16 PLL_R   :12;
};
struct R28_03806 R28B;
struct R28_03806 R28A;

struct R29_03806
{
  u8  ADR:5;
  u8  OSCin_FREQ :3;
  u32 PLL_N_CAL  :18;
};
struct R29_03806 R29B;
struct R29_03806 R29A;

struct R30_03806
{
  u8  ADR:5;
  u8  PLL_P :3;
  u32 PLL_N :18;
};
struct R30_03806 R30B;
struct R30_03806 R30A;

struct R31_03806
{
  u8  ADR:5;
  u8 READBACK_ADDR :5;
  u8 uWire_LOCK    :1;
};
struct R31_03806 R31B;
struct R31_03806 R31A;


//------------------------------------------------------------------------------

//-----------прототипы---------------------------

u32 FPGA_rSPI (u8,u8 );
u32 FPGA_wSPI (u8,u8,u32);
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
u16 timer_test=0;
u32 timeOUT_I2C=0;
u32 I2C_delay=0;

void SysTick_Handler(void)  
{  
   if (SysTickDelay != 0)  
   {  
      SysTickDelay--;  
   }  
  	++__counter;
    ++time;
    ++time_wdg;      //таймер вочдога
    ++time_Temp_DMA; //таймер функций термометра
	
	if (timer_test!=0)  --timer_test;
    if (time_led1<500) ++time_led1;
    
    if (flag_Temp_convert==1) ++time_Temp_convert; //считаем время измерения температуры
}  


void init_EXT(void)
{

GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource5); //прерывание с порта B ножки 5
EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising ;  // EXTI_Trigger_Rising
EXTI_InitStructure.EXTI_Line    = EXTI_Line5;
EXTI_InitStructure.EXTI_LineCmd = ENABLE;
EXTI_Init(&EXTI_InitStructure);

NVIC_SetPriority(EXTI9_5_IRQn, 1);
NVIC_EnableIRQ(EXTI9_5_IRQn); 

/*
GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource3); //прерывание с порта D ножки 3
EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising ;  // EXTI_Trigger_Rising
EXTI_InitStructure.EXTI_Line    = EXTI_Line3;
EXTI_InitStructure.EXTI_LineCmd = ENABLE;
EXTI_Init(&EXTI_InitStructure);

NVIC_SetPriority(EXTI3_IRQn, 4);
NVIC_EnableIRQ(EXTI3_IRQn);
 */
}


void EXTI4_IRQHandler (void)
{ 
 if (EXTI_GetITStatus(EXTI_Line4)!=RESET)
 {
	 EXTI_ClearFlag(EXTI_Line4);
     EXTI_ClearITPendingBit(EXTI_Line4);
//	 Transf("INT3\r\n");
 }
}

void EXTI3_IRQHandler (void)
{ 
if (EXTI_GetITStatus(EXTI_Line3)!=RESET)
 {
	 EXTI_ClearFlag(EXTI_Line3);
     EXTI_ClearITPendingBit(EXTI_Line3);
//	 Transf("INT2\r\n");
 }
}


void EXTI9_5_IRQHandler (void)
{ 
  if (EXTI_GetITStatus(EXTI_Line5)!=RESET)
     { 
		EXTI_ClearFlag(EXTI_Line5);
        EXTI_ClearITPendingBit(EXTI_Line5);
  //    sendT("принято прерывание! INT5\r\n");          
      }

  if (EXTI_GetITStatus(EXTI_Line6)!=RESET)
     { 
		EXTI_ClearFlag(EXTI_Line6);
        EXTI_ClearITPendingBit(EXTI_Line6);
  //    sendT("принято прерывание! INT6\r\n");          
      }
 
    if (EXTI_GetITStatus(EXTI_Line7)!=RESET)
     { 
		EXTI_ClearFlag(EXTI_Line7);
        EXTI_ClearITPendingBit(EXTI_Line7);
  //    sendT("принято прерывание! INT7\r\n");          
      }

     if (EXTI_GetITStatus(EXTI_Line8)!=RESET)
     { 
	   EXTI_ClearFlag(EXTI_Line8);
	   EXTI_ClearITPendingBit(EXTI_Line8);
  //    sendT("принято прерывание! INT8\r\n");          
      }

   if (EXTI_GetITStatus(EXTI_Line9)!=RESET)
     { 
        EXTI_ClearFlag(EXTI_Line9);
        EXTI_ClearITPendingBit(EXTI_Line9);
  //    sendT("принято прерывание! INT9\r\n");          
      }

}


void EXTI15_10_IRQHandler (void)
{ 
  
  if (EXTI_GetITStatus(EXTI_Line11))
  { 
   //  EXTI_ClearFlag(EXTI_Line11);
     EXTI_ClearITPendingBit(EXTI_Line11);
  //   sendT("принято прерывание! INT11\r\n");
  }

    if (EXTI_GetITStatus(EXTI_Line10))
  { 
   //  EXTI_ClearFlag(EXTI_Line10);
     EXTI_ClearITPendingBit(EXTI_Line10);
   //  sendT("принято прерывание! INT10\r\n");
  }

    if (EXTI_GetITStatus(EXTI_Line12))
  { 
   //  EXTI_ClearFlag(EXTI_Line12);
     EXTI_ClearITPendingBit(EXTI_Line12);
   //  sendT("принято прерывание! INT12\r\n");
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
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

     /* Configure the GPIOA  pin входы*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

     /* Configure the GPIOB  pin выходы*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

     /* Configure the GPIOB  pin входы*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	     /* Configure the GPIOC  pin выходы*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	     /* Configure the GPIOC  pin входы*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
		
        /* Configure выходы GPIOD pin */
    GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_2|
    								GPIO_Pin_3|
    								GPIO_Pin_7|
									GPIO_Pin_10|
									GPIO_Pin_11|
									GPIO_Pin_12|
									GPIO_Pin_13|
									GPIO_Pin_14|
									GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	   /* Configure входы GPIOD pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|
    							  GPIO_Pin_6;
    							
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	

        /* Configure выходы GPIOE pin */
    GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_1|
    								GPIO_Pin_2|
									GPIO_Pin_3|
									GPIO_Pin_4|
									GPIO_Pin_5|
									GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	      /* Configure входы GPIOE pin */
    GPIO_InitStructure.GPIO_Pin = 
								  GPIO_Pin_7|
								  GPIO_Pin_8|
								  GPIO_Pin_9|
								  GPIO_Pin_10|
								  GPIO_Pin_11|
								  GPIO_Pin_12|
							      GPIO_Pin_13|
								  GPIO_Pin_14|
								  GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
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

  //конфигурируем ноги АЦП порт С 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;//
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
	
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

unsigned long ADC (unsigned long ch) 
{
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

//Настраиваем порт на вход  
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
USART1_InitStructure.USART_BaudRate = 115200;
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


#define Bufer_size  4096u     //16384

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
USART2_InitStructure.USART_BaudRate = 625000;
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
 
 void Buf_send(const char* s,int l)  // процедура отправки буфера в порт 
   {
       int i=0;
	   
         if ((flag_pachka_TXT==0) )
      {
     		
        for (i=0;i<l;i++) text_buffer[i]=s[i];

        }	
		
		text_lengh=l;
	
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


void init_SPI2(void) // spi для LMK

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
    SPIConf.SPI_CPOL = SPI_CPOL_Low;//SPI_CPOL_Low;////SPI_CPOL_High
    SPIConf.SPI_CPHA = SPI_CPHA_1Edge;
    SPIConf.SPI_NSS = SPI_NSS_Soft;
    // установим скорость передачи (опытным путём выяснили, что разницы от изменения этого параметра нет)
    SPIConf.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//SPI_BaudRatePrescaler_256;
    // передаём данные старшим битом вперёд (т.е. слева направо)
    SPIConf.SPI_FirstBit = SPI_FirstBit_MSB;//SPI_FirstBit_MSB SPI_FirstBit_LSB
    // внесём настройки в SPI
    SPI_Init(SPI2, &SPIConf);
    // включим  SPI2
    SPI_Cmd(SPI2, ENABLE);
    // SS = 1
    SPI_NSSInternalSoftwareConfig(SPI2, SPI_NSSInternalSoft_Set);

}

void init_SPI1(void) //spi который идёт на компьютерную плату

{
    GPIO_InitTypeDef gpio_port;
    SPI_InitTypeDef SPIConf;

       // Включаем тактирование нужных модулей
  
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

 //  GPIO_PinRemapConfig( GPIO_Remap_SPI1, ENABLE );

      // Configure PA.5(SPI1_SCK) and PA.7(SPI1_MOSI)
  gpio_port.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
  gpio_port.GPIO_Mode = GPIO_Mode_AF_PP;  //Internal pull down
  gpio_port.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &gpio_port);
  // Configure  PA.6(SPI1_MISO)
  gpio_port.GPIO_Pin = GPIO_Pin_6;
  gpio_port.GPIO_Mode = GPIO_Mode_AF_OD;//GPIO_Mode_AF_PP;
  gpio_port.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &gpio_port);

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
void itoa(u32 val, int base,  char *bufstr) //
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

void un_out (char s[],u32 a)
{
   Transf (s);
   itoa (a,strng);
   Transf(strng);
  // Transf ("\r\n");
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
   //Transf ("\r\n");
}

void f_out (char s[],float a)
{
   Transf (s);
   sprintf (strng,"%f",a);
   Transf(strng);
   Transf ("\r\n");
}

void DATA_send (u8 a)
{
	GPIO_Write(GPIOD,a<<8);
}

void MK_DATA_send (u8 a)
{
	GPIO_Write(GPIOE,a<<8);
}


void Massiv_dbm(char y)
{
u16 i;
float e;
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
for (i=230;i<334;i++) { e= 50 +(i-230)*0.29; DBm[i]= e;}

 if (y)     //
    {
      for (i=0;i<334;i++)    { DBm[i]= DBm[i]/10;}
    }
}


unsigned int convert_to_volt(int a)
{
u16 i=0;
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

 
//Итерационный вариант
float pwr(float num, int pow) {
    int i;
    float tmp = 1;
    for (i = 1; i <= pow; i++) {
        tmp *= num;
    }
    return tmp;
}

float fsqrtf(float n)
{
  float f;
  unsigned long *i;
  f = n;
  i = ((unsigned long*)&f);
  *i = (0xbe6f0000 - *i) >> 1;
  n *= f;
  return (n * (1.5 - 0.5 * f * n));
}

float DBm_to_volt(float a)
{
float x1,x2,y;
y = (a/10);
y =  powf (10,y);
//y =  pwr (10,y);
x1 = (y/1000)*50;
x2 = sqrtf (x1);
//x2 = fsqrtf (x1);
return x2; 
}


float DBm_to_watt(float a)
{
float x1,y;
y = (a/10);
y =  powf (10,y);
x1 = y/1000;
u_out("DBm в Ватты:%4.2f",x1);
return x1; 
}

float watt_to_v(float a)
{
float x1,x2;
x1 = a*50;
x2 = sqrtf (x1);
u_out("Вт в Вольты:%4.2f",x2);
return x2; 
}


void IO ( char* str)      // функция обработки протокола обмена
{
      u16 y;  
 unsigned int i=0;
 unsigned int j=0;
 unsigned int l=0;
 unsigned int k=0;
 char sym1=0;

  i = lenght;//длинна принятой пачки
  if (lenght==0) i = leng(str);
  lenght = 0;
  j=i;
 
  index_z = 0;
  
  if ((time_uart>50u)||(SCH_LENGHT_PACKET>MAX_PL))
  {
	//-------------------
		packet_flag=0; 
	//-------------------
		time_uart=0u;  //обнуление счётчика тайм аута
		index1=0u; 
		crc_ok=0; 
		packet_ok=0; 
		index_word=0u; 
		index_data_word =1u;
		index_data_word2=1u;
		data_flag =0;
		data_flag2=0;
		FLAG_lenght=0u;
		lenght_data=0u;
		sch_lenght_data=0u;
		DATA_Word [0]=' ';
		DATA_Word2[0]=' ';
		FLAG_CW = 0u; //флаг управляющего байта, снимается сразу после исполнения
		FLAG_DATA = 0u;
		SCH_LENGHT_PACKET=0;
  }
  
  while (i>0u)   //перегрузка принятого пакета в массив обработки 
  {
	if ((str[index_z]==0x7e)&&(packet_flag==0))// обнаружено начало пакета
	  {  
		//-------------------
		packet_flag=1; 
		//-------------------
		time_uart=0u;  //обнуление счётчика тайм аута
		index1=0u; 
		crc_ok=0; 
		packet_ok=0; 
		index_word=0u; 
		index_data_word =1u;
		index_data_word2=1u;
		data_flag =0;
		data_flag2=0;
		FLAG_lenght=0u;
		lenght_data=0u;
		sch_lenght_data=0u;
		DATA_Word [0]=' ';
		DATA_Word2[0]=' ';
		FLAG_CW = 0u; //флаг управляющего байта, снимается сразу после исполнения
		FLAG_DATA = 0u;
		SCH_LENGHT_PACKET=0;		
	  }

	 InOut[index1]=str[index_z];
	 SCH_LENGHT_PACKET++;//подсчитываем длинну пакета
		 
	if (( InOut[index1]==';')&&(FLAG_DATA==0u)&&(packet_flag==1))  {packet_flag=0;packet_ok=1u;FLAG_CW=1u;break;}
    
	if (((InOut[index1]=='=')||(InOut[index1]==':'))&&(data_flag==0)) {data_flag=1u;FLAG_CW=1u;}

	if (( InOut[index1]=='.')&&(data_flag2==0)&&(FLAG_DATA==0))   {data_flag2=1u; FLAG_CW=1u;}
	
	if (( InOut[index1]=='$')&&(FLAG_lenght==0u)) {FLAG_lenght=2u;FLAG_CW=1u;}
    
	if ((index1>2u)&&(InOut[2]==' ')&&(FLAG_CW==0u)&&(FLAG_lenght<2u))  
            {
                             if   (data_flag!=1u) {Word[index_word]=InOut[index1];} // заполняем командное слово
                      
                             if  ((data_flag==1u)&&(data_flag2==0u))     DATA_Word[index_data_word]=InOut[index1];// заполняем  слово данных1
                             if  ((data_flag==1u)&&(data_flag2==1u))     DATA_Word2[index_data_word2]=InOut[index1]; // заполняем  слово данных2
                    
                             if  (data_flag!=1u)
                                     {if (index_word<buf_Word) index_word++;} 
                                   else 
                                            {
                                             if ((data_flag==1u)&&(data_flag2==0u)) if (index_data_word<buf_DATA_Word)  {index_data_word++;sch_lenght_data++;}
                                            
                                             if ((data_flag==1u)&&(data_flag2==1u)) if (index_data_word2<buf_DATA_Word) index_data_word2++;
                                            }
			}
	
		if ((FLAG_lenght==2u)&&(FLAG_CW==0u)) {lenght_data = (u8)(InOut[index1]);FLAG_lenght=1u;} //запоминаем длинну пакета данных после ":"
	
		if ((sch_lenght_data<lenght_data)&&(FLAG_lenght==1u)) FLAG_DATA = 1u; else {FLAG_DATA = 0u;}
	 
		if (index1 <BUFFER_SR)  index1++;
		if (index_z<BUFFER_SR)  index_z ++;
		i--;
		FLAG_CW=0u;
		// Transf(".");
  }
 

if (packet_ok==1u) 
  {

   SCH_7ok_PAKET++;
   //Transf("ок\r\n");
     
      if (InOut[0]==0x7e)   crc_ok=crc_ok|0x1;   // проверка первого условия пакета - начало пакета
      if (InOut[1]==Adress) crc_ok=crc_ok|0x2;   // проверка второго условия пакета - адресат назначения
 
 
if (crc_ok==0x3)  //обработка команд адресатом которых является хозяин 
{
if (strcmp(Word,"freq")==0)                     
   {
	 crc_comp =atoi(DATA_Word);
     u_out ("принял freq:", crc_comp   );
	 GEN_360(crc_comp);
   } else
if (strcmp(Word,"sync")==0)                     
   {
	 crc_comp =atoi(DATA_Word);
     u_out ("принял sync:", crc_comp   );
     if (crc_comp!=0) SYNC_LMK_1;
	 else  			  SYNC_LMK_0;
   } else
if (strcmp(Word,"sel")==0)                     
   {
	 crc_comp =atoi(DATA_Word);
     u_out ("принял sel:", crc_comp   );
     if (crc_comp==0) SEL_ETALON_0; else
	 if (crc_comp==1) SEL_ETALON_1;
   } else
if (strcmp(Word,"init")==0)                     
   {
	 crc_comp =atoi(DATA_Word);
	 u_out ("принял init:", crc_comp   ); 
     init_FAPCH (crc_comp);
   } else
if (strcmp(Word,"btn")==0)                     
   {
     Transf ("принял btn\r"    );
     Transf("\r");  
     x_out("BTN:",BTN_var);
   } else	
if (strcmp(Word,"help")==0)                     
   {
     Transf ("принял help\r"    );
     Transf("\r");  
     Menu1(0);
   } else
if (strcmp(Word,"help2")==0)                     
   {
     Transf ("принял help\r"    );
     Transf("\r");  
   } else
if (strcmp(Word,"info")==0)                     
   {
     Transf ("принял info\r"    );
     Transf("\r");  
     info();
   } else
if (strcmp(Word,"menu")==0)                     
   {
     Transf ("принял menu\r"    );
     Transf("\r");  
     Menu1();
   } else							 
					 
if (strcmp(Word,"fpga_r")==0) //
   {
	crc_comp =atoi(DATA_Word);
	crc_input=atoi(DATA_Word2);
	un_out ("принял fpga_r:",crc_comp);
	u_out (".",crc_input);
	x_out ("data:",FPGA_rSPI (crc_comp,crc_input));
   } else
if (strcmp(Word,"fpga_w")==0) //
   {
	crc_comp =atoi(DATA_Word);
	crc_input=atoi(DATA_Word2);
			
	u_out ("принял fpga_w:",crc_comp);
	u_out ("",crc_input);
	FPGA_wSPI (crc_comp,crc_input,0xaabbccdd);
                       
    } else 
             
if (strcmp(Word,"test_spi_read")==0) //
   {
	crc_comp =atoi(DATA_Word);
	crc_input=atoi(DATA_Word2);
				
	u_out ("принял test_spi_read:",crc_comp);
	test_spi_read(crc_comp);
                       
   } else
				
if (strcmp(Word,"test_spi_write")==0) //
   {
	crc_comp =atoi(DATA_Word);
	crc_input=atoi(DATA_Word2);
				
	u_out ("принял test_spi_write:",crc_comp);
	test_spi_write(crc_comp);
                       
   } else
//----------------------------------------------------------------------------------------------				
		
if (strcmp(Word,"test_out")==0) //команда управляет тестовыми выходами , выводит туда требуемые пары сигналов
     {
		crc_comp =atoi(DATA_Word);
		u_out ("принял test_out:",crc_comp);
	
	    FPGA_wSPI(8,4,crc_comp); 
      } else
		  
if (strcmp(Word,"adc")==0) //команда управляет тестовыми выходами , выводит туда требуемые пары сигналов
     {
		crc_comp =atoi(DATA_Word);
		u_out ("принял adc:",crc_comp);
	    u_out("adc:",ADC (crc_comp)); 
      } 
 
 } 

 for (i=0u;i<buf_Word;i++)             Word[i]      =0x0;
 for (i=0u;i<(buf_DATA_Word+1);  i++)  DATA_Word [i]=0x0;
 for (i=0u;i<(buf_DATA_Word+1);  i++)  DATA_Word2[i]=0x0;  
 for (i=0u;i<BUFFER_SR;i++)  
      {
        InOut[i]     =0x0;
      }  
      
	  time_uart=0;  //обнуление счётчика тайм аута
      packet_flag=0; 
      index1=0u; 
      crc_ok=0; 
      i=0;
      packet_ok=0; 
      index_word=0u; 
      index_data_word=0u;
      data_flag=0;
      index_data_word2=0u;
      data_flag2=0;
      index_z =0u;
      FLAG_lenght=0u;
      lenght_data=0u;
      sch_lenght_data=0u;
      FLAG_CW = 0u; //флаг управляющего байта, снимается сразу после исполнения
      FLAG_DATA = 0u;	  
      	  
      DATA_Word [0]=' ';
      DATA_Word2[0]=' ';
	  SCH_LENGHT_PACKET=0;
  }

  if ((packet_ok==1)&&(crc_ok==0x1))     //обработка команд адресатом которых является слейв

  {
    
    if (Master_flag==0)

      {            
      
         // TX_485;
        
          Transf("\r\n");
          ZTransf (InOut,index1);
		  Transf("\r\n");

           rx_wr_index1=0u;
           rx_rd_index1=0u;
           rx_counter1 =0u;
      }
  }
         
 } 


 void WDG_func(void)
 {
  if (time_wdg<50) WDG_off;
  if (time_wdg>50) {WDG_on;   } 
  if (time_wdg>100) {time_wdg=0;} 
 }

 void UART_conrol (void)
{
  u16 i=0u;
  u16 j=0u;
  u8 FLAG_IO=0u;

 if (rx_counter1!=0u)
  {   
    if (rx_counter1<BUFFER_SR) j = rx_counter1; else j=BUFFER_SR;
	
	//Transf(".");

    for (i=0u;i<j; i++) 
     {
      srt[i]=getchar1();
      lenght=i+1;  
      if (srt[i]==';') {FLAG_IO=1;i=j;break;}
     }
	srt[lenght]=0x00;
	IO (srt);	
  };

}


 void LED_control (void)
 {
 	static u8 l;
	static flag_l;

  if (time_led1 > 250)
  {
	
	
	//LED1_0;
	//LED1_C2_0;
	//LED2_C2_0;
	//LED3_0;
	//LED4_0;
	flag_l=0;

  }
		 
  if (time_led1 < 250)
  {
	//LED1_1;
	//LED1_C2_1;
	//LED2_C2_1;
	//LED3_1;
	//LED4_1;
	
	if (flag_l==0)  {l=l<<1;flag_l=1;}	
  }	

  switch (l)
  {
	  case 0x01:
	  	LED1_1;
		LED1_C2_0;
		LED2_C2_0;
		LED3_0;
		LED4_0; break; 
	   case 0x02:
	  	LED1_0;
		LED1_C2_1;
		LED2_C2_0;
		LED3_0;
		LED4_0; break;
	   case 0x04:
	  	LED1_0;
		LED1_C2_0;
		LED2_C2_1;
		LED3_0;
		LED4_0; break;
	   case 0x08:
	  	LED1_0;
		LED1_C2_0;
		LED2_C2_0;
		LED3_1;
		LED4_0; break;
	   case 0x10:
	  	LED1_0;
		LED1_C2_0;
		LED2_C2_0;
		LED3_0;
		LED4_1; break;
	   case 0x20:
	  	LED1_1;
		LED1_C2_0;
		LED2_C2_0;
		LED3_1;
		LED4_0; break;
	   case 0x40:
	  	LED1_0;
		LED1_C2_1;
		LED2_C2_0;
		LED3_0;
		LED4_1; break;
	   case 0x80:
	  	LED1_0;
		LED1_C2_0;
		LED2_C2_0;
		LED3_0;
		LED4_0; break;
  }
  
  if (time_led1==500)  time_led1=0;
  
   if (l==0) l=1;

      WDG_func(); //функция контроля внешнего вочдога
 }

void spisend (unsigned int d) //32 бита
{
   SPI2_Send((d >> 24)&0xff);
   SPI2_Send((d >> 16)&0xff);
   SPI2_Send((d >>  8)&0xff);
   SPI2_Send((d)      &0xff);
}



void Menu1(char a)
 
 {
//***************************************************************************

    int i;
  
 
  for (i=0;i<20;i++) Transf("\r");    // очистка терминала
  for (i=0; i<20; i++) Transf ("-");  // вывод приветствия
  Transf("\r");
  Transf("..........Terminal Тестовой платы.........\r\n");
  Transf("\r");
  Transf("MENU :\r");
  Transf("-------\r");
  Transf("Расшифровка структуры команды:\r");
  Transf("~ - стартовый байт\r");
  Transf("1 - адрес абонента\r");
  Transf(";- конец пачки \r");
  Transf(".............. \r");
  Transf("---------------------------------------------\r\n");
  Transf("IP  :192.168.1.163 - IP адрес    блока\r");
  Transf("PORT:2054          - номер порта блока\r");
  Transf("~0 help; - текущее меню\r");
  Transf("~0 info; - информация о режиме работы имитатора\r");
  Transf("~0 start; запуск формирования сетки импульсов\r");
  Transf("~0 Pni1:100; интервал перед излучением для темпа 1 (мкс)\r");
  Transf("~0 Pii1:100; интервал излучения для темпа 1 (мкс)\r");
  Transf("~0 Pnp1:100; интервал перед приёмом для темпа 1 (мкс)\r");
  Transf("~0 Pip1:100; интервал приёма для темпа 1 (мкс)\r");
  Transf("~0 QIT1:1;   число импульсов в темпе 1\r");
  Transf("~0 Pni2:100; интервал перед излучением для темпа 2 (мкс)\r");
  Transf("~0 Pii2:100; интервал излучения для темпа 2 (мкс)\r");
  Transf("~0 Pnp2:100; интервал перед приёмом для темпа 2 (мкс)\r");
  Transf("~0 Pip2:100; интервал приёма для темпа 2 (мкс)\r");
  Transf("~0 QIT2:1;   число импульсов в темпе 2\r");
  Transf("~0 Pni3:100; интервал перед излучением для темпа 3 (мкс)\r");
  Transf("~0 Pii3:100; интервал излучения для темпа 3 (мкс)\r");
  Transf("~0 Pnp3:100; интервал перед приёмом для темпа 3 (мкс)\r");
  Transf("~0 Pip3:100; интервал приёма для темпа 3 (мкс)\r");
  Transf("~0 QIT3:1;   число импульсов в темпе 3\r");
  Transf("~0 N_temp:3; число темпов < или = 3\r");
  Transf("~0 DDS_delay:56; длительность интервала для подготовки DDS (мкс)\r");
  Transf("~0 T_kalibrovka:504; длительность интервала калибровки (мкс)\r");
  Transf("~0 T_pomeha:302; длительность интервала приёма помехи (мкс)\r");
  Transf("~0 TNO:4000000; длительность интервала ТНО (мкс)\r");
  Transf("~0 Avariya:1; импульсный сигнала <авария> подаётся на блок Б610 \r");
  Transf("~0 GBR:1;  сигнала <ГБР> или <БР> подаётся на блок Б610 \r");
 
  Transf("~0 TNI:1; постоянная еденица на сигнал ТНИ\r");
  Transf("~0 TKI:1; постоянная еденица на сигнал ТКИ\r");
  Transf("~0 TNP:1; постоянная еденица на сигнал ТНП\r");
  Transf("~0 TKP:1; постоянная еденица на сигнал ТКП\r");
  Transf("~0 b610_event:255; имитация состояний блока 610 для блока 660\r");
  
  Transf("~0 615_test:65535; 65283(ок) 0xFFFF для проверки к615, имитация сигналов ФТ и т.д.\r");
  Transf("~0 614_test;для проверки к614,контроль выходных сигналов\r");
  Transf("~0 info_615; - информация о сигналах контроля для К615\r"); 
  Transf("~0 info_614; - информация о сигналах контроля для К614\r");
  Transf("~0 test_out:0; -тестовый вывод сигналов на разъёмы f5Mhz и T5min\r");
  Transf("1:ТНЦ|IntI 2:ТНЦ|IntP 3:IntI|IntP  4:TNO|ТНЦ   5:ТНЦ|TNP 6:ТНЦ|TKP \r");
  Transf("7:ТНЦ|TNI  8:ТНЦ|TKI  9:ТНЦ|Error 10:ТНЦ|TOBM 11:ТНЦ|(IntP|IntI)   \r");
  Transf("-------------------------------------------\r");
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

u32 FPGA_wSPI (u8 size,u8 adr,u32 data)
{
   u8 d[4];
   u8 i,k;
   
   k=size/8;
    
   if (k==1)  d[3]=data;
   if (k==2) {d[2]=data;
			  d[3]=data>>8;
			  }
   if (k==3) {d[1]=data;
			  d[2]=data>>8;
			  d[3]=data>>16;
			  }
   if (k==4) {d[0]=data;
			  d[1]=data>>8;
			  d[2]=data>>16;
			  d[3]=data>>24;
			  }
 	
// FPGA_CS_0;
   SPI2_Send (adr|0x80); //передаём адресс
   for (i=0;i<(size/8);i++)  SPI2_Send (d[3-i]);  //считываем данные
// FPGA_CS_1;

   return 0;
}  
  
u32 FPGA_rSPI (u8 size,u8 adr)
{
   u8 d[4];
   u8 i,k;
   u8 adr_a=0;
   u32 value;
   
   k=size/8;
   adr_a=adr;
 	
//   FPGA_CS_0;
   SPI2_Send (adr_a); //
   for (i=0;i<(size/8);i++) d[i] = SPI2_Send (0);  //считываем данные
//   FPGA_CS_1;
   
   if (k==1) value =   d[0];
   if (k==2) value =  (d[0]<< 8)+ d[1];
   if (k==3) value =  (d[0]<<16)+(d[1]<< 8)+ d[2];
   if (k==4) value =  (d[0]<<24)+(d[1]<<16)+(d[2]<<8)+d[3]; 

   return value;
}

void test_spi_read(u32 k)
{
	u32 i;
	u32 var;
	u32 error=0;
	
	UART_DMA_TX (); //отправка по DMA сообщений 
	
	for (i=0;i<k;i++)	
	{ var=FPGA_rSPI(32,30);
	  if (var!=0xdeedbeef) error++;
	}
	
	u_out("проведено:",k);
	Transf(" попыток\r\n");
	u_out("Число ошибок:",error);
}

void test_spi_write(u32 k)
{
	u32 i;
	u32 var;
	u32 error=0;
	
	UART_DMA_TX (); //отправка по DMA сообщений 
	
	for (i=0;i<k;i++)	
	{
		FPGA_wSPI(32,36,0xdeedbeef);//запись
		var=FPGA_rSPI(32,35);//чтение
	  if (var!=0xdeedbeef) error++;//проверка
	}
	u_out("проведено:",k);
	Transf(" попыток\r\n");
	u_out("Число ошибок:",error);
}


void info ()
{

}



u8 BTN_control ()
{
 u8 v[7];
	
	v[0]=0x1&GPIO_ReadInputDataBit (GPIOC,GPIO_Pin_7 ); //проверка положения переключателя SA8
	v[1]=0x1&GPIO_ReadInputDataBit (GPIOC,GPIO_Pin_8 ); //проверка положения переключателя SA7
	v[2]=0x1&GPIO_ReadInputDataBit (GPIOC,GPIO_Pin_9 ); //проверка положения переключателя SA6
	v[3]=0x1&GPIO_ReadInputDataBit (GPIOA,GPIO_Pin_8 ); //проверка положения переключателя SA5
	v[4]=0x1&GPIO_ReadInputDataBit (GPIOA,GPIO_Pin_11); //проверка положения переключателя SA4
	v[5]=0x1&GPIO_ReadInputDataBit (GPIOA,GPIO_Pin_12); //проверка положения переключателя SA3
	v[6]=0x1&GPIO_ReadInputDataBit (GPIOB,GPIO_Pin_5 ); //проверка положения переключателя SA2
	v[7]=0x1&GPIO_ReadInputDataBit (GPIOC,GPIO_Pin_6 ); //проверка положения переключателя SA1
	
	return ((v[7]<<7)+(v[6]<<6)+(v[5]<<5)+(v[4]<<4)+(v[3]<<3)+(v[2]<<2)+(v[1]<<1)+v[0]); 
}

#define buf_n (2048+3)  //(стартовый байт и финальный 0)

void test_packet (char l)
{
	char     a[buf_n];
	short  cos[buf_n];
	int i=0;
	float f=0.1f;//2.4 Mhz for 240 Fs
	float x,y;
    char odd=0;	
	short z;
	short k=0;
	char hb,lb;
	float noise;
	
	float var_pi=3.14159265359f;
//-----------------------------	
	for  (i=0;i<buf_n;i++) a[i]=0x00;//очищаем фильтр
//--------преамбула------------	
	a[0]=0x23; //код #
	a[1]=0x0A; //код 0x0A '\n'
	a[2]=0x3A; //код 0x0A ':'
	
	k=3;

	if (l=='s') //синус
	{
		for (i=0;i<(buf_n-3)/2;i++) 
		{
			noise=rand() % 1000;
			x=2*var_pi*f*i;
			y=sin(x)*32000+noise;
			z=(short)y;
			//z=i;
			hb=z>>8;
			lb=z&0xff;
			
			a[k++]=hb;
			a[k++]=lb;
			
			//a[k++]=k;
			//a[k++]=k;
		}
	}		
	if (l=='r') //числа
	{
		for (i=0;i<(buf_n-3)/2;i++) 
		{
			
			hb=i>>8;
			lb=i&0xff;
			
			a[k++]=hb;
			a[k++]=lb;

		}

	}
	
	a[buf_n-1]=0x00;
	Buf_send(a,buf_n);
	
//	Transf("\r\n");
//	u_out("i=",i);	
}
//----------------LMK-------------

void spi_FAPCH_B (u32 d) 
{
   spisend (d);  
   Delay(1);  
   B_LE_WIRE_LMK_1;
   Delay(1);
   B_LE_WIRE_LMK_0;   
}

void spi_FAPCH_A (u32 d) 
{
   Delay(1);
   A_LE_WIRE_LMK_0;
   spisend (d);  
   Delay(1);  
   A_LE_WIRE_LMK_1;
   Delay(1);
   A_LE_WIRE_LMK_0;   
}

void INIT_REG_FAPCH_B_100MHz (void)
{
	R0B.RST = 1;
	R0B.DIV = 25;// 100 MHz  25
	R0B.PD  = 0x00;
	R0B.ADR = 0x00;

	R1B.RST = 0;
	R1B.DIV = 25;// 100 MHz
	R1B.PD  = 0;
	R1B.ADR = 0x01;

	R2B.RST = 0;
	R2B.DIV = 100;//6
	R2B.PD  = 0x00;
	R2B.ADR = 0x02;

	R3B.RST = 0;
	R3B.DIV = 250;//10 MHz 250
	R3B.PD  = 0x00;
	R3B.ADR = 0x03;

	R4B.RST = 0;
	R4B.DIV = 20;// 125 MHz
	R4B.PD  = 0x00;
	R4B.ADR = 0x04;
	
	R5B.RST = 0;
	R5B.DIV = 250;// 10 MHz 250
	R5B.PD  = 0x00;
	R5B.ADR = 0x05;

//-----------------------
	R6B.TYPE0 = 5;//выход 0   2 - lvpecl (700 мВ) - выход 0 | 5 - 2000 mV lvPecl
	R6B.TYPE1 = 0;//выход 1
	R6B.TYPE2 = 5;//выход 2 , 1 - lvds  это тактовая для DD9 - в поделке
	R6B.TYPE3 = 0;//выход 3
	R6B.ADR   = 6;
	
	

	R7B.TYPE0 = 0;//выход 4,
	R7B.TYPE1 = 0;//выход 5,
	R7B.TYPE2 = 2;//выход 6,  //6 - LVCMOS (Norm/Inv)
	R7B.TYPE3 = 0;//выход 7,
	R7B.ADR   = 7;
	
	R8B.TYPE0 = 0;//выход 8,  //6 - LVCMOS (Norm/Inv) ,1 - lvds
	R8B.TYPE1 = 0;//выход 9,
	R8B.TYPE2 = 6;//выход 10, //1 - lvds
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
	R11B.NO_SYNC1 	  = 0;
	R11B.NO_SYNC3 	  = 0;
	R11B.NO_SYNC5 	  = 0;
	R11B.NO_SYNC7 	  = 0;
	R11B.NO_SYNC9 	  = 0;
	R11B.NO_SYNC11 	  = 0;
	R11B.SYNC_POL_INV = 0;//SYNC is active high
	R11B.SYNC_TYPE 	  = 1;
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
    R26B.PLL_CP_GAIN   = 2;//3200 uA
    R26B.EN_PLL_REF_2X = 0;
	
	R28B.ADR      = 28;
    R28B.PLL_R    = 100;
	
	R29B.ADR 	     =29;
	R29B.OSCin_FREQ  =1;  //0 < 63 MHz   ; 1>63 MHz to 127 MHz
    R29B.PLL_N_CAL   =300;  // 5 - 100 MHz ref,25 - 20 MHz ref
	
	R30B.ADR 	     =30;
	R30B.PLL_P       =8;  // 
    R30B.PLL_N       =300;  // 5 - 100 MHz ref,25 - 20 MHz ref
	
	R31B.ADR 	       =31;
	R31B.READBACK_ADDR = 0;  // READBACK R0
    R31B.uWire_LOCK    = 0;  // 
}

void INIT_REG_FAPCH_A_100MHz (u32 freq)
{
	R0B.RST = 1;
	R0B.DIV = 1;//не нужен
	R0B.PD  = 0x01;
	R0B.ADR = 0x00;

	R1B.RST = 0;
	R1B.DIV = 1;//не нужен
	R1B.PD  = 0x00;//общий повердаун(?)
	R1B.ADR = 0x01;

	R2B.RST = 0;
	R2B.DIV = 1;//не нужен
	R2B.PD  = 0x01;
	R2B.ADR = 0x02;

	R3B.RST = 0;
	R3B.DIV = 1;//не нужен
	R3B.PD  = 0x01;
	R3B.ADR = 0x03;

	R4B.RST = 0;
	R4B.DIV = 1;//не нужен
	R4B.PD  = 0x01;
	R4B.ADR = 0x04;
	
	R5B.RST = 0;
	R5B.DIV = 7;//360 MHz 7
	R5B.PD  = 0x00;
	R5B.ADR = 0x05;

//-----------------------
	R6B.TYPE0 = 0;//2 -lvpecl (700 мВ) - выход 0
	R6B.TYPE1 = 0;//выход 1
	R6B.TYPE2 = 0;//выход 2 , 1 - lvds
	R6B.TYPE3 = 0;//выход 3
	R6B.ADR   = 6;

	R7B.TYPE0 = 0;//выход 4,
	R7B.TYPE1 = 0;//выход 5,
	R7B.TYPE2 = 0;//выход 6,
	R7B.TYPE3 = 0;//выход 7,
	R7B.ADR   = 7;
	
	R8B.TYPE0 = 0;//выход 8,
	R8B.TYPE1 = 0;//выход 9,
	R8B.TYPE2 = 2;//выход 10,2 -lvpecl (700 мВ)  LVPECL (2000 mVpp)
	R8B.TYPE3 = 2;//выход 11,2 -lvpecl (700 мВ)  LVPECL (2000 mVpp)
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
	R11B.NO_SYNC1 	  = 0;
	R11B.NO_SYNC3 	  = 0;
	R11B.NO_SYNC5 	  = 0;
	R11B.NO_SYNC7 	  = 0;
	R11B.NO_SYNC9 	  = 0;
	R11B.NO_SYNC11 	  = 0;
	R11B.SYNC_POL_INV = 0;//SYNC is active high
	R11B.SYNC_TYPE 	  = 1;//SYNC IO pin type
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
    R24B.PLL_R3_LF = 1;//200R
    R24B.PLL_R4_LF = 1;//200R
    R24B.PLL_C3_LF = 7;//0.01nF
	R24B.PLL_C4_LF = 7;//0.01nF
	
	R26B.ADR           = 26;
    R26B.PLL_DLD_CNT   = 3000;//window of acceptable phase error
    R26B.PLL_CP_GAIN   = 3;   //CHARGE PUMP CURRENT (µA) 0 -0.1 mA |1 - 0.4 mA|2-1.6 mA|3 - 3.2 mA
    R26B.EN_PLL_REF_2X = 0;
	
	R28B.ADR         =  28;
    R28B.PLL_R       = 100; //5 - 100 Mhz   1- 20 MHz
	
	R29B.ADR 	     = 29;
	R29B.OSCin_FREQ  =  1;   // >63 MHz to 127 MHz
    R29B.PLL_N_CAL   =freq;  // 63 - 20MHz
	
	R30B.ADR 	     = 30;
	R30B.PLL_P       =  7;  //  2 -  100 MHz
    R30B.PLL_N       =freq;  // 63 - 100 MHz
	
	R31B.ADR 	       =31;
	R31B.READBACK_ADDR = 0;  // READBACK R0
    R31B.uWire_LOCK    = 0;  // 
}


void FAPCH_B (void)
{
 //   INIT_REG_FAPCH_B ();	

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
	
	spi_FAPCH_B(B_stz.R0);Transf("R0 " );	//8.5.1.1 Special Programming Case for R0 to R5 for CLKoutX_Y_DIV > 25
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
	delay_us(100);
	SYNC_LMK_0;
}

void FAPCH_A (void)
{
 //   INIT_REG_FAPCH_A ();	

	A_stz.R0 =0x00000000;
	A_stz.R0 =	     A_stz.R0|
				(R0B.RST<<17)|
				(R0B.PD <<31)|
				(R0B.DIV<< 5)|
				(R0B.ADR<< 0);
	
	spi_FAPCH_A(A_stz.R0);//сбрасываем микросхемы битом RST
	Transf("R0 - RST\r\n " );	
//------------------------------------------------------------	
	A_stz.R0 =0x00000000;
	A_stz.R0 =	     A_stz.R0|
				(0      <<17)|
				(R0B.PD <<31)|
				(R0B.DIV<< 5)|
				(R0B.ADR<< 0);
				
	A_stz.R1 =0x00000000;
	A_stz.R1 =	     A_stz.R1|
				(R1B.RST<<17)|
				(R1B.PD <<31)|
				(R1B.DIV<< 5)|
				(R1B.ADR<< 0);
				
	A_stz.R2 =0x00000000;
	A_stz.R2 =	     A_stz.R2|
				(0      <<17)|
				(R2B.PD <<31)|
				(R2B.DIV<< 5)|
				(R2B.ADR<< 0);
				
	A_stz.R3 =0x00000000;
	A_stz.R3 =	     A_stz.R3|
				(0      <<17)|
				(R3B.PD <<31)|
				(R3B.DIV<< 5)|
				(R3B.ADR<< 0);
	
	A_stz.R4 =0x00000000;
	A_stz.R4 =	     A_stz.R4|
				(0      <<17)|
				(R4B.PD <<31)|
				(R4B.DIV<< 5)|
				(R4B.ADR<< 0);
				
	A_stz.R5 =0x00000000;
	A_stz.R5 =	     A_stz.R5|
				(0      <<17)|
				(R5B.PD <<31)|
				(R5B.DIV<< 5)|
				(R5B.ADR<< 0);
				
	A_stz.R6 =0x00000000;
	A_stz.R6 =	       A_stz.R6|
				(R6B.TYPE0<<16)|
				(R6B.TYPE1<<20)|
				(R6B.TYPE2<<24)|
				(R6B.TYPE3<<28)|
				(R6B.ADR  << 0);
				
	A_stz.R7 =0x00000000;
	A_stz.R7 =	       A_stz.R7|
				(R7B.TYPE0<<16)|
				(R7B.TYPE1<<20)|
				(R7B.TYPE2<<24)|
				(R7B.TYPE3<<28)|
				(R7B.ADR  << 0);

	A_stz.R8 =0x00000000;
	A_stz.R8 =	       A_stz.R8|
				(R8B.TYPE0<<16)|
				(R8B.TYPE1<<20)|
				(R8B.TYPE2<<24)|
				(R8B.TYPE3<<28)|
				(R8B.ADR  << 0);
				
	A_stz.R9 =0x00000000;
	A_stz.R9 =	                A_stz.R9|
				(0x05		       <<28)|
				(0x05		       <<24)|
				(0x05		       <<20)|
				(0x05		       <<16)|
				(0x05		       <<12)|
				(0x05		       << 8)|
				(0x02		       << 5)|
				(R9B.ADR           << 0);
				
	A_stz.R10 =0x00000000;
	A_stz.R10 =	       A_stz.R10|
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
				

	A_stz.R11 =0x00000000;
	A_stz.R11 =	       		  A_stz.R11|
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

	A_stz.R12 =0x00000000;
	A_stz.R12 =	             A_stz.R12|
				(R12B.SYNC_PLL_DLD<<23)|
				(R12B.LD_TYPE     <<24)|
				(R12B.LD_MUX      <<27)|
				(0x03		      <<18)|
				(0x03		      << 5)|
				(R12B.ADR         << 0);

	A_stz.R13 =0x00000000;
	A_stz.R13 =	               A_stz.R13|
				(R13B.GPout0       <<16)|
				(R13B.READBACK_TYPE<<24)|
				(0x07		       <<27)|
				(0x01		       <<15)|
				(R13B.ADR          << 0);

	A_stz.R14 =0x00000000;
	A_stz.R14 =	               A_stz.R14|
				(R14B.GPout1       <<24)|
				(R14B.ADR          << 0);	
				
	A_stz.R16 =0x00000000;
	A_stz.R16 =	               A_stz.R16|
				(0x03		       <<30)|
				(0x01		       <<24)|
				(0x01		       <<22)|
				(0x01		       <<20)|
				(0x01		       <<18)|
				(0x01		       <<16)|
				(0x01		       <<10)|
				(R16B.ADR          << 0);

	A_stz.R24 =0x00000000;
	A_stz.R24 =	           A_stz.R24|
				(R24B.PLL_R3_LF<<16)|
				(R24B.PLL_R4_LF<<20)|
				(R24B.PLL_C3_LF<<24)|
				(R24B.PLL_C4_LF<<28)|
				(R24B.ADR      << 0);

	A_stz.R26 =0x00000000;
	A_stz.R26 =	              A_stz.R26|
				(R26B.PLL_DLD_CNT  << 6)|
				(R26B.PLL_CP_GAIN  <<26)|
				(R26B.EN_PLL_REF_2X<<29)|
				(0x02		       <<30)|
				(0x07		       <<23)|
				(0x01		       <<21)|
				(R26B.ADR          << 0);			

	A_stz.R28 =0x00000000;
	A_stz.R28 =	               A_stz.R28|
				(R28B.PLL_R        <<20)|
				(R28B.ADR          << 0);
				
	A_stz.R29 =0x00000000;
	A_stz.R29 =	            A_stz.R29|
				(R29B.OSCin_FREQ<<24)|
				(R29B.PLL_N_CAL << 5)|
				(0x01		    <<23)|
				(R29B.ADR       << 0);
	
	A_stz.R30 =0x00000000;
	A_stz.R30 =	       A_stz.R30|
				(R30B.PLL_P<<24)|
				(R30B.PLL_N<< 5)|
				(R30B.ADR  << 0);
				
	A_stz.R31 =0x00000000;
	A_stz.R31 =	               A_stz.R31|
				(R31B.READBACK_ADDR<<16)|
				(R31B.uWire_LOCK   << 5)|
				(R31B.ADR          << 0);
				
				
//программируем синтезатор A
	spi_FAPCH_A(A_stz.R0);Transf("R0 " );	
	spi_FAPCH_A(A_stz.R1);Transf("R1 " );	
	spi_FAPCH_A(A_stz.R2);Transf("R2 " );	
	spi_FAPCH_A(A_stz.R3);Transf("R3 " );	
	spi_FAPCH_A(A_stz.R4);Transf("R4 " );	
	spi_FAPCH_A(A_stz.R5);Transf("R5 " );	
	
	spi_FAPCH_A(A_stz.R0);Transf("R0 " );	
	spi_FAPCH_A(A_stz.R1);Transf("R1 " );	
	spi_FAPCH_A(A_stz.R2);Transf("R2 " );	
	spi_FAPCH_A(A_stz.R3);Transf("R3 " );	
	spi_FAPCH_A(A_stz.R4);Transf("R4 " );	
	spi_FAPCH_A(A_stz.R5);Transf("R5 " );
	
	spi_FAPCH_A(A_stz.R6);Transf("R6 " );	
	spi_FAPCH_A(A_stz.R7);Transf("R7 " );	
	spi_FAPCH_A(A_stz.R8);Transf("R8 " );
	spi_FAPCH_A(A_stz.R9);Transf("R9 " );
	spi_FAPCH_A(A_stz.R10);Transf("R10 " );	
	spi_FAPCH_A(A_stz.R11);Transf("R11 " );	
	spi_FAPCH_A(A_stz.R12);Transf("R12 " );
	spi_FAPCH_A(A_stz.R13);Transf("R13 " );	
	spi_FAPCH_A(A_stz.R14);Transf("R14 " );	
	spi_FAPCH_A(A_stz.R16);Transf("R16 " );	
	spi_FAPCH_A(A_stz.R24);Transf("R24 " );	
	spi_FAPCH_A(A_stz.R26);Transf("R26 " );	
    spi_FAPCH_A(A_stz.R28);Transf("R28 " );	
	spi_FAPCH_A(A_stz.R29);Transf("R29 " );	
	spi_FAPCH_A(A_stz.R30);Transf("R30 " );	
	spi_FAPCH_A(A_stz.R31);Transf("R31\r\n" );	
	//----------------------------
	SYNC_LMK_0;
	delay_us(100);
	SYNC_LMK_1;
	delay_us(100);
	SYNC_LMK_0;
}

void init_FAPCH (u8 a)
{

	 Transf("\r\n" );
	 Transf("-----------------------------------\r\n" );
	 Transf("Программирую REF=100 MHz,ФАПЧ_A:\r" );
	 INIT_REG_FAPCH_A_100MHz(357);
     FAPCH_A();
	 Transf("...\r" );
	 Transf("Выполненно!\r" );
	 Transf("\r\n" );	
	
	 //----------------------------------------------
	 Transf("\r\n" );
	 Transf("-----------------------------------\r\n" );
	 Transf("Программирую REF=100 MHz, ФАПЧ_B:\r" );
	 INIT_REG_FAPCH_B_100MHz();
     FAPCH_B();
	 Transf("...\r" );
	 Transf("Выполненно!\r" );
	 Transf("\r\n" );	
}

void GEN_360 (u32 freq)
{
	 INIT_REG_FAPCH_A_100MHz(freq);
     FAPCH_A();
}

void PWR_B (u32 freq)
{
	 INIT_REG_FAPCH_A_100MHz(freq);
     FAPCH_A();
}
//--------------------------------
void init_I2C(void)
{

uint32_t I2C_speed;

I2C_speed = 100000;   // 1000Hz
I2C_InitTypeDef  I2C_InitStructure;

  /*enable I2C*/
 
  // Включаем тактирование нужных модулей
   /* I2C1 clock enable */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

     /* I2C1 SDA and SCL configuration */
    // I2C использует две ноги микроконтроллера, их тоже нужно настроить
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);//на всякий случай перезапустим и2с

   /* I2C1 configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = I2C_speed ;

    I2C_Init(I2C1, &I2C_InitStructure);
    I2C_Cmd(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);

    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_SetPriority(I2C1_EV_IRQn,1);
}

void I2C_StartTransmission(I2C_TypeDef* I2Cx, u8 transmissionDirection,  u8 slaveAddress)
{
 //  sendT("\r\nшина I2C\r\n");
 // На всякий слуыай ждем, пока шина осовободится
  timeOUT_I2C=0;

    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
    {
      if (timeOUT_I2C>I2C_delay) break;
    }
 
    // Генерируем старт - тут все понятно )
    I2C_GenerateSTART(I2Cx, ENABLE);
   
    // Ждем пока взлетит нужный флаг
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
    {
      if (timeOUT_I2C>I2C_delay) break;
    }

    // Посылаем адрес подчиненному
    I2C_Send7bitAddress(I2Cx, slaveAddress, transmissionDirection);


    // А теперь у нас два варианта развития событий - в зависимости от выбранного направления обмена данными
    if(transmissionDirection== I2C_Direction_Transmitter)
    {
        while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
        {
          if (timeOUT_I2C>I2C_delay) break;
        }
    }

    if(transmissionDirection== I2C_Direction_Receiver)
    {
  
      while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
      {
        if (timeOUT_I2C>I2C_delay) break;
      }
      while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
      {
        if (timeOUT_I2C>I2C_delay) break;
      }

    }
}

/*******************************************************************/
void I2C_WriteData(I2C_TypeDef* I2Cx, u8 data)
{
  // Просто вызываем готоваую функцию из SPL и ждем, пока данные улетят
  // sendT("шлём данные по I2c\r\n");

    I2C_SendData(I2Cx, data);
   
      /* Test on EV8 and clear it */
    timeOUT_I2C=0;
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
       if (timeOUT_I2C>I2C_delay) break;
    }; /*!< EV8 */
      //while ((I2C1->SR1 & 0x00004) != 0x000004){}
}

/*******************************************************************/
u8 I2C_ReadData(I2C_TypeDef* I2Cx)
{
  char data;
    // Тут картина похожа, как только данные пришли быстренько считываем их и возвращаем
  timeOUT_I2C=0;
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) )
    	{
    		Circl_FUNC ();
        if (timeOUT_I2C>I2C_delay) break;
    	};
    data = I2C_ReceiveData(I2Cx);
    return data;
}

 //  I2C_GenerateSTOP(I2Cx, ENABLE)

// Вот так вот получилось. При использовании I2C для передачи данных, например, последовательность действий должна быть такой:
// 1. Инициализируем модуль I2C, нужные ножки контроллера, ну и все остальное
// 2. Посылаем «старт»
// 3. Шлем данные
// 4. Генерируем «стоп»

/*******************************************************************/


void i2c_write (u8 adr,u8 data)
{
     
 I2C_StartTransmission(I2C1 ,I2C_Direction_Transmitter,adr<<1);//0x68<<1
 I2C_WriteData(I2C1,data);
 I2C_GenerateSTOP(I2C1, ENABLE);
    /*stop bit flag*/
 timeOUT_I2C=0;

    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF))
    	{
        if (timeOUT_I2C>I2C_delay) break;
    	};
}


void i2c_write_to_adr (u8 adr,u8 adr_reg,u8 data)
{
    I2C_StartTransmission(I2C1 ,I2C_Direction_Transmitter,adr<<1);//0x68<<1
    I2C_WriteData(I2C1,adr_reg); //запись адреса регистра
    I2C_WriteData(I2C1,data);   // запись данных в регистр
    I2C_GenerateSTOP(I2C1, ENABLE);
    /*stop bit flag*/
    timeOUT_I2C=0;
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF))
    	{
        if (timeOUT_I2C>I2C_delay) break;
    	};
}


u8 i2c_read (u8 adr)
{
  u8 data;
 
    I2C_StartTransmission(I2C1 ,I2C_Direction_Receiver,adr<<1);//0x68<<1
    data=I2C_ReadData(I2C1);
    I2C_AcknowledgeConfig(I2C1, DISABLE);
    I2C_GenerateSTOP(I2C1, ENABLE);
  
    /*stop bit flag*/
    timeOUT_I2C=0;
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF)) 
      {
        if (timeOUT_I2C>I2C_delay) break;
      }
    return data;
}

//--------------------------------
int main(void)
{
    int i;
  
    init_RCC   ();
    init_GPIO  ();
    init_USART1();
    init_USART2();
    init_ADC   ();
//   init_TIM    ();
    init_EXT   ();
    init_SPI2  ();
//	init_SPI1  (); 
    init_I2C   ();
 // _enable_irq ();

 	Transf("\r\n");
    Transf("------");
    Transf("\r\n");
    Transf("работаем!\r\n");
    Transf("------");
    Transf("\r\n");

    Menu1(0);
	
//-----------ИНИЦИАЛИЗАЦИЯ-------------
I2C_delay=100;
//----------------------------------------
IO("~0 sel:0;");
Massiv_dbm(1); //расчёт массива ДБ для детектора

init_FAPCH (100);

while(1)
 {
//----------------UART------------------
      UART_conrol ();
      UART_DMA_TX (); //отправка по DMA сообщений 
//---------------------------------------------        
      LED_control (); // управление индикаторными светодиодами
      WDG_func(); //функция контроля внешнего вочдога
  }
}

