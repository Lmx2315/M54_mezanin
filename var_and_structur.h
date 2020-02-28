#include "stm32f10x.h"
#include "stm32f10x_gpio.h"

#define  u8  unsigned char
#define  u16 unsigned short
#define  u32 unsigned int


#define int8_t   char
#define int16_t  short
#define uint8  unsigned char
#define uint16 unsigned short
#define uint32 unsigned int
#define int16  short
#define u64    unsigned long long
#define int64  long long

#define  ON  1
#define  OFF 0

u32 SYNC_T0Tnc;
u32 START_packet;

u32 TNC_actual; //текущий ТНЦ	
u32 TNO_actual; //текущий ТНО
u32 CEV_actual; //текущий СЕВ
u32 FLAG_TOBM;
u32 FLAG_START;

u8 BTN_var;
u8 BTN_temp;

u8 flag_INT;

u8 state_INT5;
u8 state_INT6;
u8 state_INT7;
u8 state_INT8;
u8 state_INT9;
u8 state_INT10;
u8 state_INT11;
u8 state_INT12;
u8 state_INT13;
u8 state_INT14;
u8 state_INT15;
u8 INT_var;

u32 sch_packet_UDP_reciv;
u8  flag_RX_UDP;
u32 RX_resive_MAX;
u32 lsr_razmer;
u32 TIMER_UDP_LS_DOWN;//счётчик времени контроля работы линии UDP ETHERNET
u16 text_lengh_TCP; //длинна отправляемых данных


#define RX_MAX_BUF_SIZE  200
#define TX_MAX_BUF_SIZE  200
#define ZX_MAX_BUF_SIZE  170

u8 RX_BUF[RX_MAX_BUF_SIZE];
u8 TX_BUF[TX_MAX_BUF_SIZE];

u8 text_buffer_TCP[ZX_MAX_BUF_SIZE];

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

#define PD1_0  GPIOD->BRR = GPIO_Pin_1
#define PD1_1  GPIOD->BSRR= GPIO_Pin_1

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

#define PC3_0  GPIOC->BRR   = GPIO_Pin_3
#define PC3_1  GPIOC->BSRR  = GPIO_Pin_3

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

#define PA4_0  GPIOA->BRR  = GPIO_Pin_4
#define PA4_1  GPIOA->BSRR = GPIO_Pin_4  

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

#define PB8_0  GPIOB->BRR   = GPIO_Pin_8
#define PB8_1  GPIOB->BSRR  = GPIO_Pin_8

#define PB9_0  GPIOB->BRR   = GPIO_Pin_9
#define PB9_1  GPIOB->BSRR  = GPIO_Pin_9

#define PB12_0  GPIOB->BRR   = GPIO_Pin_12
#define PB12_1  GPIOB->BSRR  = GPIO_Pin_12

#define PB13_0  GPIOB->BRR   = GPIO_Pin_13
#define PB13_1  GPIOB->BSRR  = GPIO_Pin_13

#define PB15_0  GPIOB->BRR   = GPIO_Pin_15
#define PB15_1  GPIOB->BSRR  = GPIO_Pin_15

#define SYNC_LMK_0  PC7_0
#define SYNC_LMK_1  PC7_1

#define CS_AT45_0  PA12_0
#define CS_AT45_1  PA12_1

#define WDG_on   PA11_0
#define WDG_off  PA11_1

#define LED1_0  PE6_1
#define LED1_1  PE6_0

#define LED1_C2_0  PD10_1
#define LED1_C2_1  PD10_0

#define LED2_C2_0  PD11_1
#define LED2_C2_1  PD11_0

#define LED3_0  PE2_0
#define LED3_1  PE2_1

#define LED4_0  PE3_0
#define LED4_1  PE3_1

#define B_LE_WIRE_LMK_1 PD3_1
#define B_LE_WIRE_LMK_0 PD3_0

#define A_LE_WIRE_LMK_1 PD2_1
#define A_LE_WIRE_LMK_0 PD2_0

#define SEL_ETALON_0 PC3_0
#define SEL_ETALON_1 PC3_1

#define TST1_BUF(a)         ((a==1)?PВ12_1:PВ12_0)
#define SEL_ETALON(a)       ((a==1)?PC3_1 :PC3_0 )
#define RESET_BUF(a)        ((a==1)?PE1_1 :PE1_0 )
#define ENABLE_LM25056(a)   ((a==1)?PD7_1 :PD7_0 )

