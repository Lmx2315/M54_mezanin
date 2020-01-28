#include "periph.h"
#include "stm32f10x.h"
#include "RTE_Device.h"
#include "gpio_init.h"

// RCC setup constants
const RCC_TypeDef RCC_ = {0x03035683, 0x001D040A, 0, 0, 0, 0x00000017, 0x0000687D, 0x003EC03F, 0, 0};
const RCC_TypeDef RCC_MASK = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0};

//AFIO setup constants
const AFIO_TypeDef AFIO_ = {0,0x0200000E, 0, 0, 0, 0x00000010};
const AFIO_TypeDef AFIO_MASK = {0,0xF8FFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};

// GPIO setup constants

const GPIO_TypeDef GPIO_A = {0x44444444, 0x64464444, 0, 0x00000000, 0, 0, 0};
const GPIO_TypeDef GPIO_B = {0x8BB4B464, 0x34468BFF, 0, 0x00000880, 0, 0, 0};
const GPIO_TypeDef GPIO_C = {0x44446686, 0x666B8B44, 0, 0x00000803, 0, 0, 0};
const GPIO_TypeDef GPIO_D = {0x38B83863, 0x44444444, 0, 0x000000C8, 0, 0, 0};
const GPIO_TypeDef GPIO_E = {0x66633383, 0x44444444, 0, 0x00000003, 0, 0, 0};

const GPIO_TypeDef GPIO_A_MASK = {0xFFFFFFFF,0xFFFFFFFF,0x00000000,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF};
const GPIO_TypeDef GPIO_B_MASK = {0xFFFFFFFF,0xFFFFFFFF,0x00000000,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF};
const GPIO_TypeDef GPIO_C_MASK = {0xFFFFFFFF,0xFFFFFFFF,0x00000000,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF};
const GPIO_TypeDef GPIO_D_MASK = {0xFFFFFFFF,0xFFFFFFFF,0x00000000,0xFFFFFF7E,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF};
const GPIO_TypeDef GPIO_E_MASK = {0xFFFFFFFF,0xFFFFFFFF,0x00000000,0xFFFFFFE1,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF};

// SPI3 setup constants
const SPI_TypeDef SPI_3 = {0x0344, 0, 0x0003, 0};
const SPI_TypeDef SPI_3_MASK = {0xFFFF, 0,  0xFFFF, 0};

// DMA setup constants

typedef struct
{
  __IO uint32_t CCR;
  __IO void* CNDTR;
  __IO void* CPAR;
  __IO void* CMAR;
} DMA_Channel_TypeDef_;

const DMA_Channel_TypeDef_ DMA2_1 = {0x00001082, 0,&(SPI3->DR),0};
const DMA_Channel_TypeDef_ DMA2_2 = {0x00001092, 0,&(SPI3->DR),0};

const DMA_Channel_TypeDef DMA_MASK = {0xFFFFFFFF, 0,0xFFFFFFFF,0xFFFFFFFF};
const DMA_Channel_TypeDef DMA_MASK_2 = {0xFFFFFFFF, 0,0xFFFFFFFF,0};

void Init_Periph(void* Periph, void* Struct, void* MaskStruct, uint32_t Length)
{
	uint32_t i;
	static uint32_t mask;
	for (i=0;i<Length;i+=4)
	{
		mask =(*(uint32_t*)((uint32_t)MaskStruct+i));
		if (mask)
			(*(uint32_t*)((uint32_t)Periph+i)) = (*(uint32_t*)((uint32_t)Struct+i));
	}
}

void Init_Periphs(void)
{
	Init_Periph(RCC,(void*)&RCC_,(void*)&RCC_MASK,sizeof(RCC_));
	Init_Periph(AFIO,(void*)&AFIO_,(void*)&AFIO_MASK,sizeof(AFIO_));
	Init_Periph(GPIOA,(void*)&GPIO_A,(void*)&GPIO_A_MASK,sizeof(GPIO_A));
	Init_Periph(GPIOB,(void*)&GPIO_B,(void*)&GPIO_B_MASK,sizeof(GPIO_B));
	
	GPIOC->ODR = 0x0001;
	Init_Periph(GPIOC,(void*)&GPIO_C,(void*)&GPIO_C_MASK,sizeof(GPIO_C));
	Init_Periph(GPIOD,(void*)&GPIO_D,(void*)&GPIO_D_MASK,sizeof(GPIO_D));
	Init_Periph(GPIOE,(void*)&GPIO_E,(void*)&GPIO_E_MASK,sizeof(GPIO_E));

	Init_Periph(SPI3,(void*)&SPI_3,(void*)&SPI_3_MASK,sizeof(SPI_3));

	Init_Periph(DMA2_Channel1,(void*)&DMA2_1,(void*)&DMA_MASK_2,sizeof(DMA2_1));
	Init_Periph(DMA2_Channel2,(void*)&DMA2_2,(void*)&DMA_MASK_2,sizeof(DMA2_2));
}
