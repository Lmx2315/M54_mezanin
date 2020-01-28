#ifndef HAL_H
#define HAL_H

#include "stm32f10x.h"
#include "RTE_Device.h"
#include "bitbanding.h"

#define UNIQUE_ID ((uint8_t*)0x1FFFF7E8)

__STATIC_INLINE void GPIO_Bit_Reset(GPIO_TypeDef *GPIO, uint16_t mask)
{
	GPIO->BRR = mask;
}

__STATIC_INLINE void GPIO_Bit_Set(GPIO_TypeDef *GPIO, uint16_t mask)
{
	GPIO->BSRR = mask;
}

__STATIC_INLINE void GPIO_Bit_Set_Reset(GPIO_TypeDef *GPIO, uint32_t mask)
{
	*(__IO uint32_t*)&GPIO->BSRR = mask;
}

__STATIC_INLINE uint32_t GPIO_Get_Data(GPIO_TypeDef *GPIO)
{
	return GPIO->IDR;
}

#endif
