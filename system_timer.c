#include "hal.h"
#include "system_timer.h"
#include "mutex.h"
#include "bitbanding.h"

typedef __packed struct
{
	uint8_t lock:7;
	uint8_t event:1;
} Timer_Status_Typedef;

typedef struct
{
    uint8_t mode;
		Timer_Status_Typedef status;
    uint16_t counter;
    uint16_t compare;
#ifdef SYSTEM_TIMER_HANDLER_ENABLE
	void (*handler)();
#endif
} System_Timer_t;

volatile System_Timer_t System_Timers[SYSTEM_TIMER_COUNT];

uint8_t System_Timer_Get_Status(int8_t TimerId)
{
  if ((TimerId < SYSTEM_TIMER_COUNT) && (TimerId >= 0))
	  if (System_Timers[TimerId].mode != tm_Free)
		  return System_Timers[TimerId].status.event;
	return 0;
}

void System_Timer_Reset_Status(int8_t TimerId)
{
  if ((TimerId < SYSTEM_TIMER_COUNT) && (TimerId >= 0))
		BIT_BAND_SRAM(&System_Timers[TimerId].status, 7) = 0;		
}	

void System_Timer_Lock(int8_t TimerId)
{
  if ((TimerId < SYSTEM_TIMER_COUNT) && (TimerId >= 0))
    Atomic8AddAndFetch((uint8_t*)&System_Timers[TimerId].status, 1);
}

void System_Timer_Unlock(int8_t TimerId)
{
  if ((TimerId < SYSTEM_TIMER_COUNT) && (TimerId >= 0))
    Atomic8AddAndFetch((uint8_t*)&System_Timers[TimerId].status, -1);
}

void System_Timer_Delete(int8_t TimerId)
{
  if ((TimerId < SYSTEM_TIMER_COUNT) && (TimerId >= 0))
	{
    System_Timers[TimerId].mode = tm_Free;
		__CLREX();
	}
}

void Init_System_Timer(void)
{
  uint8_t i;
  NVIC_SetPriority(SysTick_IRQn, 15);
  for(i = 0; i < SYSTEM_TIMER_COUNT; i++)
    System_Timer_Delete(i);
  SysTick_Config(RTE_HCLK / SYSTEM_TIMER_FREQUENCY -1);
}

int8_t System_Timer_Set_Mode(int8_t TimerId, Timer_Mode_t mode) //if OK - return 1
{
  int8_t temp = 0;
  if ((TimerId < SYSTEM_TIMER_COUNT) && (TimerId >= 0))
  {  
    if (mode != tm_Free)
    {
      do
      {
         temp = __LDREXB(&System_Timers[TimerId].mode);
         if (temp != tm_Free)
           temp = mode;
      }while(__STREXB(temp, &System_Timers[TimerId].mode));
    }
  }
return temp != 0;
}

void System_Timer_Clear_Counter(int8_t TimerId)
{
  if ((TimerId < SYSTEM_TIMER_COUNT) && (TimerId >= 0))
  {  
    System_Timers[TimerId].counter = 1;
		__CLREX();
  }
}

void System_Timer_Set_Counter(int8_t TimerId, uint16_t Counter)
{
  if ((TimerId < SYSTEM_TIMER_COUNT) && (TimerId >= 0))
  {  
    System_Timers[TimerId].counter = Counter;
		__CLREX();
  }
}

void System_Timer_Set_Compare(int8_t TimerId, uint16_t compare)
{
  if ((TimerId < SYSTEM_TIMER_COUNT) && (TimerId >= 0))
  {  
    System_Timers[TimerId].compare = compare;
  }
}

uint16_t System_Timer_Get_Counter(int8_t TimerId)
{
  uint16_t counter = 0;
  if ((TimerId < SYSTEM_TIMER_COUNT) && (TimerId >= 0))
  {
    counter = System_Timers[TimerId].counter;
  }
  return counter;
}

int8_t System_Timer_Add(Timer_Mode_t mode, uint16_t compare
#ifdef SYSTEM_TIMER_HANDLER_ENABLE
																														,void (*handler)()
#endif
																																							)
																															
{
  uint8_t i;
  Timer_Mode_t temp;
  uint8_t result;
  if(mode != tm_Free)
  {
    for (i = 0; i < SYSTEM_TIMER_COUNT; i++)
    {
      do
      {
        result = 2;
        temp = (Timer_Mode_t)__LDREXB(&System_Timers[i].mode);
        if (temp == tm_Free)
          result = __STREXB(tm_Stop, &System_Timers[i].mode);
        else
          __CLREX();
      }while(result == 1);
      if (!result)
      {
        System_Timers[i].counter = 1;
        System_Timers[i].compare = compare;
#ifdef SYSTEM_TIMER_HANDLER_ENABLE
        System_Timers[i].handler = handler;
#endif
        *((uint8_t*)&System_Timers[i].status) = 0;
        System_Timers[i].mode = mode;
        return i;
      }
    }
  }
  return -1;
}

void SysTick_Handler(void)
{
  uint8_t i;
#ifdef SYSTEM_TIMER_HANDLER_ENABLE
  void (*handler)();
#endif
  Timer_Mode_t mode; 
  for(i = 0; i < SYSTEM_TIMER_COUNT; i++)
  {
    if ((!(System_Timers[i].status.lock)) && (System_Timers[i].mode > tm_Stop))
    {
      if(System_Timers[i].counter >= System_Timers[i].compare)
      {
        System_Timers[i].counter = 1;
#ifdef SYSTEM_TIMER_HANDLER_ENABLE
        handler = System_Timers[i].handler;
#endif
        do
        {
          mode = (Timer_Mode_t)__LDREXB(&(System_Timers[i].mode));
          if (mode == tm_Once)
            mode = tm_Stop;
        }while(__STREXB(mode, &System_Timers[i].mode));
				BIT_BAND_SRAM(&System_Timers[i].status,7) = 1;
#ifdef SYSTEM_TIMER_HANDLER_ENABLE
        if ((handler) && (mode))
          handler();
#endif
      }
      else
        Atomic16AddAndFetch(&System_Timers[i].counter, 1);
    }
  }
}
