#ifndef SYSTEM_TIMER_H
#define SYSTEM_TIMER_H

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------
#ifndef SYSTEM_TIMER_COUNT
// <h> System Timer Initialization
//   <o> Count: <1-20>
//   <i> System-Timer count setup
#define SYSTEM_TIMER_COUNT                25
#endif
#ifndef SYSTEM_TIMER_FREQUENCY
//   <o> Frequency Hz <5-2000>
//   <i> System-Timer Frequency setup
#define SYSTEM_TIMER_FREQUENCY            1000
// </h>
#endif
#include <stdint.h>

typedef enum
{
 tm_Free,
 tm_Stop,
 tm_Repeat,
 tm_Once
} Timer_Mode_t;


int8_t System_Timer_Add(Timer_Mode_t mode, uint16_t compare
#ifdef SYSTEM_TIMER_HANDLER_ENABLE
																														,void (*handler)()
#endif
																																							);
void Init_System_Timer(void);
uint8_t System_Timer_Get_Status(int8_t TimerId);
void System_Timer_Reset_Status(int8_t TimerId);
int8_t System_Timer_Set_Mode(int8_t TimerId, Timer_Mode_t mode);
void System_Timer_Clear_Counter(int8_t TimerId); 
void System_Timer_Set_Compare(int8_t TimerId, uint16_t compare); 
uint16_t System_Timer_Get_Counter(int8_t TimerId);
void System_Timer_Set_Counter(int8_t TimerId, uint16_t Counter);
void System_Timer_Delete(int8_t TimerId);
void System_Timer_Lock(int8_t TimerId);
void System_Timer_Unlock(int8_t TimerId);
#endif
