#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h" 
#include "stdbool.h"

extern unsigned char Count_timer;
extern unsigned char Flag_timer_2S;
extern unsigned char Count_timer_2;
extern unsigned char Flag_timer_Accumulated_Mileage_1S;
extern unsigned char rtc_hour;
extern unsigned char rtc_min;
extern unsigned char rtc_sec;

extern bool Flag_state_Accumulated_Mileage;
void TIM3_Int_Init(u16 arr,u16 psc);
#endif
