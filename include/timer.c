/* timer.c
    
*/
#include <ch554.h>
#include <debug.h>
#include "timer.h"

#define CPU_FREQ 16 // Mhz
#define RELOAD_VALUE 56 // This gives 200 counts = 50us
//#define MICROSECONDS_PER_SECOND 1000000 // 1000000us = 1s
#define OVERFLOWS 20 // how many overflows for 1 ms
//#define OVERFLOWS ((MICROSECONDS_PER_SECOND/((256-RELOAD_VALUE)*(1*100/(CPU_FREQ/4))))/100) //

__xdata volatile uint32_t millis_cnt = 0;

// Set timer up for millis count
void timer0_init()
{
    TMOD &= ~bT0_CT;  // Set to timing mode
    TMOD &= ~bT0_M0; //
    TMOD |= bT0_M1; // MODE 2 - 8 bit reload timer 
    TH0 = RELOAD_VALUE; // When timer0 overflows, this value gets loaded as the initial value.
    T2MOD |= bT0_CLK; // Set Timer0 clock to Fsys/4 = 16 Mhz to 4 Mhz = 250 ns period
    T2MOD &= ~bTMR_CLK; // Set to use divided /4 clock
    TR0 = 1;  // Start Timer
}

void timer0_ISR() __interrupt (INT_NO_TMR0)
{   
    static volatile uint8_t overflows = 0;
    // Interrupt flag auto cleared.
    overflows++;
    if(overflows >= OVERFLOWS)
    {
        overflows = 0;
        millis_cnt++; // increment millis
    }
}

inline uint32_t millis()
{
    return millis_cnt;
}
