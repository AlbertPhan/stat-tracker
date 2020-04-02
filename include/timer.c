/* timer.c
    
*/
#include <ch554.h>
#include <debug.h>
#include "timer.h"

#define RELOAD_VALUE 56 // This gives 200 counts = 100 us

void timer0_Init()
{
    TMOD &= ~bT0_CT;  // Set to timing mode
    TMOD &= ~bT0_M0; //
    TMOD |= bT0_M1; // MODE 2 - 8 bit reload timer 
    TH0 = RELOAD_VALUE; // When timer0 overflows, this value gets loaded as the initial value.

    T2MOD &= ~bT0_CLK; // Set Timer0 clock to Fsys/12 = 24 Mhz to 2 Mhz = 500 ns period
    T2MOD &= ~bTMR_CLK; // Set to use divided /12 clock
    TR0 = 1;  // Start Timer
    
}

void timer0_ISR() __interrupt (INT_NO_TMR0)
{   
    static uint32_t millis = 0;
    static uint8_t overflows = 0;

    // Interrupt flag auto cleared.
    // Every overflow is 200 * 500 ns = 100 us
    overflows++;
    if(overflows == 10)
    {
        overflows = 0;
        millis++; // increment millis
    }
}