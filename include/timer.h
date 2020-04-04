/* timer.h
Utility to configure and set up the timer.
Provides use of millis();



*/


#define enable_timer0_interrupt() (ET0 = 1)   // Enable timer 0 interrupt 
void timer0_init();
inline uint32_t millis();
void timer0_ISR() __interrupt (INT_NO_TMR0); // 