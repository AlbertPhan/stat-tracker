// Blink an LED connected to pin 3.3

#include <ch554.h>
#include <debug.h>

#define LED_PIN 3
SBIT(LED, 0xB0, LED_PIN); // 0x90 is Port 1 0xB0 is Port 3

void main() {
    CfgFsys();

    // Configure pin 1.7 as GPIO output
    //P1_DIR_PU &= 0x0C; // what is this
    
    
    //P1_MOD_OC = P1_MOD_OC & ~(1<<LED_PIN);
    //P1_DIR_PU = P1_DIR_PU |	(1<<LED_PIN);
    
    
    // Configure Pin 3.3 as GPIO output
    P3_MOD_OC &= ~(1<<LED_PIN); // Set Push pull output mode
    P3_DIR_PU |= (1<<LED_PIN);  // Set to output mode
    
  

    while (1) {
    	mDelaymS(100);
        LED = !LED;
    }
}
