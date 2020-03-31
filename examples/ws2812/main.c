// Blink an LED connected to pin 3.1

#include <stdint.h>

#include <ch554.h>
#include <debug.h>
#include <bootloader.h>
#include "bitbang.h"

#define BUILTIN_LED 3   // Built in led
SBIT(LED_ONBOARD, 0xB0, BUILTIN_LED); // 0x90 is Port 1 0xB0 is Port 3

#define PROG_PB 6	// Pushbutton on P3.4
SBIT(PROG, 0xB0, PROG_PB);

#define LED_COUNT (61)
__xdata uint8_t led_data[LED_COUNT*3];
__xdata uint32_t millis_cnt;

void rgbPattern()
{
    static uint8_t counter = 0;
    static uint8_t step = 0;

    uint8_t i;

    // Shift LED data to the end of the array
	for (i = LED_COUNT-1; i > 0; i--)
	{

		led_data[i*3+1] = led_data[(i-1)*3+1];
		led_data[i*3+0] = led_data[(i-1)*3+0];
		led_data[i*3+2] = led_data[(i-1)*3+2];
    }

	const uint8_t intensity = 30;
	
    // Write the next color to the first LED data
	
	switch (step)
	{
		case 0:		// Green to Red
			led_data[1] = counter;
			led_data[0] = intensity - counter;
			led_data[2] = 0;
			break;
			
		case 1:		// Red to Blue
			led_data[1] = intensity - counter;
			led_data[0] = 0;
			led_data[2] = counter;
			break;	

		case 2:		// Blue to Green
			led_data[1] = 0;
			led_data[0] = counter;
			led_data[2] = intensity - counter;
			break;	
		
	}
	
	// Increment counters
	if (counter == intensity)
	{
		counter = 0;
		if (step == 2)
			step = 0;
		else
			step++;
	}
	else
	{
		counter += 1;
	}

}

void main() {
    CfgFsys();
    mDelaymS(5);
    
	// Set up onboard LED output
    P3_MOD_OC &= ~(1<<BUILTIN_LED); // Set Push pull output mode
    P3_DIR_PU |= (1<<BUILTIN_LED);  // Set to output mode
	
	// Set up programming pushbutton
	P3_MOD_OC |= (1<<PROG_PB);	// Open drain	
	P3_DIR_PU |= (1<<PROG_PB);	// pull-up enabled
	
	
    bitbangSetup(); // ws2812 pin is setup here
    

    while (1) {
        rgbPattern();
        
        bitbangWs2812(LED_COUNT, led_data);
        LED_ONBOARD = !LED_ONBOARD;
        mDelaymS(5);
		
		if (PROG == 1)
			break;
    }
	
	// Turn off WS2812's before starting bootloader
	uint16_t i;
	for (i = 0; i < LED_COUNT*3; i++)
		led_data[i] = 0;
	bitbangWs2812(LED_COUNT, led_data);

	
	EA = 0;	// Disable interrupts
	mDelaymS(100);
	
	bootloader();
	while(1);
	
}
