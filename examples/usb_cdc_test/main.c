// usb_cdc_test
// usb cdc implementation from
// https://github.com/andreasb242/CH55xG-StateLight

/*
TODO:
[ ] Modify UsbCdc_processInput() to handle received data
[ ] Modify / remove prepareUsbIds(), we don't want to store IDs in EEPROM
[ ] Modify / remove readNameFromEeprom() 
*/

#include <ch554.h>
#include <debug.h>
#include "inc.h"
#include "hardware.h"
#include "usb-cdc.h"

#define LED_PIN 3
SBIT(LED, 0xB0, LED_PIN); // 0x90 is Port 1 0xB0 is Port 3

#define BOOT_PB 6		// BOOT Pushbutton on P3.6
SBIT(BOOT, PORT3, BOOT_PB);


/**
 * Interrupt needs to be here in the Main file
 * else it simple won't be called.
 */
void DeviceInterrupt(void) __interrupt(INT_NO_USB) {
	usbInterrupt();
}


void main() {
	
	// CH55x clock configuration
    CfgFsys();
	
	// Wait for clock frequency to stabilize
	mDelaymS(5);
	
	// Enable USB Port
	USBDeviceCfg();
	
	// Endpoint configuration
	USBDeviceEndPointCfg();	

   	// Interrupt initialization
	USBDeviceIntCfg();
	
	// Endpoint send length must be cleared
	UEP0_T_LEN = 0;
	UEP1_T_LEN = 0;
	UEP2_T_LEN = 0;
    
    // Configure Pin 3.3 as GPIO output
    P3_MOD_OC &= ~(1<<LED_PIN); // Set Push pull output mode
    P3_DIR_PU |= (1<<LED_PIN);  // Set to output mode
	
	// Set up programming pushbutton
	P3_MOD_OC |= (1<<BOOT_PB);	// Open drain	
	P3_DIR_PU |= (1<<BOOT_PB);	// pull-up enabled	
    
  

    while (1) 
	{
		UsbCdc_processOutput();
		UsbCdc_processInput();

		mDelaymS(10);
        LED = !LED;
		
		if (BOOT == 1)
			break; 		
    }
	
	jumpToBootloader();
}
