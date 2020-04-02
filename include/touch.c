/* touch.c
Albert Phan
Date April 2 2020

Description: CH552 - touch key sampling and interval setting, channel selection, conversions and processing touch data


*/

#include <ch554.h>
#include <debug.h>
#include "touch.h"

/****************************************************************
Function Name   : getNokeyData()
Description     : Samples TOUCH_AVERAGING_SIZE times and averages to get a nokey pressed baseline capacitance value. 
Input           : channel to sample (eg. 1 for TIN1)
Output          : none
Return          : nokey pressed data
****************************************************************/
uint16_t getNokeyData(uint8_t channel)
{
    uint32_t sum = 0;
    uint8_t i;

	// Sample many times
    TKEY_CTRL = TKEY_CTRL & 0b11111000 | channel+1; // Select channel
	for(i = 0; i < TOUCH_AVERAGING_SIZE; i++)
	{
		
		while((TKEY_CTRL & bTKC_IF) == 0);	// reading bTKC_IF starts conversion - wait for flag to SET
		sum += (TKEY_DAT & ~bTKD_CHG);	// get data immediatly mask out bTKD_CHG as mentioned in data sheet
	}
    return sum/TOUCH_AVERAGING_SIZE;
}

/****************************************************************
Function Name   : touchKeyData(uint8_t channel)
Description     : Gets channel capacitance data. A smaller number means more capacitance ie. button is pressed. Touch data is 14 bits
Input           : channel to sample (eg. 1 for TIN1)
Output          : none
Return          : uint16_t touch data
****************************************************************/

uint16_t getTouchKeyData(uint8_t channel)
{
    TKEY_CTRL = TKEY_CTRL & 0b11111000 | channel + 1; // channel select in TKEY_CTRL is 1 indexed (eg. TIN1 is 2)
    while((TKEY_CTRL & bTKC_IF) == 0);	// reading bTKC_IF starts conversion and wait for flag to set
	return (TKEY_DAT & ~bTKD_CHG);	// get data immediately and mask out bTKD_CHG as mentioned in data sheet
}

/****************************************************************
Function Name   : touchKeyChannelSelect(uint8_t channel)
Description     : Selects channel to be queried and starts query. Used for interrupt mode.
Input           : none
Output          : none
Return          : none
****************************************************************/
uint8_t touchKeyChannelSelect(uint8_t channel)
{
    if(channel < 6)
    {
        // Writing to TKEY_CTRL clears interrupt flag and starts conversion
        TKEY_CTRL = (TKEY_CTRL & 0b11111000 | channel)+1;
        return 1;
    }
    return 0;
}
/****************************************************************
Function Name   : touchKey_ISR()
Description     : Interrupt Service Routine
Input           : none
Output          : none
Return          : none
****************************************************************/

// UNTESTED
// void touchKey_ISR() __interrupt (INT_NO_TKEY)
// {
//     // Get touch data and compare to no key data
// }