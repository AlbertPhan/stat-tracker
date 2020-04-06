#pragma once


#define TOUCH_AVERAGING_SIZE 50 // number of samples to take for nokey value
#define TOUCH_HYSTERESIS 200	// Hyseresis amount 

#define enableTouchKeyInterrupt() (IE_TKEY = 1)    // Interrupt enable - turn on after getting no key data
#define touchKeyOFF() {TKEY_CTRL &= 0b11111000;}  // Touch Module off
#define touchKeyON_NoChannel() {TKEY_CTRL = TKEY_CTRL & 0x11111000 | 0b111;}  // Touch Module ON but no channel selected
#define touchKeyQueryCyl1ms() {TKEY_CTRL &= ~bTKC_2MS;} // Set 0-1ms period selection
#define touchKeyQueryCyl2ms() {TKEY_CTRL |= bTKC_2MS;} // Set 1-2 ms period selection

/****************************************************************
Function Name        : getNokeyData()
Description : Samples TOUCH_AVERAGING_SIZE times and averages to get a no key pressed baseline capacitance value. 
Input       : channel to sample (eg. 1 for TIN1)
Output      : none
Return      : uint16_t nokey pressed data
****************************************************************/
uint16_t getNokeyData(uint8_t channel);

/****************************************************************
Function Name   : touchKeyData(uint8_t channel)
Description     : Gets channel capacitance data. A smaller number means more capacitance ie. button is pressed. Touch data is 14 bits
Input           : channel to sample (eg. 1 for TIN1)
Output          : none
Return          : uint16_t touch data
****************************************************************/

uint16_t getTouchKeyData(uint8_t channel);