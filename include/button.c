/*
efficient debouncing button and touchkey library for embedded CH552/CH554 MCU
Date 4/20/2020
Albert Phan

TODO:
[] update button to do long held as well like tkey button is done
[] find out if we can use only 8 bits for some of the timings


This code gives certain button states such as activated/deactivated/held/long_held/retrigger/de_retrigger. A timer library with millis() function is required.
Most notable feature is retrigger which will return true for every RETRIGGER_TIME after held time has passed.
Time counters are shortened to main loop periods instead of millis() to save on bytes . Be sure that your times can fit within a single byte (max 255) where necessary .

Usage button example:
 - Create Button struct with all values initialized to 0;
 - Update each button with the button value (1 for button ON, 0 for button off) per main loop
 - Use functions to return button states and act upon them.

Modified code from https://github.com/thomasfredericks/Bounce2
*/

#include <stdint.h>
#include <timer.h> // LOOP_PERIOD_MS and millis()
#include "button.h"
#include <touch.h> 

// Times in ms
#define DEBOUNCE_TIME 20	// debounce time in ms
#define HELD_TIME 300		// time before retriggering happens 
#define RETRIGGER_TIME 100	// time between button retriggers when held down
#define TKEY_HELD_TIME 500
#define TKEY_RETRIGGER_TIME 100
#define LONG_HELD_TIME 2000 // Testing to 2.0s for now - 5.0s for default

#define DEBOUNCED_STATE (1<<0)
#define UNSTABLE_STATE (1<<1)	// Used with a different type of debouncing that's currently not implemented
#define STATE_CHANGED (1<<2)
#define STATE_ACTIVATED (1<<3)
#define STATE_DEACTIVATED (1<<4)
#define STATE_HELD_ON (1<<5)
#define STATE_RETRIGGER (1<<6)
#define STATE_LONG_HELD_ON (1<<7)




// Pass button read data 1 is pressed 0 is not (logic is active high)
// Pass current_button_state as if button was active high
// eg - for active low button - use update_button(&button, !active_low_button)
// Call once per button per main loop period
void update_button(button * btn, uint8_t current_button_state)
{
	// update the button
	uint16_t current_btn_millis = millis();	// truncate millis() to 16 bit
	// Update the debounced state and if state changed 
	// Reset states if previously changed
	btn->state &= ~STATE_CHANGED;
	btn->state &= ~STATE_ACTIVATED;
	btn->state &= ~STATE_DEACTIVATED;
	btn->state &= ~STATE_RETRIGGER;
	
	
	// UsbCdc_puts("current_btn_millis:");	// TESTING
	// UsbCdc_puti32(current_btn_millis);
	// UsbCdc_puts(" btn->prev_loops:");
	// UsbCdc_puti32(btn->prev_loops);
	// UsbCdc_puts(" diff:");
	// UsbCdc_puti32(current_btn_millis - btn->prev_loops * LOOP_PERIOD_MS);
	// // UsbCdc_puts(" loop:");
	// // UsbCdc_puti32(timing_millis);
	// UsbCdc_puts("\r\n");
	
	// Ignore everything if we are locked out
	if(current_btn_millis - btn->prev_loops * LOOP_PERIOD_MS >= DEBOUNCE_TIME)
	{
		if ((btn->state & DEBOUNCED_STATE )!= current_button_state)
		{
			btn->prev_loops = current_btn_millis/LOOP_PERIOD_MS;
			btn->state ^= DEBOUNCED_STATE;
			btn->state |= STATE_CHANGED;
		}
	}

	// Update if retrigger is active or not

	// if State HELD_ON
	if(btn->state & STATE_HELD_ON)
	{
		// check if longer than last retrigger by RETRIGGER_TIME
		if((current_btn_millis - btn->time_loops*LOOP_PERIOD_MS )>= RETRIGGER_TIME)
		{
			btn->time_loops = current_btn_millis/LOOP_PERIOD_MS;
			// set retrigger state
			btn->state |= STATE_RETRIGGER;
		}
	}
	// if button is on and held for more than HELD_TIME
	else if ((btn->state & DEBOUNCED_STATE) && btn->time_loops *LOOP_PERIOD_MS >= HELD_TIME)
	{
		// change state to held on
		btn->state |= STATE_HELD_ON;
		btn->time_loops = 0; // Set to 0 so we can reuse this variable in STATE_HELD_ON
	}
	// if button state hasnt changed and is still on, increment time_loops
	else if (!(btn->state & STATE_CHANGED) && btn->state & DEBOUNCED_STATE)
	{
		btn->time_loops++;
	}
	// if button state changed
	if(btn->state & STATE_CHANGED)
	{
		// reset time_loops and other states
		btn->time_loops = 0;
		btn->state &= ~STATE_HELD_ON;
		if(current_button_state)
			{
				btn->state |= STATE_ACTIVATED;
			}
		else
			btn->state |= STATE_DEACTIVATED;
	}
	
}

void update_tkey(touchkey *tkey, uint16_t current_tkey_data)
{
	// Update the button
	uint16_t current_tkey_millis = millis();	// truncate millis to 16 bit, we dont need to track long times
	// Update the debounced state and if state changed 
	// Reset states if previously changed
	tkey->data = current_tkey_data;

	if(tkey->time_loops == 0)	// reset held long on state
	{
		tkey->state &= ~STATE_LONG_HELD_ON;
	}
	tkey->state &= ~STATE_CHANGED;
	tkey->state &= ~STATE_ACTIVATED;
	tkey->state &= ~STATE_DEACTIVATED;
	tkey->state &= ~STATE_RETRIGGER;
	
	
	// UsbCdc_puts("current_tkey_millis:");	// TESTING
	// UsbCdc_puti32(current_tkey_millis);
	// UsbCdc_puts(" tkey->prev_loops:");
	// UsbCdc_puti32(tkey->prev_loops);
	// UsbCdc_puts(" diff:");
	// UsbCdc_puti32(current_tkey_millis - tkey->prev_loops * LOOP_PERIOD_MS);
	// // UsbCdc_puts(" loop:");
	// // UsbCdc_puti32(timing_millis);
	// UsbCdc_puts("\r\n");
	
	// Ignore everything if we are locked out
	if(current_tkey_millis - tkey->prev_loops * LOOP_PERIOD_MS >= DEBOUNCE_TIME)
	{
		
		// Compare data to when touch key was not pressed and determine if touch key was pressed
		if ((tkey->state & DEBOUNCED_STATE) != (tkey->data < tkey->nokey - TOUCH_HYSTERESIS))
		{
			tkey->prev_loops = current_tkey_millis/LOOP_PERIOD_MS;
			tkey->state ^= DEBOUNCED_STATE;
			tkey->state |= STATE_CHANGED;
		}
	}

	// Update if retrigger is active or not

	// if State HELD_ON
	if(tkey->state & STATE_HELD_ON)
	{
		// check if longer than last retrigger by RETRIGGER_TIME
		if((current_tkey_millis - tkey->retrigger_loops*LOOP_PERIOD_MS )>= TKEY_RETRIGGER_TIME)
		{
			tkey->retrigger_loops = current_tkey_millis/LOOP_PERIOD_MS;
			// set retrigger state
			tkey->state |= STATE_RETRIGGER;
		}
	}
	// else if button is on and held for more than TKEY_HELD_TIME
	else if ((tkey->state & DEBOUNCED_STATE) && tkey->time_loops *LOOP_PERIOD_MS >= TKEY_HELD_TIME)
	{
		// change state to held on
		tkey->state |= STATE_HELD_ON;
		tkey->retrigger_loops = 0;
	}

	// if button is on and held for more than LONG_HELD_TIME
	if((tkey->state & DEBOUNCED_STATE) && tkey->time_loops *LOOP_PERIOD_MS >= LONG_HELD_TIME)
	{
		tkey->state |= STATE_LONG_HELD_ON;
	}
	// else if button state hasnt changed and is still on, increment time_loops
	else if (!(tkey->state & STATE_CHANGED) && tkey->state & DEBOUNCED_STATE)
	{
		tkey->time_loops++;
	}
	

	// if button state changed
	if(tkey->state & STATE_CHANGED)
	{
		// reset time_loops and other states
		tkey->time_loops = 0;
		tkey->state &= ~STATE_HELD_ON;	
		// tkey on?
		if(tkey->data < tkey->nokey - TOUCH_HYSTERESIS)
			{
				
				tkey->state |= STATE_ACTIVATED;
			}
		else	// tkey off
			{
			if(tkey->state & STATE_LONG_HELD_ON) // if previously held long
			{
				tkey->state |= STATE_LONG_HELD_ON;	// Leave state long held on for one cycle to so it can be checked that it was held on and then deactivated
			}
			tkey->state |= STATE_DEACTIVATED;
			}
	}
	
}

uint8_t activated(button * btn)
{
   return btn->state & STATE_ACTIVATED;
}

uint8_t deactivated(button * btn)
{
	return btn->state & STATE_DEACTIVATED;
}

uint8_t read(button * btn)
{
	return btn->state & DEBOUNCED_STATE;
}

// Returns true every RETRIGGER_TIME after held time has passed
uint8_t retrigger(button * btn)
{
	return btn->state & STATE_RETRIGGER;
}

// Returns true every RETRIGGER_TIME after held time has passed and also when button was let go
uint8_t de_retrigger(button * btn)
{
	return btn->state & (STATE_RETRIGGER | STATE_DEACTIVATED);
}

uint8_t held(button * btn)
{
	return btn->state & STATE_HELD_ON;
}

uint8_t activated_tkey(touchkey * tkey)
{
   return tkey->state & STATE_ACTIVATED;
}

uint8_t deactivated_tkey(touchkey * tkey)
{
	return tkey->state & STATE_DEACTIVATED;
}

uint8_t read_tkey(touchkey * tkey)
{
	return tkey->state & DEBOUNCED_STATE;
}

// Returns true when button first activated and repeats periodically after held time has passed
uint8_t retrigger_tkey(touchkey * tkey)
{
	return tkey->state & STATE_RETRIGGER;
}
uint8_t de_retrigger_tkey(touchkey * tkey)
{
	return tkey->state & (STATE_RETRIGGER | STATE_DEACTIVATED);
}

uint8_t held_tkey(touchkey * tkey)
{
	return tkey->state & STATE_HELD_ON;
}

uint8_t long_held_tkey(touchkey * tkey)
{
	return tkey->state & STATE_LONG_HELD_ON;
}