

#include <stdint.h>
#include <timer.h>
#include "button.h"

#define DEBOUNCE_TIME 20	// debounce time in ms
#define HELD_TIME 300		// time before retriggering happens
#define RETRIGGER_TIME 100	// time between button retriggers when held down

#define DEBOUNCED_STATE (1<<0)
#define UNSTABLE_STATE (1<<1)
#define STATE_CHANGED (1<<2)
#define STATE_ACTIVATED (1<<3)
#define STATE_DEACTIVATED (1<<4)
#define STATE_HELD_ON (1<<5)
#define STATE_RETRIGGER (1<<6)




// Pass button read data 1 is pressed 0 is not (logic is active high)
// Pass current_button_state as if button was active high
// eg - for active low button - use update_button(&button, !active_low_button)
// Call once per button per main loop period
void update_button(button * btn, uint8_t current_button_state)
{
	// update the button
	uint32_t current_btn_millis = millis();
	// Update the debounced state and if state changed 
	// Reset states if previously changed
	btn->state &= ~STATE_CHANGED;
	btn->state &= ~STATE_ACTIVATED;
	btn->state &= ~STATE_DEACTIVATED;
	btn->state &= ~STATE_RETRIGGER;
	// Ignore everything if we are locked out
	
	// UsbCdc_puts("current_btn_millis:");	// TESTING
	// UsbCdc_puti32(current_btn_millis);
	// UsbCdc_puts(" btn->prev_loops:");
	// UsbCdc_puti32(btn->prev_loops);
	// UsbCdc_puts(" diff:");
	// UsbCdc_puti32(current_btn_millis - btn->prev_loops * LOOP_PERIOD_MS);
	// // UsbCdc_puts(" loop:");
	// // UsbCdc_puti32(timing_millis);
	// UsbCdc_puts("\r\n");
	

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
		// FIX retrigger not toggling
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
				btn->state |= STATE_RETRIGGER; // activates for initial press
			}
		else
			btn->state |= STATE_DEACTIVATED;
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

// Returns true when button first activated and repeats periodically after held time has passed
uint8_t retrigger(button * btn)
{
	return btn->state & STATE_RETRIGGER;
}

uint8_t held(button * btn)
{
	return btn->state & STATE_HELD_ON;
}