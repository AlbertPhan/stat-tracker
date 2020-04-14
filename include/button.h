
// Button library for C
// This library handles button debouncing and states to determine if button was pressed or held on
// Requires: a timer library with millis()
// Author: Albert Phan
// Date: April 13 2020
// 
#pragma once

// button structure
typedef struct button
{
	uint8_t state;	// Button state
	uint32_t prev_loops;	// like prev_millis but in terms of main loop periods
	uint32_t time_loops;	// used to track retrigger and held time
	
} button;

void update_button(button *btn, uint8_t current_button_state);
uint8_t activated(button * btn);
uint8_t deactivated(button * btn);
uint8_t read(button * btn);
uint8_t retrigger(button * btn);
uint8_t held(button * btn);
uint8_t read(button * btn);



