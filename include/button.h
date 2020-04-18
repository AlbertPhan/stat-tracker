
// Button library for C
// This library handles button and touchkey debouncing and states to determine if button was pressed or held on
// Requires: a timer library with millis()
// Author: Albert Phan
// Date: April 13 2020
// 
#pragma once

// button structure
typedef struct button
{
	uint8_t state;			// Button state
	uint16_t prev_loops;	// like prev_millis but in terms of main loop periods
	uint16_t time_loops;	// used to track retrigger and held time
	
} button;

typedef struct touchkey
{
	uint16_t nokey;		// data when no keypressed
	uint16_t data;		// current touch data value
	uint8_t state;		// Tkey states
	uint16_t prev_loops;	// like prev_millis but in terms of main loop periods
	uint16_t time_loops;	// used to track held time
	uint16_t retrigger_loops; // used to track retrigger time
} touchkey;


void update_button(button *btn, uint8_t current_button_state);
uint8_t activated(button * btn);
uint8_t deactivated(button * btn);
uint8_t read(button * btn);
uint8_t retrigger(button * btn);
uint8_t held(button * btn);
uint8_t read(button * btn);

void update_tkey(touchkey *tkey, uint16_t current_tkey_data);
uint8_t activated_tkey(touchkey * tkey);
uint8_t deactivated_tkey(touchkey * tkey);
uint8_t read_tkey(touchkey * tkey);
uint8_t retrigger_tkey(touchkey * tkey);
uint8_t held_tkey(touchkey * tkey);
uint8_t long_held_tkey(touchkey * tkey);
uint8_t read_tkey(touchkey * tkey);



