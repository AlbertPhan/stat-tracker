/* Hyper Light Drifter Stat Tracker firmware
Displays Health, Energy, and Dash points.
Manages buttons to increment points up and down

TODO:
[] change retrigger to delay + repeat (need timer)
[] cdc usb serial
[x] ADC
[] Add timer for millis()
[] charging status
[x] change to HSV for LEDs (convert functions to use HSV)
[] Touch buttons for brightness control
[x] FIX HSVtoRGB() not properly converting for saturation values above 128
[] Add configuration states using multi button presses
*/
#include <stdint.h>

#include <ch554.h>
#include <debug.h>
#include <bootloader.h>
#include <adc.h>
#include <bitbang.h>
#include <color_utils.h>

// Number of cycles that a key must be held before it's repeated
#define KEY_REPEAT_COUNT 6

// Averaging Buffer Size
#define AVERAGING_SIZE 10

// LEDs are numbered 1-46, as labeled on the PCB
#define PWR_LED 46
#define HP_BAR_START 1
#define HP_BAR_END 20
#define EN_BAR_START 21
#define EN_BAR_END 40
#define DASH_BAR_START 41
#define DASH_BAR_END 45
#define FULL_BAR_LENGTH 20
#define DASH_BAR_LENGTH 5

// ADC defines
#define BATT_MIN_ADC 163 // ADC count for 3.2V/5V * 255
#define BATT_MAX_ADC 214 // ADC count for 4.2V/5V * 255

// Colors {R,G,B}
//__code uint8_t hp_bar_color[] = {10,60,30}; 
//__code uint8_t hp_alt_bar_color[] = {10,20,100};
// __code uint8_t en_bar_color[] = {50,10,20}; 
// __code uint8_t en_alt_bar_color[] = {80,80,20}; 
// __code uint8_t dash_bar_color[] = {5,20,60}; 
// __code uint8_t dash_alt_bar_color[] = {80,10,80}; 

#define RED_HUE 0
#define GREEN_HUE 85
#define BLUE_HUE 170
#define MAX_BRIGHTNESS 100
#define DEFAULT_BRIGHTNESS 30

__xdata uint8_t brightness = DEFAULT_BRIGHTNESS; // default brightness
__xdata uint8_t saturation = 255;	// default saturation

// Colors {H,S,V} HSV values go from 0 to 255

__code HsvColor hp_bar_color = {110,255,DEFAULT_BRIGHTNESS};
__code HsvColor hp_alt_bar_color = {167,255,DEFAULT_BRIGHTNESS};
__code HsvColor en_bar_color = {235,255,DEFAULT_BRIGHTNESS}; 
__code HsvColor en_alt_bar_color = {25,255,DEFAULT_BRIGHTNESS};
__code HsvColor dash_bar_color = {185,255,DEFAULT_BRIGHTNESS};
__code HsvColor dash_alt_bar_color = {210,255,DEFAULT_BRIGHTNESS}; 

__code HsvColor off_color = {0,0,0};
__code HsvColor charged_color = {GREEN_HUE, 255 , DEFAULT_BRIGHTNESS};
__code HsvColor boot_color = {160,180,100};




#define PIN_STAT 4		// STATUS from lipo charger P3.4
SBIT(STAT, PORT3, PIN_STAT);

#define BOOT_PB 6		// BOOT Pushbutton on P3.6
SBIT(BOOT, PORT3, BOOT_PB);

#define PIN_MUL_N 6		// Negative buttons multiplex pin on P1.6
SBIT(MUL_N, PORT1, PIN_MUL_N);

#define PIN_MUL_P 7		// Positive buttons multiplex pin on P1.7
SBIT(MUL_P, PORT1, PIN_MUL_P);

#define PIN_HP_BTN 0	// HP buttons on pin P3.0
SBIT(HP_BTN, PORT3, PIN_HP_BTN);

#define PIN_EN_BTN 1	// ENergy buttons on pin P3.1
SBIT(EN_BTN, PORT3, PIN_EN_BTN);

#define PIN_DASH_BTN 3	// DASH buttons on P3.3
SBIT(DASH_BTN, PORT3, PIN_DASH_BTN);

#define PIN_VBAT 5	// VBAT on AIN2 pin P1.5


#define LED_COUNT (46)

// States
#define CHARGING 0	//
#define RUNNING 1	//
#define DEMO 2 		//

__xdata uint8_t led_data[LED_COUNT*3];
//__xdata uint32_t millis_cnt;

// Button state outputs
__xdata uint8_t HP_P_BTN_FELL;
__xdata uint8_t HP_N_BTN_FELL;
__xdata uint8_t EN_P_BTN_FELL;
__xdata uint8_t EN_N_BTN_FELL;
__xdata uint8_t DASH_P_BTN_FELL;
__xdata uint8_t DASH_N_BTN_FELL;

// Previous button states
__xdata uint8_t PREV_HP_P;
__xdata uint8_t PREV_HP_N;
__xdata uint8_t PREV_EN_P;
__xdata uint8_t PREV_EN_N;
__xdata uint8_t PREV_DASH_P;
__xdata uint8_t PREV_DASH_N;



void update_buttons()
{
	static uint8_t repeat_timer = 0;
	
	// Clear button states in case they were active previously
	HP_P_BTN_FELL = 0;
	HP_N_BTN_FELL = 0;
	EN_P_BTN_FELL = 0;
	EN_N_BTN_FELL = 0;
	DASH_P_BTN_FELL = 0;
	DASH_N_BTN_FELL = 0;
	
	MUL_P = 0;	// Drive MUL_P low to sense + buttons
	MUL_N = 1;	// Open MUL_N to let it float
	mDelayuS(5);
	
	// Check + buttons
	if (PREV_HP_P == 1 && HP_BTN == 0)
		HP_P_BTN_FELL = 1;
	
	if (PREV_EN_P == 1 && EN_BTN == 0)
		EN_P_BTN_FELL = 1;
	
	if (PREV_DASH_P == 1 && DASH_BTN == 0)
		DASH_P_BTN_FELL = 1;	
	
	// Current states become previous states for next time
	PREV_HP_P = HP_BTN;
	PREV_EN_P = EN_BTN;
	PREV_DASH_P = DASH_BTN;
	
	MUL_P = 1;	// Open MUL_P to let it float	
	MUL_N = 0;	// Drive MUL_N low to sense - buttons
	mDelayuS(5);	
	
	// Check - buttons
	if (PREV_HP_N == 1 && HP_BTN == 0)
		HP_N_BTN_FELL = 1;
	
	if (PREV_EN_N == 1 && EN_BTN == 0)
		EN_N_BTN_FELL = 1;	
	
	if (PREV_DASH_N == 1 && DASH_BTN == 0)
		DASH_N_BTN_FELL = 1;

	
	// Current states become previous states for next time
	PREV_HP_N = HP_BTN;
	PREV_EN_N = EN_BTN;
	PREV_DASH_N = DASH_BTN;
	
	MUL_N = 1; 	// float MUL_N now that we're done checking buttons
	
	// Increment the repeat timer if any button is being pressed
	if (PREV_HP_P == 0 || PREV_HP_N == 0 || PREV_EN_P == 0 || PREV_EN_N == 0 || PREV_DASH_P == 0 || PREV_DASH_N == 0)
	{
		repeat_timer++;
		
		// If a button has been held uint8_t enough
		if (repeat_timer >= KEY_REPEAT_COUNT)
		{
			repeat_timer = 0;	// Reset counter
			
			// Set all previous states to 1, so that any keys held down on next iteration think there was a falling edge
			PREV_HP_P = 1;
			PREV_HP_N = 1;
			PREV_EN_P = 1;
			PREV_EN_N = 1;
			PREV_DASH_P = 1;
			PREV_DASH_N = 1;
		}
	}
	else
	{
		repeat_timer = 0;
	}

}

// Set RGB values for a single LED
void set_led(uint8_t index, const HsvColor *hsv)
{
	RgbColor rgb;	// temp rgb struct
	//RgbColor * rgb_ptr = &rgb; 	// may not need pointer, just pass address of rgb
	HsvToRgb(hsv, &rgb);
	// rgb_ptr->r = 10; // Testing remove after
	// rgb_ptr->b = 10;
	// rgb_ptr->g = 10;
	led_data[(index-1)*3 + 1] = rgb.r;
	led_data[(index-1)*3 + 0] = rgb.g;
	led_data[(index-1)*3 + 2] = rgb.b;
}


// Fill in a bar of LEDs
inline void fill_bar(uint8_t start, uint8_t end, uint8_t value, const HsvColor *hsv)
{
	uint8_t i;
	for (i = 0; i <= end-start; i++)
	{
		if (i < value)	// LED ON
			set_led(start+i, hsv);
		else			// LED OFF
			set_led(start+i, &off_color);
	}
}

/*
Maps a value from one range to another

value: the number to map.
fromLow: the lower bound of the value’s current range.
fromHigh: the upper bound of the value’s current range.
toLow: the lower bound of the value’s target range.
toHigh: the upper bound of the value’s target range.

*/
inline uint8_t map(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void main() 
{
	uint8_t hp_value = 10;
	uint8_t energy_value = 10;
	uint8_t dash_value = 2;
	uint8_t state = RUNNING;	// Default State
	uint16_t battery_charge = 0; // from 0 - 255 and also used for summing (need 16 bits)
	uint8_t buffer_index = 0;
	__xdata uint8_t battery_adc_buffer[AVERAGING_SIZE];
	__xdata HsvColor batt_icon_hsv;	// Battery Icon Color
	__xdata HsvColor hsv = {0,255,DEFAULT_BRIGHTNESS};	// testing in DEMO mode
	
	CfgFsys();
    mDelaymS(5);
    ADCInit(1); 	// Set fast mode adc
	ADC_ChannelSelect(2);	// Set ADC channel AIN2 for VBAT
	
	
	// Set up programming pushbutton
	P3_MOD_OC |= (1<<BOOT_PB);	// Open drain	
	P3_DIR_PU |= (1<<BOOT_PB);	// pull-up enabled
	
	// Set up user buttons
	// MUX pins are open drain outputs
	P1_MOD_OC |= (1<<PIN_MUL_N) | (1<<PIN_MUL_P);
	P1_DIR_PU &= ~((1<<PIN_MUL_N) | (1<<PIN_MUL_P));
	
	// Button pins are inputs with pullups
	P3_MOD_OC |= (1<<PIN_HP_BTN) | (1<<PIN_EN_BTN) | (1<<PIN_DASH_BTN);
	P3_DIR_PU |= (1<<PIN_HP_BTN) | (1<<PIN_EN_BTN) | (1<<PIN_DASH_BTN);

	// STAT input pin pull up
	P3_MOD_OC |= (1<<PIN_STAT);
	P3_DIR_PU |= (1<<PIN_STAT);

	// VBAT as input for ADC no pull up
	P1_MOD_OC &= ~(1<<PIN_VBAT);
	P1_DIR_PU &= ~(1<<PIN_VBAT);
	
	
    bitbangSetup(); // ws2812 pin is setup here
	
	// Init button states
	HP_P_BTN_FELL = 0;
	HP_N_BTN_FELL = 0;
	EN_P_BTN_FELL = 0;
	EN_N_BTN_FELL = 0;
	DASH_P_BTN_FELL = 0;
	DASH_N_BTN_FELL = 0;
	PREV_HP_P = 1;
	PREV_HP_N = 1;
	PREV_EN_P = 1;
	PREV_EN_N = 1;
	PREV_DASH_P = 1;
	PREV_DASH_N = 1;

	// Initialize the adc buffer array with AVERAGING_SIZE good samples
	uint16_t i;
	for(i = 0; i< AVERAGING_SIZE; i++)
	{
		ADC_START = 1;	// Start ADC conversion
		while(ADC_START == 1); // Wait for conversion done
		battery_adc_buffer[i] = ADC_DATA;// add to averaging Buffer
	}

    while (1) 
	{
		update_buttons();
		
		ADC_START = 1;	// Start ADC conversion
		while(ADC_START == 1); // Wait for conversion done
		battery_adc_buffer[buffer_index] = ADC_DATA;// add to averaging Buffer
		if(buffer_index < AVERAGING_SIZE)	// if not full
		{
			buffer_index++;
		}
		else // full then reset index
		{
			buffer_index = 0;
		}

		// Start Averaging math
		battery_charge = 0; // reset battery_charge
		for(i = 0; i < AVERAGING_SIZE; i++)
		{
			// Add all values in buffer
			battery_charge += battery_adc_buffer[i];
		}
		// Divide by AVERAGING_SIZE to get average
		battery_charge = battery_charge/AVERAGING_SIZE;
		// Map ADC value to a battery charge range 0-255 and clamp to min or max values
		if(battery_charge > BATT_MAX_ADC)
		{
			battery_charge = BATT_MAX_ADC;
		}
		if(battery_charge < BATT_MIN_ADC)
		{
			battery_charge = BATT_MIN_ADC;
		}
		battery_charge = map(battery_charge, BATT_MIN_ADC,BATT_MAX_ADC,0,255);

		// map battery charge from red to green for fully charged
		batt_icon_hsv.h = map(battery_charge, 0,255,RED_HUE,GREEN_HUE);
		batt_icon_hsv.s = 255;
		batt_icon_hsv.v = brightness;

		// Check stat pin to determine if in CHARGING STATE
		if(STAT == 0)
		{
			state = DEMO;	// TESTING DEMO state change to CHARGING after
		}
		else
		{
			state = RUNNING;
		}
		
		
		switch (state)
		{
		case CHARGING:
			// TODO: Fade LED ON and OFF while charging based on battery charge
			// When Constant Voltage, we can set pwr led to green for charged.

			set_led(PWR_LED, &batt_icon_hsv);


			// fill bar based on battery charge for now while testing and make dim
			fill_bar(HP_BAR_START, DASH_BAR_END, (map(battery_charge, 0, 255, 0, FULL_BAR_LENGTH)), &hp_bar_color);

			break;
		case RUNNING:
			// Light the power LED based on battery charge
			set_led(PWR_LED, &batt_icon_hsv);
			
			// Cycle the bar values
			if (hp_value < 2*(FULL_BAR_LENGTH) && HP_P_BTN_FELL)
				hp_value++;
			else if (hp_value > 0 && HP_N_BTN_FELL)
				hp_value--;
			
			if (energy_value < 2*(FULL_BAR_LENGTH) && EN_P_BTN_FELL)
				energy_value++;
			else if (energy_value > 0 && EN_N_BTN_FELL)
				energy_value--;		
			
			if (dash_value < 2*(DASH_BAR_LENGTH) && DASH_P_BTN_FELL)
				dash_value++;
			else if (dash_value > 0 && DASH_N_BTN_FELL)
				dash_value--;		
			
			
			// Draw bars

			fill_bar(HP_BAR_START, HP_BAR_END, hp_value, &hp_bar_color);
			if (hp_value > FULL_BAR_LENGTH)
				fill_bar(HP_BAR_START, HP_BAR_START - 1 + hp_value - FULL_BAR_LENGTH, 255, &hp_alt_bar_color);
			
			fill_bar(EN_BAR_START, EN_BAR_END, energy_value, &en_bar_color);
			if (energy_value > FULL_BAR_LENGTH)
				fill_bar(EN_BAR_START, EN_BAR_START - 1 + energy_value - FULL_BAR_LENGTH, 255, &en_alt_bar_color);
			
			fill_bar(DASH_BAR_START, DASH_BAR_END, dash_value, &dash_bar_color);
			if (dash_value > DASH_BAR_LENGTH)
				fill_bar(DASH_BAR_START, DASH_BAR_START - 1 + dash_value - DASH_BAR_LENGTH, 255, &dash_alt_bar_color);		
			break;
		case DEMO:
			if(HP_P_BTN_FELL)
			{
				// Increment saturation if less than 255
				if (hsv.s < 251)
				{
					hsv.s += 5;
				}
				
			}
			if(HP_N_BTN_FELL)
			{
				// Decrement saturation if more than 0
				if(hsv.s > 4)
				{
					hsv.s -= 5;
				}
			}
			if(EN_P_BTN_FELL)
			{
				// Increment value if less than 255
				if (hsv.v < 251)
				{
					hsv.v += 5;
				}
				
			}
			if(EN_N_BTN_FELL)
			{
				// Decrement value if more than 0
				if(hsv.v > 4)
				{
					hsv.v -= 5;
				}
			}

			for(i = 0; i < LED_COUNT; i++)
			{
				// go through leds and change hsv for each
				hsv.h = 5*i;
				set_led(i,&hsv);
			}
			// Set battery icon led
			set_led(PWR_LED, &batt_icon_hsv);
			break;
		default:
			break;
		}
		
        // Display led_data
        bitbangWs2812(LED_COUNT, led_data);
        mDelaymS(20); // Mostly for Button polling frequency
		
		if (BOOT == 1)
			break; 
    }
	
	// Turn off WS2812's before starting bootloader
	for (i = 0; i < LED_COUNT*3; i++)
		led_data[i] = 0;
	
	// Light the power LED to indicate boot mode
	set_led(PWR_LED, &boot_color);
	
	bitbangWs2812(LED_COUNT, led_data);
	EA = 0;	// Disable interrupts
	mDelaymS(100);
	
	bootloader();
	while(1);
	
}
