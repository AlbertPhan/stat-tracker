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
[] Fix led showing garbage data on power on
[] Fix battery icon showing wrong hue
*/
#include <stdint.h>

#include <ch554.h>
#include <debug.h>
#include <bootloader.h>
#include <adc.h>
#include <bitbang.h>
#include <color_utils.h>
#include <timer.h>
#include <touch.h>

// Number of cycles that a key must be held before it's repeated
#define KEY_REPEAT_COUNT 6

// Averaging ADC Buffer Size
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
#define MAX_BRIGHTNESS 60
#define DEFAULT_BRIGHTNESS 30
#define DEFAULT_SATURATION 255
#define BRIGHTNESS_STEP 5

__xdata uint8_t brightness = DEFAULT_BRIGHTNESS; // default brightness
__xdata uint8_t saturation = DEFAULT_SATURATION;	// default saturation

// Colors {H,S,V} HSV values go from 0 to 255

__xdata HsvColor hp_bar_color = {110,DEFAULT_SATURATION,DEFAULT_BRIGHTNESS};
__xdata HsvColor hp_alt_bar_color = {167,DEFAULT_SATURATION,DEFAULT_BRIGHTNESS};
__xdata HsvColor en_bar_color = {235,DEFAULT_SATURATION,DEFAULT_BRIGHTNESS}; 
__xdata HsvColor en_alt_bar_color = {35,DEFAULT_SATURATION,DEFAULT_BRIGHTNESS};
__xdata HsvColor dash_bar_color = {185,DEFAULT_SATURATION,DEFAULT_BRIGHTNESS};
__xdata HsvColor dash_alt_bar_color = {210,DEFAULT_SATURATION,DEFAULT_BRIGHTNESS};
__xdata HsvColor off_color = {0,0,0};
__xdata HsvColor charged_color = {GREEN_HUE,DEFAULT_SATURATION, DEFAULT_BRIGHTNESS};
__xdata HsvColor boot_color = {160,180,DEFAULT_BRIGHTNESS};




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
#define PIN_TIN2 4  // Touchkey Increase Brightness Button P1.4
#define PIN_TIN1 1	// Touchkey Decrease Brightness Button  P1.1
#define BRIGHTNESS_UP_TOUCHKEY_NUM 2 // Touch key number 
#define BRIGHTNESS_DOWN_TOUCHKEY_NUM 1 // Touch key number


#define LED_COUNT (46)

// States
#define MAX_STATES 5
#define CHARGING 1	//
#define RUNNING 2	//
#define DEMO 3 		//
#define TOUCHKEY1 4 //
#define TOUCHKEY2 5 // 

__xdata uint8_t led_data[LED_COUNT*3];
//__xdata uint32_t millis_cnt;

// button structure
typedef struct button
{
	uint8_t PREV;		// previous button state
	uint8_t ACTIVATED;	// transition to active state
	uint8_t DEACTIVATED;	// transition to deactivated state
} button;

typedef struct touchkey
{
	uint16_t NOKEY;		// data when no keypressed
	uint16_t DATA;		// current touch data value
	uint8_t PREV;		// previous state 
	uint8_t ACTIVATED;	// transition to active state
	uint8_t DEACTIVATED;	// transition to deactivated state
} touchkey;

// button structs
__xdata button HP_P_BTN = {0,0,0};
__xdata button HP_N_BTN = {0,0,0};
__xdata button EN_P_BTN = {0,0,0};
__xdata button EN_N_BTN = {0,0,0};
__xdata button DASH_P_BTN = {0,0,0};
__xdata button DASH_N_BTN = {0,0,0};

__xdata touchkey BRIGHTNESS_UP_TKEY = {0,0,0,0,0};
__xdata touchkey BRIGHTNESS_DOWN_TKEY = {0,0,0,0,0};



void update_buttons()
{
	static uint8_t repeat_timer = 0;
	
	// Clear button states in case they were active previously
	HP_P_BTN.ACTIVATED = 0;
	HP_N_BTN.ACTIVATED = 0;
	EN_P_BTN.ACTIVATED = 0;
	EN_N_BTN.ACTIVATED = 0;
	DASH_P_BTN.ACTIVATED = 0;
	DASH_N_BTN.ACTIVATED = 0;
	BRIGHTNESS_UP_TKEY.ACTIVATED = 0;
	BRIGHTNESS_DOWN_TKEY.ACTIVATED = 0;
	
	MUL_P = 0;	// Drive MUL_P low to sense + buttons
	MUL_N = 1;	// Open MUL_N to let it float
	mDelayuS(5); // Give time to switch pins
	
	// Check + buttons
	if (HP_P_BTN.PREV == 1 && HP_BTN == 0)
		HP_P_BTN.ACTIVATED = 1;
	
	if (EN_P_BTN.PREV == 1 && EN_BTN == 0)
		EN_P_BTN.ACTIVATED = 1;
	
	if (DASH_P_BTN.PREV == 1 && DASH_BTN == 0)
		DASH_P_BTN.ACTIVATED = 1;	
	
	// Current states become previous states for next time
	HP_P_BTN.PREV = HP_BTN;
	EN_P_BTN.PREV = EN_BTN;
	DASH_P_BTN.PREV = DASH_BTN;
	
	MUL_P = 1;	// Open MUL_P to let it float	
	MUL_N = 0;	// Drive MUL_N low to sense - buttons
	mDelayuS(5);	// Give time to switch pins
	
	// Check - buttons
	if (HP_N_BTN.PREV == 1 && HP_BTN == 0)
		HP_N_BTN.ACTIVATED = 1;
	
	if (EN_N_BTN.PREV == 1 && EN_BTN == 0)
		EN_N_BTN.ACTIVATED = 1;	
	
	if (DASH_N_BTN.PREV == 1 && DASH_BTN == 0)
		DASH_N_BTN.ACTIVATED = 1;

	
	// Current states become previous states for next time
	HP_N_BTN.PREV = HP_BTN;
	EN_N_BTN.PREV = EN_BTN;
	DASH_N_BTN.PREV = DASH_BTN;
	
	MUL_N = 1; 	// float MUL_N now that we're done checking buttons
	
	// Increment the repeat timer if any button is being pressed
	if (HP_P_BTN.PREV == 0 || HP_N_BTN.PREV == 0 || EN_P_BTN.PREV == 0 || EN_N_BTN.PREV == 0 || DASH_P_BTN.PREV == 0 || DASH_N_BTN.PREV == 0)
	{
		repeat_timer++;
		
		// If a button has been held uint8_t enough
		if (repeat_timer >= KEY_REPEAT_COUNT)
		{
			repeat_timer = 0;	// Reset counter
			
			// Set all previous states to 1, so that any keys held down on next iteration think there was a falling edge
			HP_P_BTN.PREV = 1;
			HP_N_BTN.PREV = 1;
			EN_P_BTN.PREV = 1;
			EN_N_BTN.PREV = 1;
			DASH_P_BTN.PREV = 1;
			DASH_N_BTN.PREV = 1;
		}
	}
	else
	{
		repeat_timer = 0;
	}

	// Touchkeys

	// Start brightness up tkey query
	BRIGHTNESS_UP_TKEY.DATA = getTouchKeyData(BRIGHTNESS_UP_TOUCHKEY_NUM);
	// Data is inversely proportional to the capacitance. When touch button is pressed data is smaller than when not pressed.

	// Compare data to when key was no pressed
	if(BRIGHTNESS_UP_TKEY.PREV == 0 && BRIGHTNESS_UP_TKEY.DATA < BRIGHTNESS_UP_TKEY.NOKEY-TOUCH_HYSTERESIS) // button is pressed
	{
		BRIGHTNESS_UP_TKEY.ACTIVATED = 1;
	}
	// Current state becomes previous state
	BRIGHTNESS_UP_TKEY.PREV = BRIGHTNESS_UP_TKEY.DATA < BRIGHTNESS_UP_TKEY.NOKEY -TOUCH_HYSTERESIS;

	// Start brightness down tkey query
	BRIGHTNESS_DOWN_TKEY.DATA = getTouchKeyData(BRIGHTNESS_DOWN_TOUCHKEY_NUM);
	if(BRIGHTNESS_DOWN_TKEY.PREV == 0 && BRIGHTNESS_DOWN_TKEY.DATA < BRIGHTNESS_DOWN_TKEY.NOKEY-TOUCH_HYSTERESIS) // button is pressed
	{
		BRIGHTNESS_DOWN_TKEY.ACTIVATED = 1;
	}
	BRIGHTNESS_DOWN_TKEY.PREV = BRIGHTNESS_DOWN_TKEY.DATA < BRIGHTNESS_DOWN_TKEY.NOKEY -TOUCH_HYSTERESIS;

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
inline uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
	return (uint32_t)((x - in_min) * ((out_max - out_min) / (in_max - in_min)) + out_min);
}


inline void set_battery_icon_led()
{
	// Do stuff while CHARGING
	if(CHARGING)
	{
		// Fade LED on and off - colour is charge
		;
	}
	// Do stuff while RUNNING
}


void main() 
{
	uint8_t hp_value = 10;
	uint8_t energy_value = 10;
	uint8_t dash_value = 2;
	uint8_t state = RUNNING;	// Default State
	uint16_t battery_charge = 0; // from 0 - 255 and also used for summing (need 16 bits)
	uint8_t buffer_index = 0;
	uint16_t i;	// General Purpose index
	__xdata uint8_t battery_adc_buffer[AVERAGING_SIZE];
	__xdata HsvColor batt_icon_hsv;	// Battery Icon Color 
	batt_icon_hsv.s = DEFAULT_SATURATION;
	batt_icon_hsv.v = DEFAULT_BRIGHTNESS;
	__xdata HsvColor hsv = {0,DEFAULT_SATURATION,DEFAULT_BRIGHTNESS};	// testing in DEMO mode
	// Init button states
	HP_P_BTN.ACTIVATED = 0;
	HP_N_BTN.ACTIVATED = 0;
	EN_P_BTN.ACTIVATED = 0;
	EN_N_BTN.ACTIVATED = 0;
	DASH_P_BTN.ACTIVATED = 0;
	DASH_N_BTN.ACTIVATED = 0;
	BRIGHTNESS_UP_TKEY.ACTIVATED = 0;
	BRIGHTNESS_DOWN_TKEY.ACTIVATED = 0;
	HP_P_BTN.PREV = 1;
	HP_N_BTN.PREV = 1;
	EN_P_BTN.PREV = 1;
	EN_N_BTN.PREV = 1;
	DASH_P_BTN.PREV = 1;
	DASH_N_BTN.PREV = 1;
	BRIGHTNESS_UP_TKEY.PREV = 0;		// active high
	BRIGHTNESS_DOWN_TKEY.PREV = 0;	
	
	CfgFsys();
	bitbangSetup(); // ws2812 pin is setup here
    bitbangWs2812(LED_COUNT, led_data); // Blank out all leds to get rid of garbage data (not working)
	mDelaymS(5);

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

	// Touchkey inputs as floating input mode no pull up
	P1_DIR_PU &= ~(1<<PIN_TIN1 | 1 << PIN_TIN2);	// P1.4 TIN 2 and P1.1 TIN1

	//enable_timer0_interrupt(); // off for now while testing

    ADCInit(1); 	// Set fast mode adc
	ADC_ChannelSelect(2);	// Set ADC channel AIN2 for VBAT

	
	// Touch key setup 
	touchKeyQueryCyl2ms();	// set period selection
	// Get no touchkey pressed data
	BRIGHTNESS_UP_TKEY.NOKEY = getNokeyData(BRIGHTNESS_UP_TOUCHKEY_NUM);	// set no key threshold
	BRIGHTNESS_DOWN_TKEY.NOKEY = getNokeyData(BRIGHTNESS_DOWN_TOUCHKEY_NUM);	// set no key threshold

	// Initialize the adc buffer array with AVERAGING_SIZE good samples
	for(i = 0; i< AVERAGING_SIZE; i++)
	{
		ADC_START = 1;	// Start ADC conversion
		while(ADC_START == 1); // Wait for conversion done
		battery_adc_buffer[i] = ADC_DATA;// add to averaging Buffer
	}

	
    while (1) 
	{
		update_buttons();
		
		// Start battery voltage adc stuff
		ADC_START = 1;	// Start ADC conversion
		while(ADC_START == 1); // Wait for conversion done
		battery_adc_buffer[buffer_index] = ADC_DATA;// add to averaging Buffer
		// if(buffer_index < AVERAGING_SIZE)	// if not full
		// {
		// 	buffer_index++;
		// }
		// else // full then reset index
		// {
		// 	buffer_index = 0;
		// }

		// // Start Averaging math
		// battery_charge = 0; // reset battery_charge
		// for(i = 0; i < AVERAGING_SIZE; i++)
		// {
		// 	// Add all values in buffer
		// 	battery_charge += battery_adc_buffer[i];
		// }
		// // Divide by AVERAGING_SIZE to get average
		// battery_charge = battery_charge/AVERAGING_SIZE;
		// Map ADC value to a battery charge range 0-255 and clamp to min or max values

		battery_charge = ADC_DATA; 	// TESTING not averaging
		if(battery_charge > BATT_MAX_ADC)
		{
			battery_charge = BATT_MAX_ADC;
		}
		if(battery_charge < BATT_MIN_ADC)
		{
			battery_charge = BATT_MIN_ADC;
		}
		battery_charge = map(battery_charge,BATT_MIN_ADC,BATT_MAX_ADC,0,255);

		// map battery charge from red to green for fully charged
		batt_icon_hsv.h = map(battery_charge,0,255,RED_HUE,GREEN_HUE);
		
		// Check stat pin to determine if in CHARGING STATE
		// if(STAT == 0)
		// {
		// 	state = CHARGING;	// TESTING DEMO state change to CHARGING after
		// }
		// else
		// {
		// 	state = RUNNING;
		// }

		// Brightness control
		if(BRIGHTNESS_UP_TKEY.ACTIVATED)
		{
			// Increment value if <= MAX_BRIGHTNESS - BRIGHTNESS_STEP
				if (brightness <= MAX_BRIGHTNESS - BRIGHTNESS_STEP)
				{
					brightness += BRIGHTNESS_STEP;
				}
		}
		if(BRIGHTNESS_DOWN_TKEY.ACTIVATED)
		{
			// Decrement value if >= 0 + BRIGHTNESS_STEP
			if( brightness > BRIGHTNESS_STEP)
			{
				brightness -= BRIGHTNESS_STEP;
			}
		}
		// change all colors brightness
		hp_bar_color.v = brightness; 
		hp_alt_bar_color.v = brightness;
		en_bar_color.v =  brightness;
		en_alt_bar_color.v = brightness;
		dash_bar_color.v = brightness;
		dash_alt_bar_color.v = brightness; 
		charged_color.v = brightness;
		boot_color.v = brightness;
		batt_icon_hsv.v = brightness;

		// TESTING using dash buttons to switch between states
		if(DASH_P_BTN.ACTIVATED)
		{
			if(state < MAX_STATES)
			{
				state++;
			}
		}
		if(DASH_N_BTN.ACTIVATED)
		{
			if(state > CHARGING)
			{
				state--;
			}
		}

		
		switch (state)
		{
		case CHARGING:
			// TODO: Fade LED ON and OFF while charging based on battery charge
			// When Constant Voltage, we can set pwr led to green for charged.



			// fill bar based on battery charge for now while testing and make dim
			fill_bar(HP_BAR_START, DASH_BAR_END, (map(battery_charge, 0, 255, 0, FULL_BAR_LENGTH)), &hp_bar_color);
			set_led(PWR_LED, &batt_icon_hsv);
			break;
		case RUNNING:
			
			
			// Cycle the bar values
			if (hp_value < 2*(FULL_BAR_LENGTH) && HP_P_BTN.ACTIVATED)
				hp_value++;
			else if (hp_value > 0 && HP_N_BTN.ACTIVATED)
				hp_value--;
			
			if (energy_value < 2*(FULL_BAR_LENGTH) && EN_P_BTN.ACTIVATED)
				energy_value++;
			else if (energy_value > 0 && EN_N_BTN.ACTIVATED)
				energy_value--;		
			
			if (dash_value < 2*(DASH_BAR_LENGTH) && DASH_P_BTN.ACTIVATED)
				dash_value++;
			else if (dash_value > 0 && DASH_N_BTN.ACTIVATED)
				dash_value--;		
			
			
			// Draw bars

			fill_bar(HP_BAR_START, HP_BAR_END, hp_value, &hp_bar_color);
			if (hp_value > FULL_BAR_LENGTH)
				fill_bar(HP_BAR_START, HP_BAR_START - 1 + hp_value - FULL_BAR_LENGTH, DEFAULT_SATURATION, &hp_alt_bar_color);
			
			fill_bar(EN_BAR_START, EN_BAR_END, energy_value, &en_bar_color);
			if (energy_value > FULL_BAR_LENGTH)
				fill_bar(EN_BAR_START, EN_BAR_START - 1 + energy_value - FULL_BAR_LENGTH, DEFAULT_SATURATION, &en_alt_bar_color);
			
			fill_bar(DASH_BAR_START, DASH_BAR_END, dash_value, &dash_bar_color);
			if (dash_value > DASH_BAR_LENGTH)
				fill_bar(DASH_BAR_START, DASH_BAR_START - 1 + dash_value - DASH_BAR_LENGTH, DEFAULT_SATURATION, &dash_alt_bar_color);

			// Light the power LED based on battery charge
			set_led(PWR_LED, &batt_icon_hsv);		
			break;
		case DEMO:
			// colour test pallette
			if(HP_P_BTN.ACTIVATED)
			{
				// Increment saturation if less than 255
				if (hsv.s < 251)
				{
					hsv.s += 5;
				}
				
			}
			if(HP_N_BTN.ACTIVATED)
			{
				// Decrement saturation if more than 0
				if(hsv.s > 4)
				{
					hsv.s -= 5;
				}
			}
			if(EN_P_BTN.ACTIVATED)
			{
				// Increment value if less than 255
				if (hsv.v < 251)
				{
					hsv.v += 5;
				}
				
			}
			if(EN_N_BTN.ACTIVATED)
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
		case TOUCHKEY1: // Debug data for TIN1 BRIGHTNESS_DOWN_TOUCHKEY_NUM
			fill_bar(HP_BAR_START,PWR_LED,PWR_LED,&off_color);
			// Display nokey data
			fill_bar(HP_BAR_START,HP_BAR_END, map(BRIGHTNESS_DOWN_TKEY.NOKEY-TOUCH_HYSTERESIS,0,2^14, 0,FULL_BAR_LENGTH), &hp_bar_color);
			// Display capacitance data
			fill_bar(EN_BAR_START,EN_BAR_END, map(BRIGHTNESS_DOWN_TKEY.DATA,0,2^14, 0,FULL_BAR_LENGTH), &en_bar_color);

			set_led(DASH_BAR_START, &dash_bar_color);

			// Turn on dash leds if touch data is less than nokey threshold
			if(BRIGHTNESS_DOWN_TKEY.DATA < BRIGHTNESS_DOWN_TKEY.NOKEY - TOUCH_HYSTERESIS)
			{
				set_led(DASH_BAR_END-1, &hp_bar_color);
			}
			if(BRIGHTNESS_UP_TKEY.DATA < BRIGHTNESS_UP_TKEY.NOKEY - TOUCH_HYSTERESIS)
			{
				set_led(DASH_BAR_END, &hp_bar_color);
			}
			// Set battery icon led
			set_led(PWR_LED, &batt_icon_hsv);
			break;
		case TOUCHKEY2:	// Debug data for TIN2 BRIGHTNESS_UP_TOUCHKEY_NUM
			fill_bar(HP_BAR_START,PWR_LED,PWR_LED,&off_color);
			// Display capacitance data values on hp and en bars
			fill_bar(HP_BAR_START,HP_BAR_END, map(BRIGHTNESS_UP_TKEY.NOKEY-TOUCH_HYSTERESIS,0,2^14, 0,FULL_BAR_LENGTH), &hp_bar_color);
			// Display capacitance data
			fill_bar(EN_BAR_START,EN_BAR_END, map(BRIGHTNESS_UP_TKEY.DATA,0,2^14, 0,FULL_BAR_LENGTH), &en_bar_color);

			fill_bar(DASH_BAR_START,DASH_BAR_START + 1,2, &dash_bar_color);	// turn on 2 dash leds to show state

			// Turn on dash leds if touch data is less than nokey threshold
			if(BRIGHTNESS_DOWN_TKEY.DATA < BRIGHTNESS_DOWN_TKEY.NOKEY - TOUCH_HYSTERESIS)
			{
				set_led(DASH_BAR_END-1, &hp_bar_color);
			}
			if(BRIGHTNESS_UP_TKEY.DATA < BRIGHTNESS_UP_TKEY.NOKEY - TOUCH_HYSTERESIS)
			{
				set_led(DASH_BAR_END, &hp_bar_color);
			}
			// Set battery icon led
			set_led(PWR_LED, &batt_icon_hsv);
			break;
		default:
			break;
		}
		
        // Display led_data
		EA = 0; // Disable global interrupts
        bitbangWs2812(LED_COUNT, led_data);
		EA = 1;	// Enable global interrupts

        mDelaymS(20); // Mostly for Button polling frequency
		
		if (BOOT == 1)
			break; 
    }
	
	// Turn off WS2812's before starting bootloader
	for (i = 0; i < LED_COUNT*3; i++)
		led_data[i] = 0;
	
	// Light the power LED to indicate boot mode
	set_led(PWR_LED, &boot_color);


	EA = 0;	// Disable global interrupts while bitbanging
	bitbangWs2812(LED_COUNT, led_data);
	

	mDelaymS(100); // delay for bootloader
	bootloader();
	while(1);
	
}
