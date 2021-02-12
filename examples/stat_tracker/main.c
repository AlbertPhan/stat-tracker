/* Hyper Light Drifter Stat Tracker firmware
Displays Health, Energy, and Dash points.
Manages buttons to increment points up and down

by Albert Phan 2020

http://www.pegasusprototyping.ca/stattracker

TODO:
[x] Fix main loop timing - main loop time intermittenly breaks and loops continously.
[x] create button library to be reused in other projects
[x] change retrigger to delay + repeat (need timer)
[x] cdc usb serial - if needed for debugging
[x] ADC
[x] Add timer for millis()
[x] charging status
[x] change to HSV for LEDs (convert functions to use HSV)
[x] Touch buttons for brightness control
[x] FIX HSVtoRGB() not properly converting for saturation values above 128
[x] Add eeprom health and energy values saving
	[x] Add initilization to eeprom code
	[x] Add function to save just the stats on eeprom
	[x] save stats everytime they change
[x] Add color configuration
	[x] Add eeprom code to remember colour config
	[x] Fix brightness not toggling when trying to enter colour config mode
	[x] No brightness control while in config
	[x] Create default colour palettes
	[x] Create function to show colour palettes using colour palette type struct
	[x] add eeprom save
[] Fix slow fading not working at 3 seconds but 1 second is ok
[x] Add Startup animation
[x] Fix led showing garbage data on power on
[x] Adjust flickering leds
[x] Fix battery icon showing wrong hue - ADC related? ADC and touch inputs were damaged from ESD. Series protection resitors were added

[x] Fix fill_bar_binary not working past 16 bits
[] Use interrupts for touch buttons if needed to save some time during the update loop (about 4 ms)
[x] Dim display when battery critically low
[x] Add flag for critically low batt so it doesnt toggle the flashing state
[x] Fix battery icon to not fade after constant voltage charged
[x] Add tkey to button stuff so it can retrigger

[x] Add Charging status screen when charging and back to running mode when button pressed
	[x] Display state when charging detected
	[x] Press buttons while in charging state to switch to running state
[x] Adjust min Brightness values for fading
[x] Adjust ADC values with new VCC of 4.28V
[x] Fixed Battery led indicator in battery states (charging and adc)
[x] Change how flash/fade brightness works to use an update function and to reset it.
[x] Flash when entering config state
[] If needing RAM space, change the 32 bit timers to loop counters so it doesnt take up so much ram.




*/

//#define USB_ENABLE

#define bitbang(led_count,led_data)	bitbangSk6812_2020(led_count, led_data)	// choosing correct led (see bitbang.h)

#include <stdint.h>
#include <ch554.h>
#include <debug.h>
#include <bootloader.h>
#include <adc.h>
#include <bitbang.h>
#include <color_utils.h>
#include <touch.h>
#include <button.h>
#include <timer.h>
#include <eeprom.h>


#ifdef USB_ENABLE

#include <usb-cdc.h>

#endif

// Uncomment for access to debug states using dash up and down buttons
//#define DEBUG	

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
#define VCC_VOLTAGE 4.28	// Rev 0.9 has a VCC of 5V
#define BATT_MIN_ADC ((2.50 * 255/VCC_VOLTAGE)) // ADC count for 2.50V/VCC * 255 = 148.9
#define BATT_3V0_ADC ((3.00 * 255/VCC_VOLTAGE))
#define BATT_3V2_ADC ((3.20 * 255/VCC_VOLTAGE))
#define BATT_3V3_ADC ((3.30 * 255/VCC_VOLTAGE)) // ADC count for 3.3V/VCC * 255 = 168
#define BATT_3V9_ADC ((3.90 * 255/VCC_VOLTAGE)) // ADC count for 3.9V/VCC * 255
#define BATT_MAX_ADC ((4.20 * 255/VCC_VOLTAGE)) // ADC count for 4.2V/VCC * 255
#define BATT_CRITICALLY_LOW_ADC BATT_MIN_ADC	// Threshold for critically low batt icon fast flashing
#define BATT_LOW_ADC BATT_3V2_ADC				// Threshold for low batt icon begins flashing 

// Timing defines

#define CHARGED_TIMEOUT_MS 300000	// Time while at constant voltage to be considered fully charged
#define LOW_BATTERY_TIMEOUT_MS 10000	// Time below critically low battery before icon flashes
#define STARTUP_ANIMATION_INTERVAL 25	// Time between led increments for startup animation

#define RED_HUE 0
#define ORANGE_HUE 30
#define YELLOW_HUE 50
#define GREEN_HUE 85
#define TEAL_HUE 125
#define BLUE_HUE 170
#define PURPLE_HUE 195
#define PINK_HUE 225
#define HUE_INCREMENT 5	// Amount adjusted per button press
#define DEFAULT_BRIGHTNESS 25 // Previously 30
#define DEFAULT_SATURATION 255
#define BRIGHTNESS_MAX 60	// TESTING 255 - set to 60 afterwards
#define BRIGHTNESS_STEP 5
#define BRIGHTNESS_MIN 5
#define BRIGHTNESS_CRITICALLY_LOW 10
#define FADE_MIN_BRIGHTNESS 3
#define DEFAULT_HEALTH_HUE 110
#define DEFAULT_HEALTH_ALT_HUE 167
#define DEFAULT_ENERGY_HUE 235
#define DEFAULT_ENERGY_ALT_HUE 35
#define DEFAULT_DASH_HUE 185
#define DEFAULT_DASH_ALT_HUE 210
#define DEFAULT_BATT_CHARGED_HUE GREEN_HUE
#define DEFAULT_BATT_LOW_HUE RED_HUE

#define DEFAULT_HP 10
#define DEFAULT_EP 10
#define DEFAULT_DP 3

__xdata uint8_t brightness = DEFAULT_BRIGHTNESS; // default brightness
__xdata uint8_t saturation = DEFAULT_SATURATION;	// default saturation

// Colors {H,S,V} HSV values go from 0 to 255
__xdata HsvColor hsv_health = {DEFAULT_HEALTH_HUE,DEFAULT_SATURATION,DEFAULT_BRIGHTNESS};
__xdata HsvColor hsv_health_alt = {DEFAULT_HEALTH_ALT_HUE,DEFAULT_SATURATION,DEFAULT_BRIGHTNESS};
__xdata HsvColor hsv_energy = {DEFAULT_ENERGY_HUE,DEFAULT_SATURATION,DEFAULT_BRIGHTNESS}; 
__xdata HsvColor hsv_energy_alt = {DEFAULT_ENERGY_ALT_HUE,DEFAULT_SATURATION,DEFAULT_BRIGHTNESS};
__xdata HsvColor hsv_dash = {DEFAULT_DASH_HUE,DEFAULT_SATURATION,DEFAULT_BRIGHTNESS};
__xdata HsvColor hsv_dash_alt = {DEFAULT_DASH_ALT_HUE,DEFAULT_SATURATION,DEFAULT_BRIGHTNESS};
__xdata HsvColor hsv_boot = {160,180,DEFAULT_BRIGHTNESS};
__xdata HsvColor hsv_off = {0,0,0};
__xdata HsvColor hsv_batt_icon = {0,DEFAULT_SATURATION,DEFAULT_BRIGHTNESS};

__xdata uint8_t batt_charged_hue = DEFAULT_BATT_CHARGED_HUE;
__xdata uint8_t batt_low_hue = DEFAULT_BATT_LOW_HUE;

typedef struct
{
	uint8_t health_hue;
	uint8_t health_alt_hue;
	uint8_t energy_hue;
	uint8_t energy_alt_hue;
	uint8_t dash_hue;
	uint8_t dash_alt_hue;
	uint8_t batt_charged_hue;
	uint8_t batt_low_hue;
}colour_palette;


#define MAX_COLOUR_PALETTE 2
enum cpalette {default_colour_palette, colour_blind,custom};
__xdata enum cpalette current_colour_palette_enum = default_colour_palette;

#define MAX_CONFIG_STATE 7
enum config_state {HEALTH_ALT,HEALTH,ENERGY_ALT,ENERGY,DASH_ALT,DASH,BATT_CHARGED,BATT_LOW,NO_CONFIG,BRIGHTNESS};

// "EEPROM" data flash
#define EEPROM_BUFFER_LEN 14
#define INIT_FLASH_BYTE 0x55
#define EEPROM_READ_SUCCESS_INITIALIZED 1
#define EEPROM_READ_SUCCESS_NO_INIT (1<<1)
#define EEPROM_WRITE_SUCCESS (1<<2)
#define EEPROM_WRITE_FAIL (1<<3)
__xdata uint8_t eeprom_buffer[EEPROM_BUFFER_LEN];
enum address {INIT_BYTE_ADDR,HP_ADDR,EP_ADDR,DP_ADDR,HEALTH_HUE_ADDR,HEALTH_ALT_HUE_ADDR,ENERGY_HUE_ADDR,ENERGY_ALT_HUE_ADDR,DASH_HUE_ADDR,DASH_ALT_HUE_ADDR,BATT_CHARGED_ADDR,BATT_LOW_ADDR,COLOUR_PALETTE_ADDR,BRIGHTNESS_ADDR};
__xdata uint8_t eeprom_flag = 0; // For debugging

#define LED_PIN 2	// addressable led data pin
SBIT(LED, PORT3, LED_PIN); // LED is used in bitbang.c
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
#define MAX_STATES 11
#define BATTERY_CHARGE 1
#define BATTERY_ADC 2	
#define RUNNING 3
#define COLOUR_PALETTE 4
#define TOUCHKEY1 5
#define TOUCHKEY2 6
#define TIMER 7
#define LOOP 8
#define BUTTON 9
#define CONFIG 10
#define STARTUP_ANIMATION 11

__xdata uint8_t led_data[LED_COUNT*3];

// fade timing defines
#define BATTERY_FADE_PERIOD_MS 500 // Number of milliseconds for a period of fading from full to off
#define SLOW_FADE_PERIOD_MS 1000

// struct for keeping track of fading
typedef struct fade
{
	uint8_t frames;	// total frames for fading 0 to max (fade_time/LOOP_PERIOD_MS)
	uint8_t direction;	// 0 is down 1 is up
	uint16_t fade_time;	// time for 0 to max in ms
} fade;

__xdata fade fade_battery = {0,0,BATTERY_FADE_PERIOD_MS};
__xdata fade fade_battery_fast = {0,0,BATTERY_FADE_PERIOD_MS/2};
__xdata fade fade_very_fast = {0,0,BATTERY_FADE_PERIOD_MS/4};
__xdata fade fade_slow = {0,0,SLOW_FADE_PERIOD_MS};



// button structs
__xdata button btn_hp_p = {0,0,0};
__xdata button btn_hp_n = {0,0,0};
__xdata button btn_en_p = {0,0,0};
__xdata button btn_en_n = {0,0,0};
__xdata button btn_dash_p = {0,0,0};
__xdata button btn_dash_n = {0,0,0};
__xdata button btn_charging = {0,0,0};


__xdata touchkey tkey_brightup = {0,0,0,0,0};
__xdata touchkey tkey_brightdown = {0,0,0,0,0};

// Flags
__xdata uint8_t flag_low_batt = 0;

#ifdef USB_ENABLE
// USB ISR
void DeviceInterrupt(void) __interrupt(INT_NO_USB) {
	usbInterrupt();
}
#endif

void update_buttons()
{	
	MUL_P = 0;	// Drive MUL_P low to sense + buttons
	MUL_N = 1;	// Open MUL_N to let it float
	mDelayuS(5); // Give time to switch pins
	
	// Update + buttons
	update_button(&btn_hp_p,!HP_BTN);
	update_button(&btn_en_p,!EN_BTN);
	update_button(&btn_dash_p,!DASH_BTN);

	MUL_P = 1;	// Open MUL_P to let it float	
	MUL_N = 0;	// Drive MUL_N low to sense - buttons
	mDelayuS(5);	// Give time to switch pins

	// Update - buttons
	update_button(&btn_hp_n,!HP_BTN);
	update_button(&btn_en_n,!EN_BTN);
	update_button(&btn_dash_n,!DASH_BTN);
	
	MUL_N = 1; 	// float MUL_N now that we're done checking buttons
	
	// Update touchkeys
	
	update_tkey(&tkey_brightup, getTouchKeyData(BRIGHTNESS_UP_TOUCHKEY_NUM));
	update_tkey(&tkey_brightdown, getTouchKeyData(BRIGHTNESS_DOWN_TOUCHKEY_NUM));

	// Update Charging status (not actually a button)
	update_button(&btn_charging,!STAT);
}

// Set RGB values for a single LED indexed from 1
void set_led(const uint8_t index, const HsvColor *hsv)
{
	RgbColor rgb;	// temp rgb struct
	HsvToRgb(hsv, &rgb);
	led_data[(index-1)*3 + 1] = rgb.r;
	led_data[(index-1)*3 + 0] = rgb.g;
	led_data[(index-1)*3 + 2] = rgb.b;
}

// Set led data off - indexed from 1
void set_led_off(const uint8_t i)
{
	led_data[(i-1)*3 + 1] = 0;
	led_data[(i-1)*3 + 0] = 0;
	led_data[(i-1)*3 + 2] = 0;
}

// Fill in a bar of LEDs
void fill_bar(const uint8_t start, const uint8_t end, const uint8_t value, const HsvColor *hsv)
{
	uint8_t i;
	for (i = 0; i <= end-start; i++)
	{
		if (i < value)	// LED ON
			set_led(start+i, hsv);
		else			// LED OFF
			set_led_off(start+i);
	}
}

// Fill in a bar with binary data (LED starting at LSB) 
void fill_bar_binary(const uint8_t lsb_start, const uint32_t value, const uint8_t bits, const HsvColor *hsv)
{
	uint8_t i;
	for (i = 0; i < bits; i++)	// loop 16 times for 16 bits
	{
		// Is bit set?
		if(value >> i & 1) // Mask 1 bit 
		//if (value & (uint32_t)(1<<i))	// LED ON
			set_led(lsb_start-i, hsv);
		else			// LED OFF
			set_led_off(lsb_start-i);
	}
}



// Clear all of display
void clear_display()
{
	uint8_t i;
	for(i = 0; i < LED_COUNT; i++)
	{
		// set all led data to 0
		set_led_off(i+1);
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
uint16_t map(uint16_t x, const uint16_t in_min, const uint16_t in_max, const uint16_t out_min, const uint16_t out_max)
{
	// clamp min and max values
	if(x > in_max)
	{
		x = in_max;
	}
	if(x < in_min)
	{
		x = in_min;
	}
	return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

uint8_t map8bit(uint8_t x, const uint8_t in_min, const uint8_t in_max, const uint8_t out_min, const uint8_t out_max)
{
	// clamp min and max values
	if(x > in_max)
	{
		x = in_max;
	}
	if(x < in_min)
	{
		x = in_min;
	}
	return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}


// get_fade_frame_brightness() function returns the  brightness value from 0 to brightness based on the current frame of fading , can also do flashing instead
// flash_flag - 0 or 1 - 1 for flashing
uint8_t get_fade_frame_brightness(uint8_t max_brightness, fade *fade_var,uint8_t flash_flag)
{	
	uint8_t return_val = 0;
	// if flash instead of fading
	if(flash_flag)
	{
		return_val = max_brightness * fade_var->direction;
	}
	else
	{
		return_val = max_brightness * fade_var->frames/(fade_var->fade_time/LOOP_PERIOD_MS);
		if(return_val < FADE_MIN_BRIGHTNESS)
		{
			return_val = FADE_MIN_BRIGHTNESS;
		}
	}
	// return a minimal brightness as hue changes dramatically at low brightness values
	return return_val;
}

//update_fade_animation() function updates the fade animation frames each main loop
// every loop all brightnesses that are fading ie charging, critically low batt, other health fading should be updated with this function
void update_fade_animation(uint8_t max_brightness, fade *fade_var)
{
	if(fade_var->direction == 0)	// down
	{
		
		if(fade_var->frames == 0)
		{
			fade_var->direction = 1;	// go back up
		}
		else
		{
			fade_var->frames--;
		}
	}
	else
	{
		if(fade_var->frames >= fade_var->fade_time/LOOP_PERIOD_MS)
		{
			fade_var->direction = 0;	// go back down
		}
		else
		{
			fade_var->frames++;
		}
	}
}

// start_fade_animation() resets the animation to the first frame (OFF for flashing)
void start_fade_animation(uint8_t max_brightness, fade *fade_var)
{
	fade_var->direction = 0;
	// sets frames to the max amount
	fade_var->frames = fade_var->fade_time/LOOP_PERIOD_MS;
}

// start_flash_animation() resets the animation to the last frame (ON for flashing)
void start_flash_animation(uint8_t max_brightness, fade *fade_var)
{
	fade_var->direction = 1;

	fade_var->frames = 0;
}

// Fill entire tracker with the configuration colour palette and flash current value being edited.
void fill_colour_palette(colour_palette * colours, enum config_state config_mode)
{
	HsvColor hsv = {0,DEFAULT_SATURATION,brightness};

	// Health alt colour leds 1-10
	hsv.h = colours->health_alt_hue;
	if(config_mode == HEALTH_ALT || config_mode == NO_CONFIG)
		hsv.v = get_fade_frame_brightness(brightness, &fade_battery,1);
	else
		hsv.v = brightness;
	
	fill_bar(HP_BAR_START,HP_BAR_START+10,255,&hsv);
	// Health colour leds 11-20
	hsv.h = colours->health_hue;
	if(config_mode == HEALTH || config_mode == NO_CONFIG)
		hsv.v = get_fade_frame_brightness(brightness, &fade_battery,1);	
	else
		hsv.v = brightness;
	fill_bar(HP_BAR_START+10,HP_BAR_END,255,&hsv);
	// energy alt colour leds 21-30
	hsv.h = colours->energy_alt_hue;
	if(config_mode == ENERGY_ALT || config_mode == NO_CONFIG)
		hsv.v = get_fade_frame_brightness(brightness, &fade_battery,1);
	else
		hsv.v = brightness;
	fill_bar(EN_BAR_START,EN_BAR_START+10,255,&hsv);
	// energy colour leds 31-40
	hsv.h = colours->energy_hue;
	if(config_mode == ENERGY || config_mode == NO_CONFIG)
		hsv.v = get_fade_frame_brightness(brightness, &fade_battery,1);
	else
		hsv.v = brightness;
	fill_bar(EN_BAR_START+10,EN_BAR_END,255,&hsv);
	// dash alt colour leds 41-43
	hsv.h = colours->dash_alt_hue;
	if(config_mode == DASH_ALT || config_mode == NO_CONFIG)
		hsv.v = get_fade_frame_brightness(brightness, &fade_battery,1);
	else
		hsv.v = brightness;
	fill_bar(DASH_BAR_START,DASH_BAR_START+2,255,&hsv);
	// dash colour leds 44-45
	hsv.h = colours->dash_hue;
	if(config_mode == DASH || config_mode == NO_CONFIG)
		hsv.v = get_fade_frame_brightness(brightness, &fade_battery,1);
	else
		hsv.v = brightness;
	fill_bar(DASH_BAR_START+2,DASH_BAR_END,255,&hsv);

	// charged batt colour
	if(config_mode == BATT_CHARGED)
	{
		hsv.h = colours->batt_charged_hue;
		hsv.v = get_fade_frame_brightness(brightness, &fade_battery,1);
	}
	else if(config_mode == BATT_LOW)
	{
		hsv.h = colours->batt_low_hue;
		hsv.v = get_fade_frame_brightness(brightness, &fade_very_fast,1);
	}	
	else
		hsv.v = brightness;
	
	// fade between batt charged and low colours when not in config
	if((config_mode != BATT_CHARGED) && (config_mode != BATT_LOW))
	{
		hsv.h = map8bit(fade_slow.frames,0,fade_slow.fade_time/LOOP_PERIOD_MS,colours->batt_low_hue,colours->batt_charged_hue);
	}
	set_led(PWR_LED,&hsv);

}

// returns a value after it has been incremented or decremented and clamped to min 0 or max 255
// increment - true to increment, false to decrement
// clamp - true to clamp to min 0 or max 255, false to loop around
uint8_t adjust8bit(uint8_t value, uint8_t amount_change, uint8_t increment, uint8_t clamp)
{
	// if increasing
	if(increment)
	{
		// Increment hue if less than 251
		if(clamp)
		{
			if (value <= 255 - amount_change)
			{
				value += amount_change;
			}
		}
		else
		{
			value += amount_change;	// when overflow, it should loop back to 0
		}
		
	}
	else // if decreasing
	{
		if(clamp)
		{// Decrement saturation if more than 4
			if(value >= 0 + amount_change)
			{
				value -= amount_change;
			}
		}
		else
		{
			value -= amount_change; // when carry, it should loop back to 255
		}
	}
	return value;
}

/* This function adjusts one part of a colour palette by HUE_INCREMENT.
Inputs: colours - colour_palette pointer
config - which part of the palette to change
increment - true to increment , false to decrement
*/
void adjust_colour_palette(colour_palette * colours, enum config_state config, uint8_t increment)
{	
	switch (config)
	{
	case HEALTH:
		colours->health_hue = adjust8bit(colours->health_hue,HUE_INCREMENT,increment,0);
		break;
	case HEALTH_ALT:
		colours->health_alt_hue = adjust8bit(colours->health_alt_hue,HUE_INCREMENT,increment,0);
		break;
	case ENERGY:
		colours->energy_hue = adjust8bit(colours->energy_hue,HUE_INCREMENT,increment,0);
		break;
	case ENERGY_ALT:
		colours->energy_alt_hue = adjust8bit(colours->energy_alt_hue,HUE_INCREMENT,increment,0);
		break;
	case DASH:
		colours->dash_hue = adjust8bit(colours->dash_hue,HUE_INCREMENT,increment,0);
		break;
	case DASH_ALT:
		colours->dash_alt_hue = adjust8bit(colours->dash_alt_hue,HUE_INCREMENT,increment,0);
		break;
	case BATT_CHARGED:
		colours->batt_charged_hue = adjust8bit(colours->batt_charged_hue,HUE_INCREMENT,increment,0);
		break;
	case BATT_LOW:
		colours->batt_low_hue = adjust8bit(colours->batt_low_hue,HUE_INCREMENT,increment,0);
		break;
	default:
		break;
	}
}

// Changes the current colour palette
// colours - pass address of colour_palette to change to
void change_current_cpalette(colour_palette * colours)
{
	hsv_health.h = colours->health_hue;
	hsv_health_alt.h = colours->health_alt_hue;
	hsv_energy.h = colours->energy_hue;
	hsv_energy_alt.h = colours->energy_alt_hue;
	hsv_dash.h = colours->dash_hue;
	hsv_dash_alt.h = colours->dash_alt_hue;
	batt_charged_hue = colours->batt_charged_hue;
	batt_low_hue = colours->batt_low_hue;
}

// Sets the battery icon based on charge value and whether it is charging via usb or not
// Call once per update main 
void update_battery_icon_led(const uint8_t batt_charge_adc)
{
	__xdata static uint32_t batt_millis = 0; // Used for charging timing
	__xdata static uint8_t batt_charge_adc_flag = BATT_LOW_ADC + 1;	
	__xdata static uint8_t batt_constantvoltage_flag = 0;

	hsv_batt_icon.v = brightness;

	// Do stuff while battery is charging
	if(STAT == 0)	// If charging 
	{
		if(batt_charge_adc_flag != BATT_LOW_ADC + 1)
		{
				batt_charge_adc_flag = BATT_LOW_ADC + 1;	// Reset flag
				flag_low_batt = 0;
		}
		// If charger is in constant voltage mode, enable flag and start timing
		if(batt_charge_adc >= BATT_MAX_ADC)
			batt_constantvoltage_flag = 1;

		if(batt_constantvoltage_flag)
			batt_millis += LOOP_PERIOD_MS;	// increment charged time by 1 LOOP_PERIOD_MS
		else	// if not constant voltage
			// reset timer
			batt_millis = 0;

		// if charging has NOT been in constant voltage mode for CHARGED_TIMEOUT_MS then flash batt icon
		if(batt_millis < CHARGED_TIMEOUT_MS)
		{
			// flash charging icon
			hsv_batt_icon.v = get_fade_frame_brightness(brightness,&fade_battery,1);
		}
	}
	else // Not charging
	{
		batt_constantvoltage_flag = 0; // reset flag when not charging
		if(batt_charge_adc < batt_charge_adc_flag)	// if batt critically low
		{
			batt_charge_adc_flag = batt_charge_adc;	// Only allow adc flag to go lower (To stop noisy adc from triggering fading states)
		}

		if (batt_charge_adc_flag <= BATT_LOW_ADC)	// if less than Batt low adc display critically low state
		{
			// lower brightness and flash faster based on battery level
			flag_low_batt = 1; 
			brightness = map(batt_charge_adc_flag, BATT_CRITICALLY_LOW_ADC, BATT_LOW_ADC, BRIGHTNESS_MIN,BRIGHTNESS_CRITICALLY_LOW);
			fade_battery_fast.fade_time = map(batt_charge_adc_flag, BATT_CRITICALLY_LOW_ADC, BATT_LOW_ADC,BATTERY_FADE_PERIOD_MS/4,BATTERY_FADE_PERIOD_MS);
			hsv_batt_icon.v = get_fade_frame_brightness(brightness, &fade_battery_fast,1);
		}
	}
	// Show battery charge from fully charged colour to low colour
	hsv_batt_icon.h = map(batt_charge_adc,BATT_3V0_ADC,BATT_MAX_ADC,batt_low_hue,batt_charged_hue);
	set_led(PWR_LED, &hsv_batt_icon);
}

// Writes initial data to eeprom flash
// returns # bytes written
uint8_t eeprom_write_defaults()
{
	uint8_t bytes_written;	// TODO: change to custom colours since default colours need not be saved to eeprom
	eeprom_buffer[INIT_BYTE_ADDR] = INIT_FLASH_BYTE;
	eeprom_buffer[HP_ADDR] = DEFAULT_HP;
	eeprom_buffer[EP_ADDR] = DEFAULT_EP;
	eeprom_buffer[DP_ADDR] = DEFAULT_DP;
	eeprom_buffer[HEALTH_HUE_ADDR] = DEFAULT_HEALTH_HUE;
	eeprom_buffer[HEALTH_ALT_HUE_ADDR] = DEFAULT_HEALTH_ALT_HUE;
	eeprom_buffer[ENERGY_HUE_ADDR] = DEFAULT_ENERGY_HUE;
	eeprom_buffer[ENERGY_ALT_HUE_ADDR] = DEFAULT_ENERGY_ALT_HUE;
	eeprom_buffer[DASH_HUE_ADDR] = DEFAULT_DASH_HUE; 
	eeprom_buffer[DASH_ALT_HUE_ADDR] = DEFAULT_DASH_ALT_HUE;
	eeprom_buffer[BATT_LOW_ADDR] = RED_HUE;
	eeprom_buffer[BATT_CHARGED_ADDR] = GREEN_HUE;
	eeprom_buffer[COLOUR_PALETTE_ADDR] = default_colour_palette;
	eeprom_buffer[BRIGHTNESS_ADDR] = DEFAULT_BRIGHTNESS;
	bytes_written = WriteDataFlash(INIT_BYTE_ADDR,eeprom_buffer, EEPROM_BUFFER_LEN);
	if(bytes_written == EEPROM_BUFFER_LEN)
		eeprom_flag = EEPROM_WRITE_SUCCESS;	// write success // remove?
	else
		eeprom_flag = EEPROM_WRITE_FAIL; // failed write
	return bytes_written;
}

// Saves current colour palette to dataflash
// returns # bytes written

uint8_t eeprom_write_custom_colours(enum cpalette current_colour_palette,colour_palette * colours)
{
	uint8_t bytes_written;
	eeprom_buffer[INIT_BYTE_ADDR] = INIT_FLASH_BYTE;
	eeprom_buffer[HEALTH_HUE_ADDR] = colours->health_hue;
	eeprom_buffer[HEALTH_ALT_HUE_ADDR] = colours->health_alt_hue;
	eeprom_buffer[ENERGY_HUE_ADDR] = colours->energy_hue;
	eeprom_buffer[ENERGY_ALT_HUE_ADDR] = colours->energy_alt_hue;
	eeprom_buffer[DASH_HUE_ADDR] = colours->dash_hue; 
	eeprom_buffer[DASH_ALT_HUE_ADDR] = colours->dash_alt_hue;
	eeprom_buffer[BATT_LOW_ADDR] = colours->batt_low_hue;
	eeprom_buffer[BATT_CHARGED_ADDR] = colours->batt_charged_hue;
	eeprom_buffer[COLOUR_PALETTE_ADDR] = current_colour_palette;
	bytes_written = WriteDataFlash(INIT_BYTE_ADDR,eeprom_buffer, EEPROM_BUFFER_LEN);
	if(bytes_written == EEPROM_BUFFER_LEN)
		eeprom_flag = EEPROM_WRITE_SUCCESS;	// write success // remove?
	else
		eeprom_flag = EEPROM_WRITE_FAIL; // failed write
	return bytes_written;
}


uint8_t eeprom_write_stats(enum config_state stat, uint8_t value)
{
	uint8_t bytes_written;
	
	switch (stat)
	{
	case HEALTH:
		bytes_written = WriteDataFlash(HP_ADDR,&value,1);
		break;
	case ENERGY:
		bytes_written = WriteDataFlash(EP_ADDR,&value,1);
		break;
	case DASH:
		bytes_written = WriteDataFlash(DP_ADDR,&value,1);
		break;
	case BRIGHTNESS:
		bytes_written = WriteDataFlash(BRIGHTNESS_ADDR,&value,1);
		break;
	default:
		break;
	}
	if(bytes_written == 1)
		eeprom_flag = EEPROM_WRITE_SUCCESS;	// write success // remove?
	else
		eeprom_flag = EEPROM_WRITE_FAIL; // failed write
	return bytes_written;
}



void main() 
{
	__xdata uint8_t hp_value = DEFAULT_HP;
	__xdata uint8_t energy_value = DEFAULT_EP;
	__xdata uint8_t dash_value = DEFAULT_DP;
	__xdata uint8_t startup_hp_value = 0;		// Used for Startup animation
	__xdata uint8_t startup_energy_value = 0;	// Used for Startup animation
	__xdata uint8_t startup_dash_value = 0;		// Used for Startup animation
	__xdata uint8_t state = STARTUP_ANIMATION;	// Default State
	__xdata uint16_t battery_charge_adc = 0; // from 0 - 255 and also used for summing (need 16 bits)
	__xdata uint8_t buffer_index = 0;
	//__xdata uint16_t i;	// General Purpose index
	__xdata uint32_t prev_millis = 0; // used for main loop
	__xdata uint32_t prev_startup_millis = 0; // used for startup animation
	__idata uint32_t current_millis = 0;
	__xdata uint8_t battery_adc_buffer[AVERAGING_SIZE];
	__xdata HsvColor hsv = {0,DEFAULT_SATURATION,DEFAULT_BRIGHTNESS};	// General Purpose hsv
	
	__xdata static enum config_state current_config_state = HEALTH_ALT;
	// Colour palettes - must define here inside main()

	// Default Colours
	__xdata const colour_palette default_colours =
	{
		.health_hue = DEFAULT_HEALTH_HUE,
		.health_alt_hue = DEFAULT_HEALTH_ALT_HUE,
		.energy_hue = DEFAULT_ENERGY_HUE,
		.energy_alt_hue = DEFAULT_ENERGY_ALT_HUE,
		.dash_hue = DEFAULT_DASH_HUE,
		.dash_alt_hue = DEFAULT_DASH_ALT_HUE,
		.batt_charged_hue = DEFAULT_BATT_CHARGED_HUE,
		.batt_low_hue = DEFAULT_BATT_LOW_HUE
	};

	// Colour blind deu colours
	__xdata const colour_palette colour_blind_colours =
	{
		.health_hue = BLUE_HUE,
		.health_alt_hue = TEAL_HUE,
		.energy_hue = RED_HUE,
		.energy_alt_hue = YELLOW_HUE,
		.dash_hue = BLUE_HUE,
		.dash_alt_hue = TEAL_HUE,
		.batt_charged_hue = BLUE_HUE,
		.batt_low_hue = RED_HUE
	};


	__xdata colour_palette custom_colours;
	custom_colours.health_hue = GREEN_HUE;
	custom_colours.health_alt_hue = TEAL_HUE; 
	custom_colours.energy_hue = RED_HUE;
	custom_colours.energy_alt_hue = ORANGE_HUE; 
	custom_colours.dash_hue = PURPLE_HUE;	
	custom_colours.dash_alt_hue = BLUE_HUE;
	custom_colours.batt_charged_hue = PINK_HUE;
	custom_colours.batt_low_hue = RED_HUE;

	CfgFsys();
	bitbangSetup(3,LED_PIN); // using 3.2 pin as led pin
	mDelaymS(5);

	// GPIO Setup *****************************
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


	// Module Initilizations ******************************************************
	EA = 1;	// Enable global interrupts
	timer0_init();	// timer0 for millis()
	enable_timer0_interrupt();

	#ifdef USB_ENABLE 
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
	#endif
	
    ADCInit(1); 	// Set fast mode adc
	ADC_ChannelSelect(2);	// Set ADC channel AIN2 for VBAT

	// Touch key setup 
	touchKeyQueryCyl2ms();	// set period selection
	// Get no touchkey pressed data
	tkey_brightup.nokey = getNokeyData(BRIGHTNESS_UP_TOUCHKEY_NUM);	// set no key threshold
	tkey_brightdown.nokey = getNokeyData(BRIGHTNESS_DOWN_TOUCHKEY_NUM);	// set no key threshold

	// Initialize the adc buffer array with AVERAGING_SIZE good samples
	for(uint8_t i = 0; i< AVERAGING_SIZE; i++)
	{
		ADC_START = 1;	// Start ADC conversion
		while(ADC_START == 1); // Wait for conversion done
		battery_adc_buffer[i] = ADC_DATA;// add to averaging Buffer
	}
	EA = 0; // disable global interrupts
	current_millis = millis();
	prev_millis = current_millis; // set before going into loop
	prev_startup_millis = current_millis;
	EA = 1;	// enable global interrupts

	// EEPROM setup
	// Read flash
	if(ReadDataFlash(INIT_BYTE_ADDR,EEPROM_BUFFER_LEN,eeprom_buffer) == EEPROM_BUFFER_LEN) // if read successfully 
	{
		if(eeprom_buffer[INIT_BYTE_ADDR] == 0x55)	// if it has been initilizated then use the data
		{
			hp_value = eeprom_buffer[HP_ADDR];
			energy_value = eeprom_buffer[EP_ADDR];
			dash_value = eeprom_buffer[DP_ADDR];
			current_colour_palette_enum = eeprom_buffer[COLOUR_PALETTE_ADDR];
			brightness = eeprom_buffer[BRIGHTNESS_ADDR];
			switch (current_colour_palette_enum)
			{
			case default_colour_palette:
				change_current_cpalette(&default_colours);
				break;
			case colour_blind:
				change_current_cpalette(&colour_blind_colours);
				break;
			case custom:
				// get custom colour data
				custom_colours.health_hue = eeprom_buffer[HEALTH_HUE_ADDR];
				custom_colours.health_alt_hue = eeprom_buffer[HEALTH_ALT_HUE_ADDR];
				custom_colours.energy_hue = eeprom_buffer[ENERGY_HUE_ADDR];
				custom_colours.energy_alt_hue = eeprom_buffer[ENERGY_ALT_HUE_ADDR];
				custom_colours.dash_hue = eeprom_buffer[DASH_HUE_ADDR];
				custom_colours.dash_alt_hue = eeprom_buffer[DASH_ALT_HUE_ADDR];
				custom_colours.batt_charged_hue = eeprom_buffer[BATT_CHARGED_ADDR];
				custom_colours.batt_low_hue = eeprom_buffer[BATT_LOW_ADDR];
				change_current_cpalette(&custom_colours);
				break;
			default:
				break;
			}
			eeprom_flag = EEPROM_READ_SUCCESS_INITIALIZED; // testing eeprom initilized
		}
		else 
		{
			eeprom_flag = EEPROM_READ_SUCCESS_NO_INIT; // testing read eeprom success but not initilized
			eeprom_write_defaults();
		}
	}
	else
	{
		// if not read successfully then colours will be set to default(already done above) and write to the flash if possible
		eeprom_write_defaults();
	}
	
	


	// Main loop *****************************************************************************************************************************************
    while (1) 
	{
		
		// Run loop every LOOP_PERIOD_MS
		EA = 0; // disable global interrupts
		current_millis = millis();
		EA = 1; // enable global interrupts

		// Startup Animation Timing
		// Interval passed? increase displayed value if in startup_animation
		if(current_millis-prev_startup_millis >= STARTUP_ANIMATION_INTERVAL)
		{
			prev_startup_millis += STARTUP_ANIMATION_INTERVAL;

			if(state == STARTUP_ANIMATION)
			{
				// Increment data values as long as they are not at max
				if(startup_hp_value < hp_value)
				{
					startup_hp_value++;
				}
				if(startup_energy_value < energy_value)
				{
					startup_energy_value++;
				}
				if(startup_dash_value < dash_value)
				{
					startup_dash_value++;
				}
			}
		}

		if(current_millis-prev_millis >= LOOP_PERIOD_MS)
		{
			prev_millis += LOOP_PERIOD_MS;
			#ifdef USB_ENABLE 
			UsbCdc_puts("prev_millis:");	// TESTING
			UsbCdc_puti32(prev_millis);
			UsbCdc_puts(" millis():");
			UsbCdc_puti32(current_millis);
			UsbCdc_puts(" diff:");
			UsbCdc_puti32(current_millis-prev_millis);
			UsbCdc_puts(" loop:");
			UsbCdc_puti32(timing_millis);
			UsbCdc_puts("\r\n");
			#endif

			update_buttons(); 
			update_fade_animation(brightness,&fade_battery);
			update_fade_animation(brightness,&fade_battery_fast);
			update_fade_animation(brightness,&fade_very_fast);
			update_fade_animation(brightness,&fade_slow);

			// ADC Loop
			// Start battery voltage adc stuff
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
			battery_charge_adc = 0; // reset battery_charge_adc
			for(uint8_t i = 0; i < AVERAGING_SIZE; i++)
			{
				// Add all values in buffer
				battery_charge_adc += battery_adc_buffer[i];
			}
			// Divide by AVERAGING_SIZE to get average
			battery_charge_adc = battery_charge_adc/AVERAGING_SIZE;

			

			#ifdef USB_ENABLE
			UsbCdc_processOutput();
			UsbCdc_processInput();
			#endif

			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Do other button stuff related to buttons at the same time
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////


			// Brightness button controls
			if(!flag_low_batt && !(state == BUTTON))
			{
				if(de_retrigger_tkey(&tkey_brightup) && !read_tkey(&tkey_brightdown))
				{
				// Increment value if <= BRIGHTNESS_MAX - BRIGHTNESS_STEP and not low battery
					if (brightness <= BRIGHTNESS_MAX - BRIGHTNESS_STEP)
					{
						brightness += BRIGHTNESS_STEP;
						eeprom_write_stats(BRIGHTNESS,brightness);
					}
				}
				if((de_retrigger_tkey(&tkey_brightdown)) && !read_tkey(&tkey_brightup))
				{
					// Decrement value if >= 0 + BRIGHTNESS_STEP and not low battery
					if( brightness > BRIGHTNESS_STEP)
					{
						brightness -= BRIGHTNESS_STEP;
						eeprom_write_stats(BRIGHTNESS,brightness);
					}
				}
			}
			if(long_held_tkey(&tkey_brightup) && long_held_tkey(&tkey_brightdown))
			{
				state = CONFIG;
				current_config_state = 0;
				// Begin with off leds to show config mode has been entered.
				//start_fade_animation(brightness, &fade_battery);
			}

			// Goes to charging status when charging detected
			if(activated(&btn_charging)) // if charging
			{
				start_fade_animation(brightness,&fade_battery);
				state = BATTERY_CHARGE;
			}
			// Return to STARTUP_ANIMATION state after charging ends or if button press detected
			if(state == BATTERY_CHARGE)
			{
				if(deactivated(&btn_charging) || activated(&btn_dash_n)||activated(&btn_dash_p)||activated(&btn_en_n)||activated(&btn_en_p)||activated(&btn_hp_n)||activated(&btn_hp_p))
				{
					state = STARTUP_ANIMATION;
				}
			}
			

			// Goes to boot loader when both dash n and hp n buttons held
			if(held(&btn_dash_n) && held(&btn_hp_n))
			{
				// Turn off WS2812's before starting bootloader
				for (uint8_t i = 0; i < LED_COUNT*3; i++)
					led_data[i] = 0;
				
				// Light the power LED to indicate boot mode
				set_led(PWR_LED, &hsv_boot);

				bitbang(LED_COUNT, led_data);
				
				mDelaymS(100); // delay for bootloader
				#ifdef USB_ENABLE
				jumpToBootloader(); // usb cdc bootloader
				#else	// otherwise use normal bootloader
				bootloader();
				#endif
				while(1);
			}
			// change all colors brightness
			hsv_health.v = brightness; 
			hsv_health_alt.v = brightness;
			hsv_energy.v =  brightness;
			hsv_energy_alt.v = brightness;
			hsv_dash.v = brightness;
			hsv_dash_alt.v = brightness; 
			hsv_boot.v = brightness;
			hsv.v = brightness;

			

			#ifdef DEBUG
			// DEBUG using dash buttons to switch between states
			if(state != CONFIG)
			{
				if(de_retrigger(&btn_dash_p))
				{
					if(state < MAX_STATES)
					{
						state++;
					}
				}
				if(de_retrigger(&btn_dash_n))
				{
					if(state > 1)
					{
						state--;
					}
				}
			}
			#endif
			// Adjust stats in RUNNING state
			if(state == RUNNING)
			{
				// clamp to 2 * full_bar_length for overfill
				
				if (hp_value < 2*(FULL_BAR_LENGTH) && de_retrigger(&btn_hp_p))
					hp_value++;
				else if (hp_value > 0 && de_retrigger(&btn_hp_n))
					hp_value--;
				if(de_retrigger(&btn_hp_p) || de_retrigger(&btn_hp_n))
					eeprom_write_stats(HEALTH, hp_value);
				
				if (energy_value < 2*(FULL_BAR_LENGTH) && de_retrigger(&btn_en_p))
					energy_value++;
				else if (energy_value > 0 && de_retrigger(&btn_en_n))
					energy_value--;
				if(de_retrigger(&btn_en_p) || de_retrigger(&btn_en_n))
					eeprom_write_stats(ENERGY, energy_value);
				
				if (dash_value < 2*(DASH_BAR_LENGTH) && de_retrigger(&btn_dash_p))
					dash_value++;
				else if (dash_value > 0 && de_retrigger(&btn_dash_n))
					dash_value--;
				if(de_retrigger(&btn_dash_p) || de_retrigger(&btn_dash_n))
					eeprom_write_stats(DASH, dash_value);

			}
			
			// Saturation Control (only for state COLOR_PALLETTE)
			if(state == COLOUR_PALETTE)
			{
				if(de_retrigger(&btn_hp_p))
				{
					hsv.s = adjust8bit(hsv.s,HUE_INCREMENT,1,0);
					
				}
				if(de_retrigger(&btn_hp_n))
				{
					hsv.s = adjust8bit(hsv.s,HUE_INCREMENT,0,0);
				}
			}

			clear_display();

			// Update and fill battery icon
			if(state != CONFIG)
				update_battery_icon_led(battery_charge_adc);

			switch (state)
			{
			case BATTERY_CHARGE:
				// Show battery charge for user
				// Fade battery charge using leds 1-20 to represent charge using batt icon colour
				fill_bar(HP_BAR_START, HP_BAR_END, map8bit(battery_charge_adc,BATT_CRITICALLY_LOW_ADC,BATT_MAX_ADC,0,FULL_BAR_LENGTH), &hsv_batt_icon);
				break;

			case BATTERY_ADC:
				// fill bar based on battery charge for now while testing
				hsv.h = hsv_health.h;
				hsv.s = DEFAULT_SATURATION;
				fill_bar(HP_BAR_START, HP_BAR_END, map8bit(battery_charge_adc,BATT_CRITICALLY_LOW_ADC,BATT_MAX_ADC,0,FULL_BAR_LENGTH), &hsv);
				hsv.h = hsv_health.h + 10;
				fill_bar_binary(EN_BAR_END,battery_charge_adc,8,&hsv);
				if(STAT == 0)
				{
					hsv.h = ORANGE_HUE;
					set_led(DASH_BAR_END,&hsv);
				}
				else
					set_led_off(DASH_BAR_END);
				break;
			case RUNNING:	// Normal operating mode
				// Draw bars
				fill_bar(HP_BAR_START, HP_BAR_END, hp_value, &hsv_health);
				if (hp_value > FULL_BAR_LENGTH)
					fill_bar(HP_BAR_START, HP_BAR_START - 1 + hp_value - FULL_BAR_LENGTH, DEFAULT_SATURATION, &hsv_health_alt);
				
				fill_bar(EN_BAR_START, EN_BAR_END, energy_value, &hsv_energy);
				if (energy_value > FULL_BAR_LENGTH)
					fill_bar(EN_BAR_START, EN_BAR_START - 1 + energy_value - FULL_BAR_LENGTH, DEFAULT_SATURATION, &hsv_energy_alt);
				
				fill_bar(DASH_BAR_START, DASH_BAR_END, dash_value, &hsv_dash);
				if (dash_value > DASH_BAR_LENGTH)
					fill_bar(DASH_BAR_START, DASH_BAR_START - 1 + dash_value - DASH_BAR_LENGTH, DEFAULT_SATURATION, &hsv_dash_alt);
				break;
			case COLOUR_PALETTE:	// Demo of all the hues - Increments of 5
				hsv.v = brightness;
				for(uint8_t i = 0; i < LED_COUNT; i++)
				{
					// go through leds and change hsv for each
					hsv.h = 5*i;
					set_led(i+1,&hsv);
				}
				break;
			case TOUCHKEY1: // Debug data for TIN1 BRIGHTNESS_DOWN_TOUCHKEY_NUM
				// REMOVE later for space saving
				// Display nokey data
				fill_bar_binary(HP_BAR_END,tkey_brightdown.nokey,16,&hsv_health);
				// Display capacitance data
				fill_bar_binary(EN_BAR_END,tkey_brightdown.data,16,&hsv_energy);

				set_led(DASH_BAR_START, &hsv_dash);

				// Turn on dash leds if tkey is on
				if(read_tkey(&tkey_brightdown))
				{
					set_led(DASH_BAR_END-1, &hsv_health);
				}
				if(read_tkey(&tkey_brightup))
				{
					set_led(DASH_BAR_END, &hsv_health);
				}
				break;
			case TOUCHKEY2:	// Debug data for TIN2 BRIGHTNESS_UP_TOUCHKEY_NUM
				// REMOVE later for space saving
				// Display capacitance data values on hp and en bars
				fill_bar_binary(HP_BAR_END,tkey_brightup.nokey,16,&hsv_health);
				// Display capacitance data
				fill_bar_binary(EN_BAR_END,tkey_brightup.data,16,&hsv_energy);
				fill_bar(DASH_BAR_START,DASH_BAR_START + 1,2, &hsv_dash);	// turn on 2 dash leds to show state

				// Turn on dash leds if tkey is on
				if(read_tkey(&tkey_brightdown))
				{
					set_led(DASH_BAR_END-1, &hsv_health);
				}
				if(read_tkey(&tkey_brightup))
				{
					set_led(DASH_BAR_END, &hsv_health);
				}
				break;
			case TIMER:
				//// display millis
				//hsv.h = BLUE_HUE;
				
				// // Display 1/100ths of seconds in 1-10 leds
				// fill_bar(HP_BAR_START, HP_BAR_START + 9,map(millis()/10%10+1,0,10,0,10),&hsv);
				// // Display 1/10ths of seconds in 11-20
				// fill_bar(HP_BAR_START + 10, HP_BAR_END,map(millis()/100%10+1,0,10,0,10),&hsv);
				// // Display seconds in 21-30
				// fill_bar(EN_BAR_START, EN_BAR_START + 9,map(millis()/1000%10+1,0,10,0,10),&hsv);
				// // Display decaseconds in 31-40
				// fill_bar(EN_BAR_START + 10, EN_BAR_END,map(millis()/10000%10+1,0,10,0,10),&hsv);


				//fill_bar_binary(EN_BAR_END,current_millis,32,&hsv); //
				break;
			case LOOP:
				//	// REMOVE later for space saving
				// // loop counter display
				// fill_bar_binary(HP_BAR_END,prev_millis,20,&hsv_dash);	// TESTING show 20 bits of prev_millis
				// fill_bar_binary(EN_BAR_END,current_millis,20,&hsv_dash_alt);	// TESTING
				break;
			case BUTTON:
				// REMOVE later for flash saving if needed
				if(read_tkey(&tkey_brightup))
				{
					fill_bar_binary(HP_BAR_END,tkey_brightup.time_loops,16,&hsv_dash);
					fill_bar_binary(EN_BAR_END,tkey_brightup.state,8,&hsv_dash);
					if(retrigger_tkey(&tkey_brightup))
						set_led(DASH_BAR_END,&hsv_dash);
					else
						set_led_off(DASH_BAR_END);
				}
				else if(read_tkey(&tkey_brightdown))
				{
					fill_bar_binary(HP_BAR_END,tkey_brightdown.time_loops,16,&hsv_dash);
					fill_bar_binary(EN_BAR_END,tkey_brightdown.state,8,&hsv_dash);
					if(retrigger_tkey(&tkey_brightdown))
						set_led(DASH_BAR_END,&hsv_dash);
					else
						set_led_off(DASH_BAR_END);
				}
				else if(read(&btn_hp_p))
				{
					fill_bar_binary(HP_BAR_END,btn_hp_p.time_loops,16,&hsv_dash);
					fill_bar_binary(EN_BAR_END,btn_hp_p.state,8,&hsv_dash);
					if(retrigger(&btn_hp_p))
						set_led(DASH_BAR_END,&hsv_dash);
					else
						set_led_off(DASH_BAR_END);
				}
				else if(read(&btn_hp_n))
				{
					fill_bar_binary(HP_BAR_END,btn_hp_n.time_loops,16,&hsv_dash);
					fill_bar_binary(EN_BAR_END,btn_hp_n.state,8,&hsv_dash);
					if(retrigger(&btn_hp_n))
						set_led(DASH_BAR_END,&hsv_dash);
					else
						set_led_off(DASH_BAR_END);
				}
				else if(read(&btn_en_p))
				{
					fill_bar_binary(HP_BAR_END,btn_en_p.time_loops,16,&hsv_dash);
					fill_bar_binary(EN_BAR_END,btn_en_p.state,8,&hsv_dash);
					if(retrigger(&btn_en_p))
						set_led(DASH_BAR_END,&hsv_dash);
					else
						set_led_off(DASH_BAR_END);
				}
				else if(read(&btn_en_n))
				{
					fill_bar_binary(HP_BAR_END,btn_en_n.time_loops,16,&hsv_dash);
					fill_bar_binary(EN_BAR_END,btn_en_n.state,8,&hsv_dash);
					if(retrigger(&btn_en_n))
						set_led(DASH_BAR_END,&hsv_dash);
					else
						set_led_off(DASH_BAR_END);
				}
				else if(read(&btn_dash_p))
				{
					fill_bar_binary(HP_BAR_END,btn_dash_p.time_loops,16,&hsv_dash);
					fill_bar_binary(EN_BAR_END,btn_dash_p.state,8,&hsv_dash);
					if(retrigger(&btn_dash_p))
						set_led(DASH_BAR_END,&hsv_dash);
					else
						set_led_off(DASH_BAR_END);
				}
				else if(read(&btn_dash_n))
				{
					fill_bar_binary(HP_BAR_END,btn_dash_n.time_loops,16,&hsv_dash);
					fill_bar_binary(EN_BAR_END,btn_dash_n.state,8,&hsv_dash);
					if(retrigger(&btn_dash_n))
						set_led(DASH_BAR_END,&hsv_dash);
					else
						set_led_off(DASH_BAR_END);
				}

				break;
			case CONFIG:
				// Show the current colour palette
				switch (current_colour_palette_enum)
				{
				case default_colour_palette:
					fill_colour_palette(&default_colours,NO_CONFIG);
					break;
				case colour_blind:
					fill_colour_palette(&colour_blind_colours,NO_CONFIG);
					break;
				case custom:
					// health buttons to adjust hue
					if(de_retrigger(&btn_hp_p))
					{
						adjust_colour_palette(&custom_colours,current_config_state,1);
						start_flash_animation(brightness,&fade_battery);
					}
						
					if(de_retrigger(&btn_hp_n))
					{
						adjust_colour_palette(&custom_colours,current_config_state,0);
						start_flash_animation(brightness,&fade_battery);
					}
					fill_colour_palette(&custom_colours,current_config_state);
					break;
				default:
					break;
				}

				// up and down adjust colour palette
				if(deactivated(&btn_dash_p))
				{
					if(current_colour_palette_enum >= MAX_COLOUR_PALETTE)
						current_colour_palette_enum = MAX_COLOUR_PALETTE;
					else
					{
							current_colour_palette_enum++;
							start_fade_animation(brightness, &fade_battery);
					}
					
				}
				if(deactivated(&btn_dash_n))
				{
					if(current_colour_palette_enum <= 0)
						current_colour_palette_enum = 0;
					else
					{
						current_colour_palette_enum--;
						start_fade_animation(brightness, &fade_battery);
					}
				}
				// dash + and - move which colour to be edited (flashing the color)
                if(activated(&btn_en_p))
                {
                    if(current_config_state >= MAX_CONFIG_STATE)
                        current_config_state = MAX_CONFIG_STATE;
					else
						current_config_state++;
					start_fade_animation(brightness, &fade_battery);
                }
				if(activated(&btn_en_n))
				{
					if(current_config_state <= 0)
						current_config_state = 0;
					else
						current_config_state--;
					start_fade_animation(brightness, &fade_battery);
				}
				// holding both bright buttons save and exit colour
				if(held_tkey(&tkey_brightup) && held_tkey(&tkey_brightdown) && !(long_held_tkey(&tkey_brightdown)))
				{
					// save to eeprom
					eeprom_write_custom_colours(current_colour_palette_enum,&custom_colours);
					// change current colours
					switch (current_colour_palette_enum)
					{
					case default_colour_palette:
						change_current_cpalette(&default_colours);
						break;
					case colour_blind:
						change_current_cpalette(&colour_blind_colours);
						break;
					case custom:
						change_current_cpalette(&custom_colours);
						break;
					default:
						break;
					}
					state = STARTUP_ANIMATION;
				}

				// testing show current_colour_palette_enum value
				//fill_bar(HP_BAR_START,HP_BAR_START+3,current_colour_palette_enum,&hsv_health);
				break;
			case STARTUP_ANIMATION:
				
				// Draw bars
				fill_bar(HP_BAR_START, HP_BAR_END, startup_hp_value, &hsv_health);
				if (startup_hp_value > FULL_BAR_LENGTH)
					fill_bar(HP_BAR_START, HP_BAR_START - 1 + startup_hp_value - FULL_BAR_LENGTH, DEFAULT_SATURATION, &hsv_health_alt);
				
				fill_bar(EN_BAR_START, EN_BAR_END, startup_energy_value, &hsv_energy);
				if (startup_energy_value > FULL_BAR_LENGTH)
					fill_bar(EN_BAR_START, EN_BAR_START - 1 + startup_energy_value - FULL_BAR_LENGTH, DEFAULT_SATURATION, &hsv_energy_alt);
				
				fill_bar(DASH_BAR_START, DASH_BAR_END, startup_dash_value, &hsv_dash);
				if (startup_dash_value > DASH_BAR_LENGTH)
					fill_bar(DASH_BAR_START, DASH_BAR_START - 1 + startup_dash_value - DASH_BAR_LENGTH, DEFAULT_SATURATION, &hsv_dash_alt);

				// When all are at max, change state to RUNNING and reset animation
				if(startup_hp_value == hp_value && startup_energy_value == energy_value && startup_dash_value == dash_value)
				{
					state = RUNNING;
					startup_hp_value = 0;
					startup_energy_value = 0;
					startup_dash_value = 0;
				}
				break;
			default:
				break;
			}
			
			
			
			// Display led_data
			bitbang(LED_COUNT, led_data);
		}	// end main logic loop
    }	// end while
}
