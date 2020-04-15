/* Hyper Light Drifter Stat Tracker firmware
Displays Health, Energy, and Dash points.
Manages buttons to increment points up and down

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
[] Add color configuration states using multi button presses
[] Fix led showing garbage data on power on
[x] Fix battery icon showing wrong hue - ADC related? ADC and touch inputs were damaged from ESD. Series protection resitors were added
[] Add eeprom code to remember health states
[x] Fix fill_bar_binary not working past 16 bits
[] Use interrupts for touch buttons if needed to save some time during the update loop (about 4 ms)
[x] Dim display when battery critically low
[x] Add flag for critically low batt so it doesnt toggle the flashing state
[x] Fix battery icon to not fade after constant voltage charged
*/

//#define USB_ENABLE

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


#ifdef USB_ENABLE

#include <usb-cdc.h>

#endif

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
#define BATT_3V3_ADC 168 // ADC count for 3.3V/5V * 255
#define BATT_3V9_ADC 199 // ADC count for 3.9V/5V * 255
#define BATT_MAX_ADC 214 // ADC count for 4.2V/5V * 255

// Timing defines
#define BATTERY_FADE_PERIOD_MS 500 // Number of milliseconds for a period of fading from full to off
#define CHARGED_TIMEOUT_MS 300000	// Time while at constant voltage to be considered fully charged
#define LOW_BATTERY_TIMEOUT_MS 10000	// Time below critically low battery before icon flashes
#define RETRIGGER_PERIOD 200
#define RETRIGGER_DELAY 500 	// Time button held down before

#define RED_HUE 0
#define GREEN_HUE 85
#define BLUE_HUE 170
#define DEFAULT_BRIGHTNESS 30
#define DEFAULT_SATURATION 255
#define BRIGHTNESS_MAX 255	// TESTING 255 - set to 60 afterwards
#define BRIGHTNESS_STEP 5
#define BRIGHTNESS_MIN 10
#define BRIGHTNESS_CRITICALLY_LOW 15

__xdata uint8_t brightness = DEFAULT_BRIGHTNESS; // default brightness
__xdata uint8_t saturation = DEFAULT_SATURATION;	// default saturation

// Colors {H,S,V} HSV values go from 0 to 255
__xdata HsvColor hsv_health = {110,DEFAULT_SATURATION,DEFAULT_BRIGHTNESS};
__xdata HsvColor hsv_health_alt = {167,DEFAULT_SATURATION,DEFAULT_BRIGHTNESS};
__xdata HsvColor hsv_energy = {235,DEFAULT_SATURATION,DEFAULT_BRIGHTNESS}; 
__xdata HsvColor hsv_energy_alt = {35,DEFAULT_SATURATION,DEFAULT_BRIGHTNESS};
__xdata HsvColor hsv_dash = {185,DEFAULT_SATURATION,DEFAULT_BRIGHTNESS};
__xdata HsvColor hsv_dash_alt = {210,DEFAULT_SATURATION,DEFAULT_BRIGHTNESS};
__xdata HsvColor hsv_charged = {GREEN_HUE,DEFAULT_SATURATION, DEFAULT_BRIGHTNESS};
__xdata HsvColor hsv_charging = {30, DEFAULT_SATURATION,DEFAULT_BRIGHTNESS};
__xdata HsvColor hsv_boot = {160,180,DEFAULT_BRIGHTNESS};
__xdata HsvColor hsv_off = {0,0,0};



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

// Button States
#define NOTACTIVATED 0
#define ACTIVATED 1
#define DEACTIVATED 2

#define LED_COUNT (46)

// States
#define MAX_STATES 8
#define BATTERY 1	//
#define RUNNING 2	//
#define COLOR_PALLETTE 3 		//
#define TOUCHKEY1 4 //
#define TOUCHKEY2 5 // 
#define TIMER 6 
#define LOOP 7
#define BUTTON 8

__xdata uint8_t led_data[LED_COUNT*3];

// struct for keeping track of fading
typedef struct fade
{
	uint8_t frames;	// total frames for fading 0 to max( fade_time/LOOP_PERIOD_MS)
	uint8_t direction;	// 0 is down 1 is Update
	uint16_t fade_time;	// time for 0 to max in ms
} fade;

__xdata fade fade_charging = {0,0,BATTERY_FADE_PERIOD_MS};
__xdata fade fade_criticallylowbatt = {0,0,BATTERY_FADE_PERIOD_MS/2};


typedef struct touchkey
{
	uint16_t NOKEY;		// data when no keypressed
	uint16_t DATA;		// current touch data value
	uint8_t PREV;		// previous state 
	uint8_t STATE;		// button STATE
} touchkey;

// button structs
__xdata button btn_hp_p = {0,0,0};
__xdata button btn_hp_n = {0,0,0};
__xdata button btn_en_p = {0,0,0};
__xdata button btn_en_n = {0,0,0};
__xdata button btn_dash_p = {0,0,0};
__xdata button btn_dash_n = {0,0,0};


__xdata touchkey tkey_brightup = {0,0,0,0};
__xdata touchkey tkey_brightdown = {0,0,0,0};

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
	
	// Clear tkey states in case they were active previously
	tkey_brightup.STATE = 0;
	tkey_brightdown.STATE = 0;
	
	MUL_P = 0;	// Drive MUL_P low to sense + buttons
	MUL_N = 1;	// Open MUL_N to let it float
	mDelayuS(5); // Give time to switch pins
	
	// Update + buttons
	update_button(&btn_hp_p,!HP_BTN);
	update_button(&btn_en_p,!EN_BTN);
	update_button(&btn_dash_p,!DASH_BTN);
	// if (btn_hp_p.PREV == 1 && HP_BTN == 0)
	// 	btn_hp_p.STATE = ACTIVATED;
	
	// if (btn_en_p.PREV == 1 && EN_BTN == 0)
	// 	btn_en_p.STATE = ACTIVATED;
	
	// if (btn_dash_p.PREV == 1 && DASH_BTN == 0)
	// 	btn_dash_p.STATE = ACTIVATED;
	
	// // Current states become previous states for next time
	// btn_hp_p.PREV = HP_BTN;
	// btn_en_p.PREV = EN_BTN;
	// btn_dash_p.PREV = DASH_BTN;

	// // If button is ON then increment ON once to represent 1 Loop period
	// if(!btn_hp_p)
	// 	btn_hp_p.ON++;
	// else
	// 	btn_hp_p.ON = 0;
	// if(!btn_en_p)
	// 	btn_en_p.ON++;
	// else
	// 	btn_en_p.ON = 0;
	// if(!btn_dash_p)
	// 	btn_dash_p.ON++;
	// else
	// 	btn_dash_p.ON = 0;

	MUL_P = 1;	// Open MUL_P to let it float	
	MUL_N = 0;	// Drive MUL_N low to sense - buttons
	mDelayuS(5);	// Give time to switch pins

	// Update - buttons
	update_button(&btn_hp_n,!HP_BTN);
	update_button(&btn_en_n,!EN_BTN);
	update_button(&btn_dash_n,!DASH_BTN);
	
	// // Check - buttons
	// if (btn_hp_n.PREV == 1 && HP_BTN == 0)
	// 	btn_hp_n.STATE = ACTIVATED;
	
	// if (btn_en_n.PREV == 1 && EN_BTN == 0)
	// 	btn_en_n.STATE = ACTIVATED;	
	
	// if (btn_dash_n.PREV == 1 && DASH_BTN == 0)
	// 	btn_dash_n.STATE = ACTIVATED;

	
	// // Current states become previous states for next time
	// btn_hp_n.PREV = HP_BTN;
	// btn_en_n.PREV = EN_BTN;
	// btn_dash_n.PREV = DASH_BTN;

	// // If button is ON then increment ON once to represent 1 Loop period
	// if(!btn_hp_n)
	// 	btn_hp_n.ON++;
	// else
	// 	btn_hp_n.ON = 0;
	// if(!btn_en_n)
	// 	btn_en_n.ON++;
	// else
	// 	btn_en_n.ON = 0;
	// if(!btn_dash_n)
	// 	btn_dash_n.ON++;
	// else
	// 	btn_dash_n.ON = 0;
	
	MUL_N = 1; 	// float MUL_N now that we're done checking buttons


	// // Retrigger button if held down longer than RETRIGGER_DELAY

	// // if button was held for longer than RETRIGGER_DELAY
	// // TODO: figure out way to track retrigger_period
	// if(btn_hp_p.ON > RETRIGGER_DELAY/LOOP_PERIOD_MS)
	// 	btn_hp_p.STATE = ACTIVATED;
	
	// Touchkeys

	// Start brightness up tkey query
	tkey_brightup.DATA = getTouchKeyData(BRIGHTNESS_UP_TOUCHKEY_NUM);
	// Data is inversely proportional to the capacitance. When touch button is pressed data is smaller than when not pressed.

	// Compare data to when key was no pressed
	if(tkey_brightup.PREV == 0 && tkey_brightup.DATA < tkey_brightup.NOKEY-TOUCH_HYSTERESIS) // button is pressed
	{
		tkey_brightup.STATE = ACTIVATED;
	}
	// Current state becomes previous state
	tkey_brightup.PREV = tkey_brightup.DATA < tkey_brightup.NOKEY -TOUCH_HYSTERESIS;

	// Start brightness down tkey query
	tkey_brightdown.DATA = getTouchKeyData(BRIGHTNESS_DOWN_TOUCHKEY_NUM);
	if(tkey_brightdown.PREV == 0 && tkey_brightdown.DATA < tkey_brightdown.NOKEY-TOUCH_HYSTERESIS) // button is pressed
	{
		tkey_brightdown.STATE = ACTIVATED;
	}
	tkey_brightdown.PREV = tkey_brightdown.DATA < tkey_brightdown.NOKEY -TOUCH_HYSTERESIS;

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
inline void clear_display()
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


// fade() function returns a brightness value from 0 to brightness based on time passed
// every loop all brightnesses that are fading ie charging, critically low batt, other health fading should be updated with this function
uint8_t fade_brightness(uint8_t max_brightness, fade *fade_var)
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
	return max_brightness * fade_var->frames/(fade_var->fade_time/LOOP_PERIOD_MS);
}

// Sets the battery icon based on charge value and whether it is charging via usb or not
// Call once per update main 
void update_battery_icon_led(const uint8_t batt_charge_adc)
{
	__xdata static HsvColor hsv_batt_icon = {0,DEFAULT_SATURATION,DEFAULT_BRIGHTNESS};
	__xdata static uint32_t batt_millis = 0; // Used for charging timing
	__xdata static uint8_t batt_charge_adc_flag = BATT_3V3_ADC + 1;	
	__xdata static uint8_t batt_constantvoltage_flag = 0;

	hsv_batt_icon.v = brightness;

	// Do stuff while battery is charging
	if(STAT == 0)	// If charging 
	{
		if(batt_charge_adc_flag != BATT_3V3_ADC + 1)
		{
				batt_charge_adc_flag = BATT_3V3_ADC + 1;	// Reset flag
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

		// if charging has NOT been in constant voltage mode for CHARGED_TIMEOUT_MS then fade batt icon
		if(batt_millis < CHARGED_TIMEOUT_MS)
		{
			// fade charging icon - TODO: add code to start animation when charging begins
			hsv_batt_icon.v = fade_brightness(brightness,&fade_charging);
		}
	}
	else // Not charging
	{
		batt_constantvoltage_flag = 0; // reset flag when not charging
		if(batt_charge_adc < batt_charge_adc_flag)	// if batt critically low
		{
			batt_charge_adc_flag = batt_charge_adc;	// Only allow adc flag to go lower (To stop noisy adc from triggering fading states)
		}

		if (batt_charge_adc_flag <= BATT_3V3_ADC)	// if less than 3.3v display critically low state
		{
			// fade brightness and fade faster based on battery
			flag_low_batt = 1; 
			brightness = map(batt_charge_adc_flag, BATT_MIN_ADC, BATT_3V3_ADC, BRIGHTNESS_MIN,BRIGHTNESS_CRITICALLY_LOW);
			fade_criticallylowbatt.fade_time = map(batt_charge_adc_flag, BATT_MIN_ADC, BATT_3V3_ADC,BATTERY_FADE_PERIOD_MS/4,BATTERY_FADE_PERIOD_MS);
			hsv_batt_icon.v = fade_brightness(brightness, &fade_criticallylowbatt);
		}
	}
	// Show battery charge from red to green
	hsv_batt_icon.h = map(batt_charge_adc,BATT_MIN_ADC,BATT_MAX_ADC,RED_HUE,GREEN_HUE);
	set_led(PWR_LED, &hsv_batt_icon);
}


void main() 
{
	__xdata uint8_t hp_value = 10;
	__xdata uint8_t energy_value = 10;
	__xdata uint8_t dash_value = 2;
	__xdata uint8_t state = RUNNING;	// Default State
	__xdata uint16_t battery_charge_adc = 0; // from 0 - 255 and also used for summing (need 16 bits)
	__xdata uint8_t buffer_index = 0;
	__xdata uint16_t i;	// General Purpose index
	__xdata uint32_t prev_millis = 0; // used for main loop
	__xdata uint32_t prev_timing_millis = 0;
	__xdata uint32_t timing_millis = 0;
	__idata uint32_t current_millis = 0;	// TESTING
	__xdata uint8_t battery_adc_buffer[AVERAGING_SIZE];
	__xdata HsvColor hsv = {0,DEFAULT_SATURATION,DEFAULT_BRIGHTNESS};	// General Purpose hsv
	__xdata uint8_t errorFlag = 0;	// TESTING 
	
	// Init tkey states
	tkey_brightup.STATE = 0;
	tkey_brightdown.STATE = 0;
	tkey_brightup.PREV = 0;		// active high
	tkey_brightdown.PREV = 0;	



	CfgFsys();
	bitbangSetup(); // ws2812 pin is setup here
    bitbangWs2812(LED_COUNT, led_data); // Blank out all leds to get rid of garbage data (not working)
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
	tkey_brightup.NOKEY = getNokeyData(BRIGHTNESS_UP_TOUCHKEY_NUM);	// set no key threshold
	tkey_brightdown.NOKEY = getNokeyData(BRIGHTNESS_DOWN_TOUCHKEY_NUM);	// set no key threshold

	// Initialize the adc buffer array with AVERAGING_SIZE good samples
	for(i = 0; i< AVERAGING_SIZE; i++)
	{
		ADC_START = 1;	// Start ADC conversion
		while(ADC_START == 1); // Wait for conversion done
		battery_adc_buffer[i] = ADC_DATA;// add to averaging Buffer
	}
	EA = 0; // disable global interrupts
	prev_millis = millis(); // set before going into loop
	EA = 1;	// enable global interrupts
	// Main loop ************************************************************
    while (1) 
	{
		
		// Run loop every LOOP_PERIOD_MS
		EA = 0; // disable global interrupts
		current_millis = millis();
		EA = 1; // enable global interrupts
		// (prev_millis < current_millis) && 
		if(current_millis-prev_millis >= LOOP_PERIOD_MS)
		{
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

			prev_timing_millis = current_millis;	// TESTING timing check

			if(prev_millis > current_millis) // TESTING prev_millis should not be larger than current_millis at this point
			{
				errorFlag = 1;
			}
			// else if(current_millis - prev_millis >= 2 * LOOP_PERIOD_MS) // it if ever goes out of sync by more than 2 periods
			// 	errorFlag = 2;
			else // Normal - no errors
				prev_millis += LOOP_PERIOD_MS;

			update_buttons(); 

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
			for(i = 0; i < AVERAGING_SIZE; i++)
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

			// Do other button stuff related to buttons at the same time
			// Brightness control
			if(tkey_brightup.STATE == ACTIVATED && !flag_low_batt)
			{
			// Increment value if <= BRIGHTNESS_MAX - BRIGHTNESS_STEP and not low battery
				if (brightness <= BRIGHTNESS_MAX - BRIGHTNESS_STEP)
				{
					brightness += BRIGHTNESS_STEP;
				}
			}
			if(tkey_brightdown.STATE == ACTIVATED && !flag_low_batt)
			{
				// Decrement value if >= 0 + BRIGHTNESS_STEP and not low battery
				if( brightness > BRIGHTNESS_STEP)
				{
					brightness -= BRIGHTNESS_STEP;
				}
			}
			if(tkey_brightup.DATA < tkey_brightup.NOKEY - TOUCH_HYSTERESIS && tkey_brightdown.DATA < tkey_brightdown.NOKEY - TOUCH_HYSTERESIS)
			{// TESTING - goes to boot loader when both tkey buttons pressed
				// Turn off WS2812's before starting bootloader
				for (i = 0; i < LED_COUNT*3; i++)
					led_data[i] = 0;
				
				// Light the power LED to indicate boot mode
				set_led(PWR_LED, &hsv_boot);


				EA = 0;	// Disable global interrupts while bitbanging
				bitbangWs2812(LED_COUNT, led_data);
				

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
			hsv_charged.v = brightness;
			hsv_charging.v = brightness;
			hsv_boot.v = brightness;
			hsv.v = brightness;

			// TESTING using dash buttons to switch between states
			if(retrigger(&btn_dash_p))
			{
				if(state < MAX_STATES)
				{
					state++;
				}
			}
			if(retrigger(&btn_dash_n))
			{
				if(state > BATTERY)
				{
					state--;
				}
			}

			// Adjust stats in RUNNING state
			if(state == RUNNING)
			{
				// clamp to 2 * full_bar_length for overfill
				if (hp_value < 2*(FULL_BAR_LENGTH) && retrigger(&btn_hp_p))
					hp_value++;
				else if (hp_value > 0 && retrigger(&btn_hp_n))
					hp_value--;
				
				if (energy_value < 2*(FULL_BAR_LENGTH) && retrigger(&btn_en_p))
					energy_value++;
				else if (energy_value > 0 && retrigger(&btn_en_n))
					energy_value--;		
				
				if (dash_value < 2*(DASH_BAR_LENGTH) && retrigger(&btn_dash_p))
					dash_value++;
				else if (dash_value > 0 && retrigger(&btn_dash_n))
					dash_value--;
			}
			
			// Saturation Control (only for state COLOR_PALLETTE)
			if(state == COLOR_PALLETTE)
			{
				if(retrigger(&btn_hp_p))
				{
					// Increment saturation if less than 255
					if (hsv.s < 251)
					{
						hsv.s += 5;
					}
					
				}
				if(retrigger(&btn_hp_n))
				{
					// Decrement saturation if more than 0
					if(hsv.s > 4)
					{
						hsv.s -= 5;
					}
				}
			}

			clear_display();
			switch (state)
			{
			case BATTERY:
				// fill bar based on battery charge for now while testing
				hsv.h = hsv_health.h;
				hsv.s = DEFAULT_SATURATION;
				fill_bar(HP_BAR_START, HP_BAR_END, map8bit(battery_charge_adc,BATT_MIN_ADC,BATT_MAX_ADC,0,FULL_BAR_LENGTH*2), &hsv);
				hsv.h = hsv_health.h + 10;
				fill_bar_binary(EN_BAR_END,battery_charge_adc,8,&hsv);
				if(STAT == 0)
					set_led(DASH_BAR_END,&hsv_charging);
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
			case COLOR_PALLETTE:	// Demo of all the hues - Increments of 5
				hsv.v = brightness;
				for(i = 0; i < LED_COUNT; i++)
				{
					// go through leds and change hsv for each
					hsv.h = 5*i;
					set_led(i+1,&hsv);
				}
				break;
			case TOUCHKEY1: // Debug data for TIN1 BRIGHTNESS_DOWN_TOUCHKEY_NUM
				// Display nokey data
				fill_bar_binary(HP_BAR_END,tkey_brightdown.NOKEY,16,&hsv_health);
				// Display capacitance data
				fill_bar_binary(EN_BAR_END,tkey_brightdown.DATA,16,&hsv_energy);

				set_led(DASH_BAR_START, &hsv_dash);

				// Turn on dash leds if touch data is less than nokey threshold
				if(tkey_brightdown.DATA < tkey_brightdown.NOKEY - TOUCH_HYSTERESIS)
				{
					set_led(DASH_BAR_END-1, &hsv_health);
				}
				if(tkey_brightup.DATA < tkey_brightup.NOKEY - TOUCH_HYSTERESIS)
				{
					set_led(DASH_BAR_END, &hsv_health);
				}
				break;
			case TOUCHKEY2:	// Debug data for TIN2 BRIGHTNESS_UP_TOUCHKEY_NUM
				// Display capacitance data values on hp and en bars
				fill_bar_binary(HP_BAR_END,tkey_brightup.NOKEY,16,&hsv_health);
				// Display capacitance data
				fill_bar_binary(EN_BAR_END,tkey_brightup.DATA,16,&hsv_energy);
				fill_bar(DASH_BAR_START,DASH_BAR_START + 1,2, &hsv_dash);	// turn on 2 dash leds to show state

				// Turn on dash leds if touch data is less than nokey threshold
				if(tkey_brightdown.DATA < tkey_brightdown.NOKEY - TOUCH_HYSTERESIS)
				{
					set_led(DASH_BAR_END-1, &hsv_health);
				}
				if(tkey_brightup.DATA < tkey_brightup.NOKEY - TOUCH_HYSTERESIS)
				{
					set_led(DASH_BAR_END, &hsv_health);
				}
				break;
			case TIMER:
				// display millis
				hsv.h = BLUE_HUE;
				
				// // Display 1/100ths of seconds in 1-10 leds
				// fill_bar(HP_BAR_START, HP_BAR_START + 9,map(millis()/10%10+1,0,10,0,10),&hsv);
				// // Display 1/10ths of seconds in 11-20
				// fill_bar(HP_BAR_START + 10, HP_BAR_END,map(millis()/100%10+1,0,10,0,10),&hsv);
				// // Display seconds in 21-30
				// fill_bar(EN_BAR_START, EN_BAR_START + 9,map(millis()/1000%10+1,0,10,0,10),&hsv);
				// // Display decaseconds in 31-40
				// fill_bar(EN_BAR_START + 10, EN_BAR_END,map(millis()/10000%10+1,0,10,0,10),&hsv);


				fill_bar_binary(EN_BAR_END,current_millis,32,&hsv);	// testing try 0b100000000000000000 anything above 16 bits is truncated?
				break;
			case LOOP:
				// loop counter display
				fill_bar_binary(HP_BAR_END,prev_millis,20,&hsv_dash);	// TESTING show 20 bits of prev_millis
				fill_bar_binary(EN_BAR_END,current_millis,20,&hsv_dash_alt);	// TESTING
				break;
			case BUTTON:
				fill_bar_binary(HP_BAR_END,btn_dash_p.prev_loops,20,&hsv_dash);
				fill_bar_binary(EN_BAR_END,btn_dash_p.state,8,&hsv_dash);
				if(retrigger(&btn_dash_n))
					set_led(DASH_BAR_END-1,&hsv_dash);
				else
					set_led_off(DASH_BAR_END-1);
				if(retrigger(&btn_dash_p))
					set_led(DASH_BAR_END,&hsv_dash);
				else
					set_led_off(DASH_BAR_END);
				break;
			default:
				break;
			}
			
			
			// Update battery icon
			update_battery_icon_led(battery_charge_adc);
			
			
			// Display led_data
			EA = 0; // Disable global interrupts
			bitbangWs2812(LED_COUNT, led_data);
			EA = 1;	// Enable global interrupts

			if(errorFlag)	// TESTING when loop error show millis and prev millis counts when error happened
			{
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

				fill_bar_binary(HP_BAR_END,prev_millis,20,&hsv_dash);	// show 20 bits of prev_millis
				fill_bar_binary(EN_BAR_END,current_millis,20,&hsv_dash_alt);
				
				
				hsv.h = RED_HUE; // ERROR red
				fill_bar_binary(DASH_BAR_END, errorFlag,DASH_BAR_END-DASH_BAR_START+1,&hsv);

				EA = 0; // Disable global interrupts
				bitbangWs2812(LED_COUNT, led_data);
				EA = 1;
				while(1);	// loop forever during error and look at data

			}
				

			#ifndef USB_ENABLE
			if (BOOT == 1) // cant use BOOT_PB with usb cdc
				break;
			#endif
		}	// end main logic loop
		timing_millis = millis() - prev_timing_millis; // TESTING timing
    }	// end while
	
	// Turn off WS2812's before starting bootloader
	clear_display();
	
	// Light the power LED to indicate boot mode
	set_led(PWR_LED, &hsv_boot);


	EA = 0;	// Disable global interrupts while bitbanging
	bitbangWs2812(LED_COUNT, led_data);
	

	mDelaymS(100); // delay for bootloader
	bootloader();
	while(1);
	
}
