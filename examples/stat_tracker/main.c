/* Hyper Light Drifter Stat Tracker firmware
Displays Health, Energy, and Dash points.
Manages buttons to increment points up and down

TODO:
[x] Fix main loop timing - main loop time intermittenly breaks and loops continously. millis
[] change retrigger to delay + repeat (need timer)
[x] cdc usb serial - if needed for debugging
[x] ADC
[x] Add timer for millis()
[] charging status
[x] change to HSV for LEDs (convert functions to use HSV)
[x] Touch buttons for brightness control
[x] FIX HSVtoRGB() not properly converting for saturation values above 128
[] Add color configuration states using multi button presses
[] Fix led showing garbage data on power on
[x] Fix battery icon showing wrong hue - ADC related? ADC and touch inputs were damaged from ESD. Series protection resitors were added
[] Add eeprom code to remember health states
[] Fix fill_bar_binary not working past 16 bits
[] Use interrupts for touch buttons if needed to save some time during the update loop (about 4 ms)

*/

//#define USB_ENABLE

#include <stdint.h>
#include <ch554.h>
#include <debug.h>
#include <bootloader.h>
#include <adc.h>
#include <bitbang.h>
#include <color_utils.h>
#include <timer.h>
#include <touch.h>

#ifdef USB_ENABLE

#include <usb-cdc.h>

#endif








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
#define BATT_3V3_ADC 168 // ADC count for 3.3V/5V * 255
#define BATT_3V9_ADC 199 // ADC count for 3.9V/5V * 255
#define BATT_MAX_ADC 214 // ADC count for 4.2V/5V * 255

// Timing defines
#define LOOP_PERIOD_MS 10 // Main loop timing
#define BATTERY_LOOP_PERIOD_MS 500 // Number of milliseconds for a period of fading from full to off

#define RED_HUE 0
#define GREEN_HUE 85
#define BLUE_HUE 170
#define MAX_BRIGHTNESS 60
#define DEFAULT_BRIGHTNESS 30
#define DEFAULT_SATURATION 255
#define BRIGHTNESS_STEP 5
#define MIN_BATT_ICON_BRIGHTNESS 10

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


#define LED_COUNT (46)

// States
#define MAX_STATES 7
#define BATTERY 1	//
#define RUNNING 2	//
#define COLOR_PALLETTE 3 		//
#define TOUCHKEY1 4 //
#define TOUCHKEY2 5 // 
#define TIMER 6 
#define LOOP 7

__xdata uint8_t led_data[LED_COUNT*3];
// button structure
typedef struct button
{
	uint8_t PREV;		// previous button state
	uint8_t ACTIVATED;	// transition to active state
	uint8_t DEACTIVATED;	// transition to deactivated state
	uint8_t ON; // on or off
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

#ifdef USB_ENABLE
// USB ISR
void DeviceInterrupt(void) __interrupt(INT_NO_USB) {
	usbInterrupt();
}
#endif

void update_buttons()
{
	__xdata static uint8_t repeat_timer = 0;
	
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
	// // if button is on
	HP_P_BTN.ON = !HP_BTN;
	EN_P_BTN.ON = !EN_BTN;
	DASH_P_BTN.ON = !DASH_BTN;

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
	HP_N_BTN.ON = !HP_BTN;
	EN_N_BTN.ON = !EN_BTN;
	DASH_N_BTN.ON = !DASH_BTN;
	
	MUL_N = 1; 	// float MUL_N now that we're done checking buttons
	

	// TODO: change this to use timers 
	// Increment the repeat timer if any button is being pressed
	// if (HP_P_BTN.PREV == 0 || HP_N_BTN.PREV == 0 || EN_P_BTN.PREV == 0 || EN_N_BTN.PREV == 0 || DASH_P_BTN.PREV == 0 || DASH_N_BTN.PREV == 0)
	// {
	// 	repeat_timer++;
		
	// 	// If a button has been held uint8_t enough
	// 	if (repeat_timer >= KEY_REPEAT_COUNT)
	// 	{
	// 		repeat_timer = 0;	// Reset counter
			
	// 		// Set all previous states to 1, so that any keys held down on next iteration think there was a falling edge
	// 		HP_P_BTN.PREV = 1;
	// 		HP_N_BTN.PREV = 1;
	// 		EN_P_BTN.PREV = 1;
	// 		EN_N_BTN.PREV = 1;
	// 		DASH_P_BTN.PREV = 1;
	// 		DASH_N_BTN.PREV = 1;
	// 	}
	// }
	// else
	// {
	// 	repeat_timer = 0;
	// }

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
// FIX: doesnt work past 16 bits ??
void fill_bar_binary(const uint8_t lsb_start, const uint32_t value, const uint8_t bits, const HsvColor *hsv)
{
	uint8_t i;
	for (i = 0; i < bits; i++)	// loop 16 times for 16 bits
	{
		// Is bit set?
		if ((uint32_t)(value & (1<<i)))	// LED ON
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

void set_battery_icon_led(const uint8_t batt_charge_adc)
{
	__xdata static HsvColor hsv_batt_icon = {0,DEFAULT_SATURATION,DEFAULT_BRIGHTNESS};
	hsv_batt_icon.v = brightness;
	__xdata static uint32_t prev_millis_batt = 0; // remove if not needed
	
	__xdata static uint8_t direction = 0; // 0 is down , 1 is up
	__xdata static const uint8_t steps = BATTERY_LOOP_PERIOD_MS/LOOP_PERIOD_MS;	// remove steps if not needed
	__xdata static uint8_t increments = BATTERY_LOOP_PERIOD_MS/LOOP_PERIOD_MS;	// initially starts high
	// Do stuff while battery is charging
	if(STAT == 0)	// If charging 
	{
		// TODO: LOOP LED on and off - colour is charge
		hsv_batt_icon.h = hsv_charging.h;
		// Increments every loop period
		// Starts high
		if(direction == 0)	// down
		{
			
			if(increments == 0)
			{
				direction = 1;	// go back up
			}
			else
			{
				increments--;
			}
		}
		else
		{
			if(increments >= BATTERY_LOOP_PERIOD_MS/LOOP_PERIOD_MS)
			{
				direction = 0;	// go back down
			}
			else
			{
				increments++;
			}
			
		}
		hsv_batt_icon.v = brightness * increments/(BATTERY_LOOP_PERIOD_MS/LOOP_PERIOD_MS);
		
	}
	else	// batt icon while not charging/ fully charged
	{
		// Show battery charge from red to green
		hsv_batt_icon.h = map(batt_charge_adc,BATT_MIN_ADC,BATT_MAX_ADC,RED_HUE,GREEN_HUE);
		// TODO: If battery critically low like 3.3V to 3.2V , flash red
	}	
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
	__xdata uint32_t current_millis = 0;	// TESTING
	__xdata uint8_t battery_adc_buffer[AVERAGING_SIZE];
	__xdata HsvColor hsv = {0,DEFAULT_SATURATION,DEFAULT_BRIGHTNESS};	// General Purpose hsv
	__xdata uint8_t errorFlag = 0;	// TESTING 
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
	HP_P_BTN.ON = 0;
	HP_N_BTN.ON = 0;
	EN_P_BTN.ON = 0;
	EN_N_BTN.ON = 0;
	DASH_P_BTN.ON = 0;
	DASH_N_BTN.ON = 0;


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
	BRIGHTNESS_UP_TKEY.NOKEY = getNokeyData(BRIGHTNESS_UP_TOUCHKEY_NUM);	// set no key threshold
	BRIGHTNESS_DOWN_TKEY.NOKEY = getNokeyData(BRIGHTNESS_DOWN_TOUCHKEY_NUM);	// set no key threshold

	// Initialize the adc buffer array with AVERAGING_SIZE good samples
	for(i = 0; i< AVERAGING_SIZE; i++)
	{
		ADC_START = 1;	// Start ADC conversion
		while(ADC_START == 1); // Wait for conversion done
		battery_adc_buffer[i] = ADC_DATA;// add to averaging Buffer
	}
	
	prev_millis = millis(); // set before going into loop
	// Main loop ************************************************************
    while (1) 
	{
		
		// Run loop every LOOP_PERIOD_MS
		current_millis = millis();
		
		if((prev_millis < current_millis) && current_millis-prev_millis >= LOOP_PERIOD_MS)
		{
			#ifdef USB_ENABLE 
			UsbCdc_puts("prev_millis:");	// TESTING
			UsbCdc_puti32(prev_millis);
			UsbCdc_puts(" millis():");
			UsbCdc_puti32(millis());
			UsbCdc_puts("\r\n");
			#endif

			prev_timing_millis = current_millis;	// TESTING timing check

			if(prev_millis > current_millis) // TESTING prev_millis should not be larger than current_millis at this point
			{
				errorFlag = 1;
			}
			else // normal
				prev_millis += LOOP_PERIOD_MS;
			update_buttons();
			#ifdef USB_ENABLE
			UsbCdc_processOutput();
			UsbCdc_processInput();
			#endif
			// Do other button stuff related to buttons at the same time
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
			if(BRIGHTNESS_UP_TKEY.DATA < BRIGHTNESS_UP_TKEY.NOKEY - TOUCH_HYSTERESIS && BRIGHTNESS_DOWN_TKEY.DATA < BRIGHTNESS_DOWN_TKEY.NOKEY - TOUCH_HYSTERESIS)
			{
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
			if(DASH_P_BTN.ACTIVATED)
			{
				if(state < MAX_STATES)
				{
					state++;
				}
			}
			if(DASH_N_BTN.ACTIVATED)
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
			}
			
			// Saturation Control (only for state COLOR_PALLETTE)
			if(state == COLOR_PALLETTE)
			{
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
			}

			
			
			
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


			clear_display();
			switch (state)
			{
			case BATTERY:
				// fill bar based on battery charge for now while testing
				hsv.h = hsv_health.h;
				hsv.s = DEFAULT_SATURATION;
				hsv.v = 10; // low brightness to not drain battery while charging
				fill_bar(HP_BAR_START, EN_BAR_END, map8bit(battery_charge_adc,BATT_MIN_ADC,BATT_MAX_ADC,0,FULL_BAR_LENGTH*2), &hsv);
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
				//fill_bar(HP_BAR_START,HP_BAR_END, map(BRIGHTNESS_DOWN_TKEY.NOKEY-TOUCH_HYSTERESIS,0,2^14, 0,FULL_BAR_LENGTH), &hsv_health);
				fill_bar_binary(HP_BAR_END,BRIGHTNESS_DOWN_TKEY.NOKEY,16,&hsv_health);
				// Display capacitance data
				//fill_bar(EN_BAR_START,EN_BAR_END, map(BRIGHTNESS_DOWN_TKEY.DATA,0,2^14, 0,FULL_BAR_LENGTH), &hsv_energy);
				fill_bar_binary(EN_BAR_END,BRIGHTNESS_DOWN_TKEY.DATA,16,&hsv_energy);

				set_led(DASH_BAR_START, &hsv_dash);

				// Turn on dash leds if touch data is less than nokey threshold
				if(BRIGHTNESS_DOWN_TKEY.DATA < BRIGHTNESS_DOWN_TKEY.NOKEY - TOUCH_HYSTERESIS)
				{
					set_led(DASH_BAR_END-1, &hsv_health);
				}
				if(BRIGHTNESS_UP_TKEY.DATA < BRIGHTNESS_UP_TKEY.NOKEY - TOUCH_HYSTERESIS)
				{
					set_led(DASH_BAR_END, &hsv_health);
				}
				break;
			case TOUCHKEY2:	// Debug data for TIN2 BRIGHTNESS_UP_TOUCHKEY_NUM
				// Display capacitance data values on hp and en bars
				//fill_bar(HP_BAR_START,HP_BAR_END, map(BRIGHTNESS_UP_TKEY.NOKEY-TOUCH_HYSTERESIS,0,2^14, 0,FULL_BAR_LENGTH), &hsv_health);
				fill_bar_binary(HP_BAR_END,BRIGHTNESS_UP_TKEY.NOKEY,16,&hsv_health);
				// Display capacitance data
				//fill_bar(EN_BAR_START,EN_BAR_END, map(BRIGHTNESS_UP_TKEY.DATA,0,2^14, 0,FULL_BAR_LENGTH), &hsv_energy);
				fill_bar_binary(EN_BAR_END,BRIGHTNESS_UP_TKEY.DATA,16,&hsv_energy);
				fill_bar(DASH_BAR_START,DASH_BAR_START + 1,2, &hsv_dash);	// turn on 2 dash leds to show state

				// Turn on dash leds if touch data is less than nokey threshold
				if(BRIGHTNESS_DOWN_TKEY.DATA < BRIGHTNESS_DOWN_TKEY.NOKEY - TOUCH_HYSTERESIS)
				{
					set_led(DASH_BAR_END-1, &hsv_health);
				}
				if(BRIGHTNESS_UP_TKEY.DATA < BRIGHTNESS_UP_TKEY.NOKEY - TOUCH_HYSTERESIS)
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
				//fill_bar(HP_BAR_START,HP_BAR_END,timing_millis,&hsv_dash);
				fill_bar_binary(HP_BAR_END,prev_millis,20,&hsv_dash);	// TESTING show 20 bits of prev_millis
				fill_bar_binary(EN_BAR_END,current_millis,20,&hsv_dash_alt);	// TESTING
				// fill_bar(DASH_BAR_START,DASH_BAR_END, map(timing_millis,0,20,0,5),&hsv_dash);
				break;
			default:
				break;
			}
			
			
			// Display battery icon
			set_battery_icon_led(battery_charge_adc);
			
			
			// Display led_data
			EA = 0; // Disable global interrupts
			bitbangWs2812(LED_COUNT, led_data);
			EA = 1;	// Enable global interrupts

			if(errorFlag)	// TESTING when loop error show millis and prev millis counts when error happened
			{

				fill_bar_binary(HP_BAR_END,prev_millis,20,&hsv_dash);	// TESTING show 20 bits of prev_millis
				fill_bar_binary(EN_BAR_END,current_millis,20,&hsv_dash_alt);
				hsv.h = RED_HUE; // ERROR red
				fill_bar(DASH_BAR_START,DASH_BAR_END, 255,&hsv);

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
