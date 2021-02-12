#include <stdint.h>

#include <ch554.h>
#include "bitbang.h"

// Bitbang.c is used to bitbang the addressable led protocol for the ch55x @ 16 Mhz
// Setup: In main, please set up the SBIT with the LED label for your pin. eg.
// #define LED_PIN 2	// addressable led data pin
// SBIT(LED, PORT3, LED_PIN); // LED is used in bitbang.c
// also include which led you want to use eg. for sk6812_2020
// #define bitbang(led_count,led_data)	bitbangSk6812_2020(led_count, led_data)	// choosing correct led (see bitbang.h)
// call the setup function
// bitbangSetup(3,LED_PIN); // using 3.2 pin as led pin
//

// Choose port 3 or 1 and pin to use
void bitbangSetup(uint8_t port, uint8_t pin)
{
    // Configure Pin 3.1 as GPIO output
    if(port == 3)
    {
        P3_MOD_OC &= ~(1<<pin); // Set Push pull output mode
        P3_DIR_PU |= (1<<pin);  // Set to output mode
    }
    else if (port == 1)
    {
        P1_MOD_OC &= ~(1<<pin); // Set Push pull output mode
        P1_DIR_PU |= (1<<pin);  // Set to output mode
    }
    
}

void bitbangWs2812( uint8_t ledCount, __xdata uint8_t * ledData )
{
    EA = 0; // Disable global interrupts
    ledCount;
    ledData;

    // Bitbang routine
    // Input parameters: (determined by compilation)
    // * byteCount should be allocated in dpl
    // * ledData should be allocated with name '_bitbangWs2812_PARAM_2'

    // Strategy:
    // * Keep the data memory pointer in DPTR
    // * Keep ledCount in r2
    // * Store bitCount in r3
    // * Store the current data variable in ACC

    //  TODO: What context needs to be pushed/popped?

    __asm

    mov r2, dpl             ; Load the LED count into r2

    mov dpl, _bitbangWs2812_PARM_2  ; Load the LED data start address into DPTR
    mov dph, (_bitbangWs2812_PARM_2 + 1)

    00001$:                 ; byte loop

// Red byte
        movx a,@dptr        ; Load the current LED data value into the accumulator (1)
        inc dptr            ; and advance the counter for the next LED data value (1)

        mov r3, #8          ; Set up the bit loop (2)
    00002$:                 ; red bit loop
        setb _LED           ; Begin bit cycle- set bit high (2)

        nop                 ; Tune this count by hand, want ~.4uS (1*2)
        nop

        rlc A               ; Shift the LED data value left to get the high bit (1)
        mov _LED, C         ; Set the output bit high if the current bit is high, (2)
                            ; otherwise set it low

        nop                 ; Tune this count by hand, want ~.4uS (1*2)
        nop
        nop

        clr _LED            ; final part of bit cycle, set bit low (2)

//        nop                 ; Tune this count by hand, want ~.45uS
//        nop

        djnz r3, 00002$     ; If there are more bits in this byte (2, ?)

// Green byte
        movx a,@dptr        ; Load the current LED data value into the accumulator (1)
        inc dptr            ; and advance the counter for the next LED data value (1)

        mov r3, #8          ; Set up the bit loop (2)
    00003$:                 ; green bit loop
        setb _LED           ; Begin bit cycle- set bit high (2)

        nop                 ; Tune this count by hand, want ~.4uS (1*2)
        nop
        

        rlc A               ; Shift the LED data value left to get the high bit (1)
        mov _LED, C         ; Set the output bit high if the current bit is high, (2)
                            ; otherwise set it low

        nop                 ; Tune this count by hand, want ~.4uS (1*2)
        nop
        nop

        clr _LED            ; final part of bit cycle, set bit low (2)

//        nop                 ; Tune this count by hand, want ~.45uS
//        nop

        djnz r3, 00003$     ; If there are more bits in this byte (2, ?)

// Blue byte
        movx a,@dptr        ; Load the current LED data value into the accumulator (1)
        inc dptr            ; and advance the counter for the next LED data value (1)

        mov r3, #8          ; Set up the bit loop (2)
    00004$:                 ; blue bit loop
        setb _LED           ; Begin bit cycle- set bit high (2)

        nop                 ; Tune this count by hand, want ~.4uS (1*2)
        nop

        rlc A               ; Shift the LED data value left to get the high bit (1)
        mov _LED, C         ; Set the output bit high if the current bit is high, (2)
                            ; otherwise set it low

        nop                 ; Tune this count by hand, want ~.4uS (1*2)
        nop
        nop

        clr _LED            ; final part of bit cycle, set bit low (2)

//        nop                 ; Tune this count by hand, want ~.45uS
//        nop

        djnz r3, 00004$     ; If there are more bits in this byte (2, ?)


        djnz r2, 00001$     ; If there are more LEDs (2, ?)

    __endasm;
    EA = 1; // enable global interrupts
}


// Modified to be compatible with sk6812-2020,KTR1010 and potentially other 2020 small addressable leds. (not tested on ws2812-2020 which have a different T1L max)
// T0H = 320ns
// T0L = 944ns
// T1H = 688ns
// T1L = 560ns
// FMHZ = 12 Mhz
void bitbangSk6812_2020( uint8_t ledCount, __xdata uint8_t * ledData )
{
    EA = 0; // Disable global interrupts
    ledCount;
    ledData;
	

    // Bitbang routine
    // Input parameters: (determined by compilation)
    // * byteCount should be allocated in dpl
    // * ledData should be allocated with name '_bitbangWs2812_PARAM_2'

    // Strategy:
    // * Keep the data memory pointer in DPTR
    // * Keep ledCount in r2
    // * Store bitCount in r3
    // * Store the current data variable in ACC

    //  TODO: What context needs to be pushed/popped?

    __asm

    mov r2, dpl             ; Load the LED count into r2

    mov dpl, _bitbangWs2812_PARM_2  ; Load the LED data start address into DPTR
    mov dph, (_bitbangWs2812_PARM_2 + 1)

    00001$:                 ; byte loop

// Red byte
        movx a,@dptr        ; Load the current LED data value into the accumulator (1)
        inc dptr            ; and advance the counter for the next LED data value (1)

        mov r3, #8          ; Set up the bit loop (2)
    00002$:                 ; red bit loop
        setb _LED           ; Begin bit cycle- set bit high (2)

        nop                 ; Tune this count by hand, want ~.4uS (1*2)
        nop

        rlc A               ; Shift the LED data value left to get the high bit (1)
        mov _LED, C         ; Set the output bit high if the current bit is high, (2)
                            ; otherwise set it low

        nop                 ; Tune this count by hand, want ~.4uS (1*2)
        nop
        nop
		nop
		
        

        clr _LED            ; final part of bit cycle, set bit low (2)

        nop                 ; Tune this count by hand, want ~.45uS
        nop
		nop

        djnz r3, 00002$     ; If there are more bits in this byte (2, ?)

// Green byte
        movx a,@dptr        ; Load the current LED data value into the accumulator (1)
        inc dptr            ; and advance the counter for the next LED data value (1)

        mov r3, #8          ; Set up the bit loop (2)
    00003$:                 ; green bit loop
        setb _LED           ; Begin bit cycle- set bit high (2)

        nop                 ; Tune this count by hand, want ~.4uS (1*2)
        nop
		
        

        rlc A               ; Shift the LED data value left to get the high bit (1)
        mov _LED, C         ; Set the output bit high if the current bit is high, (2)
                            ; otherwise set it low

        nop                 ; Tune this count by hand, want ~.4uS (1*2)
        nop
        nop
		nop
        

        clr _LED            ; final part of bit cycle, set bit low (2)

        nop                 ; Tune this count by hand, want ~.45uS
        nop
		nop

        djnz r3, 00003$     ; If there are more bits in this byte (2, ?)

// Blue byte
        movx a,@dptr        ; Load the current LED data value into the accumulator (1)
        inc dptr            ; and advance the counter for the next LED data value (1)

        mov r3, #8          ; Set up the bit loop (2)
    00004$:                 ; blue bit loop
        setb _LED           ; Begin bit cycle- set bit high (2)

        nop                 ; Tune this count by hand, want ~.4uS (1*2)
        nop

        rlc A               ; Shift the LED data value left to get the high bit (1)
        mov _LED, C         ; Set the output bit high if the current bit is high, (2)
                            ; otherwise set it low

        nop                 ; Tune this count by hand, want ~.4uS (1*2)
        nop
        nop
		nop


        clr _LED            ; final part of bit cycle, set bit low (2)

        nop                 ; Tune this count by hand, want ~.45uS
        nop
		nop

        djnz r3, 00004$     ; If there are more bits in this byte (2, ?)


        djnz r2, 00001$     ; If there are more LEDs (2, ?)

    __endasm;
    EA = 1; // Enable global interrupts
}