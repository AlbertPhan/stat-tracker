TARGET = stattracker

CPU_FREQ = 16000000

# Adjust the XRAM location and size to leave space for the USB DMA buffers
# Buffer layout in XRAM:
# 0x0000 Ep0Buffer[8]
# 0x0040 Ep1Buffer[8]
# 0x0080 EP2Buffer[2*64]
#
# This takes a total of 256bytes, so there are 768 bytes left.
XRAM_SIZE = 0x0300
XRAM_LOC = 0x0100

C_FILES = \
	main.c \
	../../include/color_utils.c \
	../../include/bitbang.c \
	../../include/adc.c \
	../../include/debug.c\
	../../include/timer.c\
	../../include/touch.c\
	#../../include/usb-cdc.c\
	../../include/hardware.c\

ASM_FILES = \
	bitbang_asm.asm

pre-flash:
	

include ../Makefile.include
