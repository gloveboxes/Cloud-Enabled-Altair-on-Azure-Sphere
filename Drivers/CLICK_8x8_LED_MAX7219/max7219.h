#pragma once

#include <applibs/log.h>
#include <applibs/spi.h>
#include <stdbool.h>

#define MAX7219_REG_DECODE        0x09                        // "decode mode" register
#define MAX7219_REG_INTENSITY     0x0a                        // "intensity" register
#define MAX7219_REG_SCAN_LIMIT    0x0b                        // "scan limit" register
#define MAX7219_REG_SHUTDOWN      0x0c                        // "shutdown" register
#define MAX7219_REG_DISPLAY_TEST  0x0f                        // "display test" register

#define MAX7219_INTENSITY_MIN     0x00                        // minimum display intensity
#define MAX7219_INTENSITY_MAX     0x0f                        // maximum display intensity

typedef struct {
	SPI_InterfaceId interfaceId;
	SPI_ChipSelectId chipSelectId;
	uint32_t busSpeed;
	int handle;
	union {
		unsigned char bitmap[8];
		uint64_t bitmap64;
	};
	
} matrix8x8_t;

void max7219_clear(matrix8x8_t* panel8x8);
void max7219_display_test(matrix8x8_t* panel8x8, bool state);
void max7219_init(matrix8x8_t* panel8x8, unsigned char intialBrightness);
void max7219_panel_clear(matrix8x8_t* panel8x8);
void max7219_panel_write(matrix8x8_t* panel8x8);
void max7219_set_brightness(matrix8x8_t* panel8x8, unsigned char brightness);
void max7219_write(matrix8x8_t* panel8x8, unsigned char reg_number, unsigned char dataout);
