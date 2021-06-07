#include "max7219.h"

void max7219_write(matrix8x8_t* panel8x8, unsigned char reg_number, unsigned char dataout)
{
	if (panel8x8->handle == -1) {
		return;
	}

	unsigned char data[2];
	SPIMaster_Transfer transfer;

	data[0] = reg_number;
	data[1] = dataout;

	SPIMaster_InitTransfers(&transfer, 1);
	transfer.flags = SPI_TransferFlags_Write;
	transfer.length = 2;
	transfer.writeData = data;

	if (SPIMaster_TransferSequential(panel8x8->handle, &transfer, 1) != 2) {
		Log_Debug("SPI Write Failed");
	}
}

void max7219_set_brightness(matrix8x8_t* panel8x8, unsigned char brightness)
{
	brightness &= 0x0f;                                 // mask off extra bits
	max7219_write(panel8x8, MAX7219_REG_INTENSITY, brightness);           // set brightness
}


void max7219_display_test(matrix8x8_t* panel8x8, bool state)
{
	max7219_write(panel8x8, MAX7219_REG_DISPLAY_TEST, state ? 1 : 0);                 // put MAX7219 into "display test" mode
}


void max7219_clear(matrix8x8_t* panel8x8)
{
	char i;
	for (i = 1; i < 9; i++)
		max7219_write(panel8x8, i, 0x00);                           // turn all segments off
}

void max7219_panel_write(matrix8x8_t* panel8x8)
{
	for (unsigned char i = 0; i < sizeof(panel8x8->bitmap); i++) {
		max7219_write(panel8x8, (unsigned char)(i + 1), panel8x8->bitmap[i]);
	}
}

void max7219_panel_clear(matrix8x8_t* panel8x8)
{
	for (size_t i = 0; i < sizeof(panel8x8->bitmap); i++) {
		panel8x8->bitmap[i] = 0;
	}

	max7219_panel_write(panel8x8);
}

void max7219_init(matrix8x8_t* panel8x8, unsigned char intialBrightness)
{
	SPIMaster_Config max7219Config;

	SPIMaster_InitConfig(&max7219Config);
	max7219Config.csPolarity = SPI_ChipSelectPolarity_ActiveLow;

	panel8x8->handle = SPIMaster_Open(panel8x8->interfaceId, panel8x8->chipSelectId, &max7219Config);

	SPIMaster_SetBusSpeed(panel8x8->handle, panel8x8->busSpeed);
	SPIMaster_SetBitOrder(panel8x8->handle, SPI_BitOrder_MsbFirst);
	SPIMaster_SetMode(panel8x8->handle, SPI_Mode_0);

	max7219_set_brightness(panel8x8, intialBrightness);							// set to maximum intensity
	max7219_display_test(panel8x8, false);							// disable test mode

	max7219_write(panel8x8, MAX7219_REG_SCAN_LIMIT, 7);                   // set up to scan all eight digits
	max7219_write(panel8x8, MAX7219_REG_DECODE, 0x00);                    // set to "no decode" for all digits
	max7219_write(panel8x8, MAX7219_REG_SHUTDOWN, 1);                     // put MAX7219 into "normal" mode
}