#ifndef MYLCDDRIVER_H_
#define MYLCDDRIVER_H_

#include "stm32f1xx.h"

//The LowLevel.h is just to enable the SET_GPIO_BIT and RESET_GPIO_BIT macros defined in there.
#include "LowLevel.h"

//Basic LCD colors
enum colors{
	RED=   0b11111,
	GREEN= 0b11111100000,
	BLUE = 0b1111100000000000,
	BLACK= 0x0,
	WHITE= 0xFFFF
};


#define ST7735_NOP 0x0
#define ST7735_SWRESET 0x01
#define ST7735_RDDID 0x04
#define ST7735_RDDST 0x09

#define ST7735_SLPIN  0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON  0x12
#define ST7735_NORON  0x13

#define ST7735_INVOFF 0x20
#define ST7735_INVON 0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON 0x29
#define ST7735_CASET 0x2A
#define ST7735_RASET 0x2B
#define ST7735_RAMWR 0x2C
#define ST7735_RAMRD 0x2E

#define ST7735_COLMOD 0x3A
#define ST7735_MADCTL 0x36


#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR 0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1 0xC0
#define ST7735_PWCTR2 0xC1
#define ST7735_PWCTR3 0xC2
#define ST7735_PWCTR4 0xC3
#define ST7735_PWCTR5 0xC4
#define ST7735_VMCTR1 0xC5

#define ST7735_RDID1 0xDA
#define ST7735_RDID2 0xDB
#define ST7735_RDID3 0xDC
#define ST7735_RDID4 0xDD

#define ST7735_PWCTR6 0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

#define LCD_WIDTH	128
#define LCD_HEIGHT	160





void lcd_delay_pool( uint16_t time2delay );
void lcd_init (void);
void lcd_send_commnad(  uint8_t txCommand );
void lcd_send_data( uint8_t txData );
uint8_t lcd_spi_send( uint8_t txByte );
void lcd_draw_string(uint8_t x, uint8_t y, char *c, uint16_t color, uint8_t size);
void lcd_draw_pixel(uint8_t x, uint8_t y, uint16_t color);
void lcd_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint16_t color);
void lcd_draw_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);
void lcd_draw_fillrect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);
void lcd_setAddrWindow( uint8_t x,uint8_t y,uint8_t x1,uint8_t y1 );
void lcd_draw_pixelFromChar(uint8_t x, uint8_t y, uint16_t color);
void lcd_draw_char(uint8_t x, uint8_t y, char c, uint16_t color, uint8_t size);
void lcd_draw_char2(uint8_t x, uint8_t y, char c, uint16_t color, uint8_t size);
void lcd_draw_circle(int16_t x0, int16_t y0, int16_t r, uint16_t color);

//New function :D
void lcd_draw_fillcircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);






#endif
