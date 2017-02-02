#include <LCDdriver.h>
#include "stm32f1xx.h"
#include "FreeRTOS.h"

#include "LowLevel.h"
#include "UserTasks.h"

void lcdIntroScreen(void){
	uint8_t i;

	lcd_draw_string(0.25*LCD_WIDTH,0.25*LCD_HEIGHT,"SOTER",0xFFFF,2);
	lcd_draw_string(0,0.65*LCD_HEIGHT,"Made by:",0xFFFF,1);
	lcd_draw_string(0,0.75*LCD_HEIGHT,"Daniel Freire",0xFFFF,1);
	lcd_draw_string(0,0.80*LCD_HEIGHT,"1130858@isep.ipp.pt",0xFFFF,1);
	lcd_draw_string(0,0.90*LCD_HEIGHT,"Jose Silva",0xFFFF,1);
	lcd_draw_string(0,0.95*LCD_HEIGHT,"1130352@isep.ipp.pt",0xFFFF,1);

	//Wait some time for the user to read screen
	for(i=0;i<7;i++){
		toggleDx(D1);
		lcd_delay_pool(5000);
	}

	for(i=0;i<7;i++){
		toggleDx(D2);
		lcd_delay_pool(5000);
	}

	for(i=0;i<7;i++){
		toggleDx(D3);
		lcd_delay_pool(5000);
	}

	//Disable all LEDs again
	toggleDx(D1);
	toggleDx(D2);
	toggleDx(D3);

	lcd_draw_fillrect(0,0,LCD_WIDTH,LCD_HEIGHT,0x0);
}


int main(void){

	initLowLevel(10,10,115200);

	lcd_init();

	if(systemWasResetByWatchdog()){

		lcd_draw_string(25,0,"System",RED,2);
		lcd_draw_string(20,20,"Failure",RED,2);
		lcd_draw_string(20,120,"Watchdog reset",WHITE,1);
		while(1);
	}

	startAllTasks();

	lcdIntroScreen();

	vTaskStartScheduler();

	//Will get stuck in this loop if there wasn't enough heap space for idle task
	while(1);
}
