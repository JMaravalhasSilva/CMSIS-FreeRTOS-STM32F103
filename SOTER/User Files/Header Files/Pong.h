#ifndef HEADER_FILES_PONG_H_
#define HEADER_FILES_PONG_H_

//Based on Jose Silva's Allegro Pong game (WIP)
//https://github.com/Kagehiko/Allegro_Pong

#include "FreeRTOS.h"

void gameInit(void);

void logic(float gx);

void render(TickType_t* previousWakeTime);

#endif
