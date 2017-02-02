#ifndef USERTASKS_H_
#define USERTASKS_H_

#include <LCDdriver.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "LowLevel.h"

enum taskenum{
	DebugLEDTaskEnum,
	I2CEnum,
	USARTSendEnum,
	UIEnum,
	PongEnum,
	NumberOfTasks
};

extern unsigned short task_stacks[NumberOfTasks];

TaskHandle_t task_handlers[NumberOfTasks];


void startAllTasks(void);


#endif
