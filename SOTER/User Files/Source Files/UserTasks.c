#include "UserTasks.h"
#include "Pong.h"
#include <stdio.h>


//Static function prototypes
static void debugLED(void);
static void USARTSend(void);
static void I2C(void);
static void UI(void);
static void Pong(void);

//File-scope typedef
typedef struct I2C_data{
	float gx,gy,gz;
	uint32_t timestamp;
}I2C_data_type;

//File-scope Queues
QueueHandle_t I2CtoUSARTQueue,I2CtoPongQueue;

//Global-scope task stacks
unsigned short task_stacks[NumberOfTasks]={
		configMINIMAL_STACK_SIZE,		//DebugLEDTask
		configMINIMAL_STACK_SIZE,		//I2CTask
		configMINIMAL_STACK_SIZE+100,	//USARTSend
		configMINIMAL_STACK_SIZE+62,	//UI
		configMINIMAL_STACK_SIZE+100	//Pong
};

void startAllTasks(void){

	I2CtoUSARTQueue=xQueueCreate(10,sizeof(I2C_data_type));
	I2CtoPongQueue=xQueueCreate(1,sizeof(float));

	//I2C and USARTSend have higher priorities to make sure the data is always logged.
	//Pong never runs at the same time as UI, and has higher priority than debugLED to keep up the framerate
	//UI has the same priority as debugLED because the screen is only updated at 1Hz
	xTaskCreate((void*)debugLED,	"debugLED",		task_stacks[DebugLEDTaskEnum],	NULL,	tskIDLE_PRIORITY+1,		&task_handlers[DebugLEDTaskEnum]);
	xTaskCreate((void*)I2C,			"I2C",			task_stacks[I2CEnum],			NULL,	tskIDLE_PRIORITY+3,		&task_handlers[I2CEnum]);
	xTaskCreate((void*)USARTSend,	"USARTSend",	task_stacks[USARTSendEnum],		NULL,	tskIDLE_PRIORITY+3,		&task_handlers[USARTSendEnum]);
	xTaskCreate((void*)UI,			"UI",			task_stacks[UIEnum],			NULL,	tskIDLE_PRIORITY+1,		&task_handlers[UIEnum]);
	xTaskCreate((void*)Pong,		"Pong",			task_stacks[PongEnum],			NULL,	tskIDLE_PRIORITY+2,		&task_handlers[PongEnum]);
}


//Idle hook
void vApplicationIdleHook( void ){
	//Wait for interrupt to wake up
	__WFI();
}


static void debugLED(void){

	while(1){
		vTaskDelay(1000/portTICK_PERIOD_MS);
		toggleDx(D1);
	}
}

//#pragma GCC push_options
//#pragma GCC optimize ("O0")

static void UI(void){
	//Note: Using static variables frees up stack space
	static TickType_t previousWakeTime;
	static char buffer[40];
	static uint8_t i;

	previousWakeTime=xTaskGetTickCount();

	flushSwitchAction();

	while(1){

		//Print queues to the screen
		lcd_draw_string(0,0,"Messages in queues:",WHITE,1);
		sprintf(buffer,"I2CtoUSARTQueue: %lu  ",uxQueueMessagesWaiting(I2CtoUSARTQueue));
		lcd_draw_string(0,10,buffer,WHITE,1);
		sprintf(buffer,"I2CtoPongQueue: %lu  ",uxQueueMessagesWaiting(I2CtoPongQueue));
		lcd_draw_string(0,20,buffer,WHITE,1);


		//Print all tasks to the screen
		lcd_draw_string(0,40,"Worst free stack:",0xFFFF,1);
		for(i=0;i<NumberOfTasks;i++){
			sprintf(buffer,"%s:%lu of %hu ",pcTaskGetName(task_handlers[i]),uxTaskGetStackHighWaterMark(task_handlers[i]),task_stacks[i]*2);
			lcd_draw_string(0,50+i*10,buffer,WHITE,1);
		}

		//"Poll" 4 times for the switches and wait a total for 1000ms
		//This was necessary to make the transition to pong smoother, as only checking once very 1 second felt very slow
		for(i=0;i<4;i++){

			vTaskDelayUntil(&previousWakeTime, 250/portTICK_PERIOD_MS);

			if(getSwitchAction(0)==CenterEnum){
				//Switch out of this task and enter the Pong task
				lcd_draw_fillrect(0,0,LCD_WIDTH,LCD_HEIGHT,BLACK);
				vTaskResume(task_handlers[PongEnum]);
				vTaskSuspend(NULL);

				//If the code gets here, pong has selected this task to run
				vTaskDelay(100/portTICK_PERIOD_MS);
				flushSwitchAction();

				//Get new execution time
				previousWakeTime=xTaskGetTickCount();
				break;
			}
		}
		//Draw this string here, for extra coolness factor
		lcd_draw_string(0,120,"Press SW5 for PONG!",WHITE,1);
	}
}

//#pragma GCC pop_options

static void USARTSend(void){
	static TickType_t previousWakeTime;
	//"X=0.2f" will have maximum bytes when "X=-2.00g" is sent. That will be 8bytes for each axis
	//3 bytes for the 3 "|" characters
	//"TSTAMP=" is 7 bytes ling
	//10 bytes for timestamp maximum number of 4294967295
	//1 byte for the \n
	//45 bytes total + 1 byte for NULL char \0 gives 46bytes total
	//4 extra bytes "just in case" = 46+4=50
	static char buffer[50];
	static I2C_data_type I2Cdata;

	previousWakeTime=xTaskGetTickCount();

	while(1){
		xQueueReceive(I2CtoUSARTQueue,&I2Cdata,portMAX_DELAY);
		sprintf(buffer,"X=%0.2fg|Y=%0.2fg|Z=%0.2fg|TSTAMP=%ld\n",I2Cdata.gx,I2Cdata.gy,I2Cdata.gz,I2Cdata.timestamp);
		USARTPutString(buffer);

		//Adding a slight offset between the USART wait and I2C wait will increase the I2CtoUSART queue slowly.
		vTaskDelayUntil(&previousWakeTime,50/portTICK_PERIOD_MS);
	}
}

static void I2C(void){
	static TickType_t previousWakeTime;
	static I2C_data_type I2Cdata;

	previousWakeTime=xTaskGetTickCount();

	startupAccel();

	watchdogStart();

	while(1){

		getAccelData(&I2Cdata.gx,&I2Cdata.gy,&I2Cdata.gz);
		I2Cdata.timestamp=xTaskGetTickCount();

		//Flush old data if the USART failed to send the data
		if(xQueueSendToBack(I2CtoUSARTQueue,&I2Cdata,0)!=pdPASS){
			xQueueReset(I2CtoUSARTQueue);
			xQueueSendToBack(I2CtoUSARTQueue,&I2Cdata,0);
		}

		xQueueOverwrite(I2CtoPongQueue,&I2Cdata.gx);

		//Reset watchdog before 1 second passes
		watchdogReset();
		//Get data at a 20Hz rate
		vTaskDelayUntil(&previousWakeTime,50/portTICK_PERIOD_MS);
	}
}

static void Pong(void){
	static TickType_t previousWakeTime;
	static float gx=0;

	//Self-suspend to let UI run
	vTaskSuspend(NULL);

	//Get first tick count
	previousWakeTime=xTaskGetTickCount();

	gameInit();

	while(1){

		xQueueReceive(I2CtoPongQueue,&gx,0);

		logic(gx);

		render(&previousWakeTime);

		//50ms = 20FPS
		vTaskDelayUntil(&previousWakeTime,50/portTICK_PERIOD_MS);

		//State transition
		if(getSwitchAction(0)==CenterEnum){
			//Switch back to the UI task
			lcd_draw_fillrect(0,0,LCD_WIDTH,LCD_HEIGHT,BLACK);
			vTaskResume(task_handlers[UIEnum]);
			vTaskSuspend(NULL);

			//Flush all switch actions in case a double click on SW5 occurred
			vTaskDelay(100/portTICK_PERIOD_MS);
			flushSwitchAction();
			previousWakeTime=xTaskGetTickCount();
			gameInit();
		}
	}
}
