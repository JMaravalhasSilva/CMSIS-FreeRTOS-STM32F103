#ifndef LOWLEVEL_H_
#define LOWLEVEL_H_

#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

//These are atomic writes, no need to stop interrupts
#define SET_GPIO_BIT(gpio, bit) gpio->BSRR |= (1<<bit)
#define RESET_GPIO_BIT(gpio, bit) gpio->BRR |= (1<<bit)

//Use these to select LED in the toogleDx function
#define D1 0
#define D2 1
#define D3 2

//Use these to check if I2C communication was successful
#define I2C_OK 	 0
#define I2C_ERR -1

//Switch enumerator type
typedef enum swenum{
	RightEnum,
	LeftEnum,
	UpEnum,
	DownEnum,
	CenterEnum,
	NoAction
}swenum;


//Functions:

//All low level stuff initialization + accelerometer configuration. Lets user select USART2 baudrate and size of TX and RX queues.
//If RX queue is too small, data might get lost. If TX queue is too small, more time will be spent handling the data.
//Recommended size for both queues: 10
void initLowLevel(UBaseType_t RXQueueSize,UBaseType_t TXQueueSize,uint32_t USART2baudrate);

//Toggles leds D1, D2 or D3.
void toggleDx(uint8_t led);

//Gets actions from the switches.
swenum getSwitchAction(TickType_t xTicksToWait);

//Deletes all previous switch actions that were unattended
void flushSwitchAction(void);

//Boots up the accelerometer with the default 2g configuration - must be called from within a task
void startupAccel(void);

//Gets accelerometer data. Gives XYZ acceleration between 0g to 2g with all axis negated.
void getAccelData(float* gx,float* gy,float* gz);

//Function for getting a char from the USART2 queue - can be non blocking. Returns pdPASS or pdFAIL
BaseType_t USARTGetChar(char* USARTchar,TickType_t xTicksToWait);

//Sends a string via the USART2 - blocking function
BaseType_t USARTPutString(char* str);

//Send a char via the USART2 - can be non blocking. Returns pdPASS or pdFAIL
BaseType_t USARTPutChar(char USARTchar,TickType_t xTicksToWait);

//Starts the independent watchdog timer (set to 1 second)
void watchdogStart(void);

//Resets the watchdog timer to prevent a reset
void watchdogReset(void);

//Returns 1 if the system is booting from a watchdog reset. Otherwise returns 0.
uint8_t systemWasResetByWatchdog(void);





//Note: No need to use these functions as the "startupAccel" initializes the accelerometer and "getAccelData" abstracts accelerometer data read

//Returns I2C_OK if data read was successful, otherwise returns I2C_ERR.
//Use number_of_registers to read contiguous registers. Max value of 6 registers. Results are stored in the data array
uint8_t readFromI2C(uint8_t slave_address, uint8_t slave_register, uint8_t number_of_registers, uint8_t* data);

//Same returns as readFromI2C. Use to write values to slave registers
uint8_t writetoI2C(uint8_t slave_address, uint8_t slave_register, uint8_t data);



#endif
