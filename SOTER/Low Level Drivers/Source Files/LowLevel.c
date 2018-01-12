#include "LowLevel.h"


//File-scope variables
//***************************************************************
//Struct I2C handling without FreeRTOS queues
typedef struct I2C_struct_type{
	uint8_t slave_address;
	uint8_t register_address;
	uint8_t registers_to_read; 	//Maximum of 6 registers to read
	uint8_t data[6]; 			//Store up to 6bytes (x y and z). data[0] is used to send data
	uint8_t data_counter; 		//Stores index of data to be read
	uint8_t action_type;
	uint8_t flag;
}I2C_struct_type;

//Enumerator for types of actions the I2C can perform
enum actionsenum{
	I2C_SendEnum,
	I2C_ReadEnum,
};

//Creation of a file-scope I2C struct, blocked task handler and semaphore
I2C_struct_type I2C_struct;
TaskHandle_t BlockedI2CTask;
SemaphoreHandle_t I2Csemaphore;


//Queues for the USART2's TX and RX, a queue for the Switches, blocked task handler and semaphore
QueueHandle_t RXQueue,TXQueue,SWQueue;
SemaphoreHandle_t USARTSemaphore;
TaskHandle_t BlockedUSART2Task;

//***************************************************************





//Initializes everything: RCC, GPIO, I2C, NVIC
//***************************************************************
void initLowLevel(UBaseType_t RXQueueSize,UBaseType_t TXQueueSize,uint32_t USART2baudrate){

	I2Csemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(I2Csemaphore);

	USARTSemaphore=xSemaphoreCreateBinary();
	xSemaphoreGive(USARTSemaphore);

	RXQueue=xQueueCreate(RXQueueSize,sizeof(uint8_t));
	TXQueue=xQueueCreate(TXQueueSize,sizeof(uint8_t));

	SWQueue=xQueueCreate(10,sizeof(swenum));

	//HSE init to 72MHz using HSE
	//*********************************************************

	//While unnecessary, this makes sure the clock bypass is off
	RCC->CR &= ~RCC_CR_HSEBYP;

	//Turns on the HSE clock
	RCC->CR |= RCC_CR_HSEON;

	//Wait for HSE clock to stabilize
	while(!(RCC->CR&RCC_CR_HSERDY));

	//Reset flash latency and half-cycle bits for security
	FLASH->ACR &= ~( FLASH_ACR_HLFCYA|FLASH_ACR_LATENCY_2|FLASH_ACR_LATENCY_1|FLASH_ACR_LATENCY_0 );

	//Enable prefetch buffer and configure 2 wait cycles
	FLASH->ACR |= FLASH_ACR_LATENCY_1|FLASH_ACR_PRFTBE;

	//AHB prescaler set to no division 	(HCLK = SYSCLK)				-> RCC_CFGR_HPRE_DIV1
	//APB1 prescaler set to divide by 2 (PCLK1 = 36MHz)				-> RCC_CFGR_PPRE1_DIV2
	//APB2 prescaler set to no division (PCLK2 = 72MHz)				-> RCC_CFGR_PPRE2_DIV1
	//PLL input clock set to HSE									-> RCC_CFGR_PLLSRC
	//PLL input clock set to no division (PLL Input clock = HSE) 	-> RCC_CFGR_PLLXTPRE_HSE
	//PLL set to multiply input clock by 6 (PLLCLK=HSE*6=12Mhz*6)	-> RCC_CFGR_PLLMULL6

	RCC->CFGR |= RCC_CFGR_HPRE_DIV1|RCC_CFGR_PPRE1_DIV2|RCC_CFGR_PPRE2_DIV1|RCC_CFGR_PLLXTPRE_HSE|RCC_CFGR_PLLMULL6|RCC_CFGR_PLLSRC;

	//Turn on PLL
	RCC->CR |= RCC_CR_PLLON;

	//Wait for PLL to stabilize
	while(!(RCC->CR&RCC_CR_PLLRDY));

	//PLL selected as system clock (SYSCLK=PLLCLK=72MHz)
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	//Wait until PLLCLK is set as SYSCLK
	while( (RCC->CFGR&RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

	//*********************************************************


	//RCC inits
	//*********************************************************
	//Enable IO port A, B and C clocks. Enable AFIO clock (banged head against the wall wondering why EXTI stuff wasn't working, lol)
	RCC->APB2ENR |=  RCC_APB2ENR_IOPAEN|RCC_APB2ENR_IOPBEN|RCC_APB2ENR_IOPCEN|RCC_APB2ENR_AFIOEN;

	//Enable the I2C2 and USART2 clocks
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN|RCC_APB1ENR_USART2EN;

	//Enable the low speed internal RC clock to drive the watchdog (LSI @ 40kHz)
	RCC->CSR |= RCC_CSR_LSION;

	//Wait for the LSI to stabilize
	while( (RCC->CSR&RCC_CSR_LSIRDY) == 0);

	//*********************************************************


	//GPIO inits
	//*********************************************************

	//PORT B - LEDs on pins 0, 1 and 2. I2C on pins 10 (SCL) and 11 (SDA)

	//Put 0b11 in pin 11 CNF bits for alternate function open-drain
	//Put 0b11 in pin 11 MODE bits for 50MHz
	//NOTE: look out for writing CNF before MODE, because having 0b1100 is reserved
	//and the GPIO pins will not work!
	GPIOB->CRH |= GPIO_CRH_CNF11|GPIO_CRH_MODE11;

	//Pin 10 same as pin 11
	GPIOB->CRH |= GPIO_CRH_CNF10|GPIO_CRH_MODE10;

	//Put 0b00 in pin 2 CNF bits to ensure push-pull output
	GPIOB->CRL &= ~GPIO_CRL_CNF2;

	//Put 0b11 in pin 2 MODE bits to ensure 50 MHz output
	GPIOB->CRL |= GPIO_CRL_MODE2;

	//Pin 1 same as pin2
	GPIOB->CRL &= ~GPIO_CRL_CNF1;
	GPIOB->CRL |= GPIO_CRL_MODE1;

	//Pin 0 same as pin2
	GPIOB->CRL &= ~GPIO_CRL_CNF0;
	GPIOB->CRL |= GPIO_CRL_MODE0;


	//PORT A - Switch 5 on pin 1

	//Set pin 1 CNF bits to 0b10 for input with pull up/down
	GPIOA->CRL &= ~GPIO_CRL_CNF1;
	GPIOA->CRL |= GPIO_CRL_CNF1_1;

	//Reset pin 1 Mode bits to ensure this is an input
	GPIOA->CRL &= ~GPIO_CRL_MODE1;

	//Activate pin 1 pull-up
	GPIOA->BSRR |= GPIO_BSRR_BS1;

	//Set pin 2 to alternate function push-pull 50MHz (USART2)
	GPIOA->CRL &= ~GPIO_CRL_CNF2;
	GPIOA->CRL |= GPIO_CRL_CNF2_1;
	GPIOA->CRL |= GPIO_CRL_MODE2;

	//Just ensure that pin 3 is an input (USART2)
	GPIOA->CRL &= ~GPIO_CRL_MODE3;

	//PORT C - Switches 1 to 4 (pins 13, 10, 11 and 12, respectively)
	//Only pin 10 is activated right now (switch 2)

	//Set pin 10 CNF bits to 0b10 for input with pull up/down
	GPIOC->CRH &= ~GPIO_CRH_CNF10;
	GPIOC->CRH |= GPIO_CRH_CNF10_1;

	//Reset pin 10 Mode bits to ensure this is an input
	GPIOC->CRH &= ~GPIO_CRH_MODE10;

	//Activate pin 10 pull-up
	GPIOC->BSRR |= GPIO_BSRR_BS10;

	//Same for pin 13
	GPIOC->CRH &= ~GPIO_CRH_CNF13;
	GPIOC->CRH |= GPIO_CRH_CNF13_1;
	GPIOC->CRH &= ~GPIO_CRH_MODE13;
	GPIOC->BSRR |= GPIO_BSRR_BS13;

	//Same for pin 11
	GPIOC->CRH &= ~GPIO_CRH_CNF11;
	GPIOC->CRH |= GPIO_CRH_CNF11_1;
	GPIOC->CRH &= ~GPIO_CRH_MODE11;
	GPIOC->BSRR |= GPIO_BSRR_BS11;

	//Same for pin 12
	GPIOC->CRH &= ~GPIO_CRH_CNF12;
	GPIOC->CRH |= GPIO_CRH_CNF12_1;
	GPIOC->CRH &= ~GPIO_CRH_MODE12;
	GPIOC->BSRR |= GPIO_BSRR_BS12;




	//*********************************************************


	//I2C inits
	//*********************************************************

	//Buffer interrupt enable, Event interrupt enable, Error interrupt enable, 36 is the APB1 clock (36MHz).
	I2C2->CR2 |= I2C_CR2_ITBUFEN|I2C_CR2_ITEVTEN|36;

	//Make sure that I2C is off
	I2C2->CR1 &= 0xFFFE;

	//I2C will use standard 100kHz mode since the MMA8452Q can only update data at 800Hz
	//I2C frequency =~ 1/(th+tl)
	//PCLK1 = 36MHz
	//th = tl = CCR * 1/PCLK1
	//CCR must be 180 so that th=5us=tl and frequency =~ 1/(10us) = 100kHz
	//F/S and DUTY bits (15 and 14) are disabled by default, so I2C is by default in standard mode (100kHz)
	I2C2->CCR = 180;

	//Max allowed SCL rise time in SM mode is 1000ns
	//TRISE = (rise time in seconds / PCLK1 in seconds) + 1
	// (1000ns / 36MHz) + 1 = 36 + 1 = 37
	I2C2->TRISE = 37;

	//Enable peripheral
	I2C2->CR1 |= I2C_CR1_PE;

	//Enable acknowledge
	I2C2->CR1 |= I2C_CR1_ACK;

	//Configure Own address to 0 and keep bit 14 at 1 as specified in page 777 (RM0008)
	I2C2->OAR1 |= (1<<14);


	//*********************************************************

	//USART2 inits
	//*********************************************************

	//Nothing is written to CR2. This register changes the stop bits -> defaults to 1 stop bit
	USART2->CR2 |= 0;
	//No parity, Receiver enable, Transmitter enable, RX interrupt enable, 8 bit word length mode, USART Enable
	USART2->CR1 |= USART_CR1_RE|USART_CR1_TE|USART_CR1_RXNEIE|USART_CR1_UE;
	//Nothing from CR3 is necessary
	USART2->CR3 |= 0;

	//USARTDIV = DIV_Mantissa + (DIV_Fraction / 16) -> Page 794 RM0008
	//Tx/Rx baud rate = fPCLK1 / (USARTDIV *16) -> Page 803 RM0008

	//Example:

	//Desired baud rate: 115200 -> desired USARTDIV is 19.53125
	// DIV_Fraction = 16 * 0.53125 = 8.5 -> round to 9 = 0x9
	// Mantisa is just 19
	// real USARTDIV = 19 + (9 / 16) = 19.5625

	//Configuration would look like this:
	//USART2->BRR|=(19<<4)|9;
	//Remember to shift the mantissa by 4 bits! Page 825 RM0008

	uint32_t mantissa_final,fractional_final;
	float usartdiv, fractional;

	//WARNING: DON'T FORGET TO TYPECAST TO FLOAT! Otherwise division by integer returns no fractional part!!!!!
	usartdiv = (float) 36000000/(USART2baudrate*16);

	//Get fractional part of USARTDIV
	fractional = usartdiv - (long)usartdiv;

	//Get whole part of USARTDIV
	mantissa_final = usartdiv-fractional;

	//Get fractional part by multiplying by 16 and rounding up. (fractional*16)+0.5 will never be higher than 16.499(9)
	//meaning that fractonal_final will never be higher than 16!
	fractional_final = (uint32_t) ((fractional*16)+0.5);

	//If the fraction is bigger than 4bits (i.e. it's 0d16), carry 1 to the mantissa and subtract 0d15 or 0xF to the fractional
	if(fractional_final>0xF){
		fractional_final -= 0xF;
		mantissa_final++;
	}

	USART2->BRR |= (mantissa_final<<4)|fractional_final;
	//Way better than the code in the "standard" peripheral library :D

	//*********************************************************


	//Independent watchdog inits
	//*********************************************************

	//Write key to KR register to unlock registers PR and RLR (RM0008 page 497)
	IWDG->KR = 0x5555;

	//Set prescaler to /32 to get a frequency of 40kHz/32 = 1250Hz = 0.8ms
	IWDG->PR = 3;

	//RLR can be at maximum 4095 (0xFFF). (RLR+1)*(1/frequency) gives the time
	//that the watchdog takes to get to 0x0
	//Setting it to 1250 means it takes 1 second to generate a watchdog reset
	IWDG->RLR = 1250;

	//Reload the register with RLR so watchdog doesn't reset when turned on
	IWDG->KR = 0xAAAA;


	//*********************************************************

	//Interrupt inits (NVIC + peripheral specific registers)
	//*********************************************************
	//NOTE: the NVIC registers are available in the programming
	//manual PM0056

	//Set EXTI Line1 multiplexer to port A
	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PA;

	//Set EXTI Line10 multiplexer to port C
	AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI10_PC;

	//Set EXTI Line11 multiplexer to port C
	AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI11_PC;

	//Set EXTI Line12 multiplexer to port C
	AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI12_PC;

	//Set EXTI Line13 multiplexer to port C
	AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI13_PC;

	//Unmask EXTI Line 1, 10, 11 ,12 and 13
	EXTI->IMR |= EXTI_IMR_MR1|EXTI_IMR_MR10|EXTI_IMR_MR11|EXTI_IMR_MR12|EXTI_IMR_MR13;

	//Configure a rising edge to trigger an interrupt on lines 1, 10,11,12 and 13
	EXTI->RTSR |= EXTI_RTSR_TR1|EXTI_RTSR_TR10|EXTI_RTSR_TR11|EXTI_RTSR_TR12|EXTI_RTSR_TR13;

	//Choose type of priority bits: 4bits for priority, 0bits for subpriority
	//Check PM0056 manual! This sets the PRIGROUP in the AIRCRC register bits [10:8]
	NVIC_SetPriorityGrouping(0b011);

	//Set I2C2 Event handler priority to 0 (highest priority of all - see the errata)
	NVIC_SetPriority(I2C2_EV_IRQn,0);

	//Set USART2 Event handler priority to 1
	NVIC_SetPriority(USART2_IRQn,1);

	//Set priority of EXTI line 1 and line [10:15] to 1
	NVIC_SetPriority(EXTI1_IRQn,2);
	NVIC_SetPriority(EXTI15_10_IRQn,2);

	//Enable USART2 and EXTI line 1 and line [10:15] interrupts
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	//Enable interrupts, not really necessary since this is the default value
	__enable_irq();

}
//***************************************************************





//LED toggle function
//***************************************************************
void toggleDx(uint8_t led){

	if((GPIOB->IDR&(1<<led))==0){
		SET_GPIO_BIT(GPIOB,led);
	}else{
		RESET_GPIO_BIT(GPIOB,led);
	}
}
//***************************************************************






//Switches 1 to 5 interrupt handlers for EXTI lines, OOP style "getter" function and function for flushing previous switch actions
//***************************************************************

swenum getSwitchAction(TickType_t xTicksToWait){
	swenum SWvalue;

	if(xQueueReceive(SWQueue,&SWvalue,xTicksToWait)!=pdPASS){
		return NoAction;
	}else{
		return SWvalue;
	}
}

void flushSwitchAction(void){
	xQueueReset(SWQueue);
}

void EXTI1_IRQHandler(void){
	static BaseType_t pxHigherPriorityTaskWoken;
	static swenum SWvalue;

	//SW5
	SWvalue=CenterEnum;
	xQueueSendToBackFromISR(SWQueue,&SWvalue,&pxHigherPriorityTaskWoken);

	if(pxHigherPriorityTaskWoken==pdTRUE){
		taskYIELD();
	}

	EXTI->PR |= EXTI_PR_PR1;
	NVIC_ClearPendingIRQ(EXTI1_IRQn);
}

void EXTI15_10_IRQHandler(void){
	static BaseType_t pxHigherPriorityTaskWoken;
	static swenum SWvalue;

	//Sw1
	if( (EXTI->PR&EXTI_PR_PR13)!=0){
		SWvalue = LeftEnum;
		xQueueSendToBackFromISR(SWQueue,&SWvalue,&pxHigherPriorityTaskWoken);

		//Clear flag
		EXTI->PR |= EXTI_PR_PR13;
	}

	//Sw2
	if( (EXTI->PR&EXTI_PR_PR10)!=0){
		SWvalue = UpEnum;
		xQueueSendToBackFromISR(SWQueue,&SWvalue,&pxHigherPriorityTaskWoken);
		EXTI->PR |= EXTI_PR_PR10;
	}

	//Sw3
	if( (EXTI->PR&EXTI_PR_PR11)!=0){
		SWvalue = RightEnum;
		xQueueSendToBackFromISR(SWQueue,&SWvalue,&pxHigherPriorityTaskWoken);
		EXTI->PR |= EXTI_PR_PR11;
	}

	//Sw4
	if( (EXTI->PR&EXTI_PR_PR12)!=0){
		SWvalue = DownEnum;
		xQueueSendToBackFromISR(SWQueue,&SWvalue,&pxHigherPriorityTaskWoken);
		EXTI->PR |= EXTI_PR_PR12;
	}

	if(pxHigherPriorityTaskWoken==pdTRUE){
		taskYIELD();
	}

	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
}
//***************************************************************






//I2C functions, interrupt handlers, and accelerometers abstraction functions
//***************************************************************
uint8_t readFromI2C(uint8_t slave_address, uint8_t slave_register, uint8_t number_of_registers, uint8_t* data){
	uint8_t i=0;

	//No more than 6 contiguous registers can be read at a time
	if(number_of_registers>6){
		return I2C_ERR;
	}

	//No two tasks must have access to this function to ensure atomic read
	//and writes to the I2C struct
	if(xSemaphoreTake(I2Csemaphore,portMAX_DELAY)!=pdPASS){
		return I2C_ERR; //Should be impossible to reach here
	}

	//Wait 5 times for the bus to be free. Since this device is
	//the only master in the bus, there shouldn't be any problems here
	for(i=0;i<5;i++){
		if( (I2C2->SR2&I2C_SR2_BUSY)==0){
			break;
		}
	}

	if(i==5){
		xSemaphoreGive(I2Csemaphore);
		return I2C_ERR;
	}

	//Prepare slave address and register information
	I2C_struct.slave_address=slave_address;
	I2C_struct.register_address=slave_register;
	I2C_struct.registers_to_read=number_of_registers;

	//Clean data values
	for(i=0;i<6;i++){
		I2C_struct.data[i]=0;
	}

	//Reset data index counter
	I2C_struct.data_counter=0;

	//Choose master receiver
	I2C_struct.action_type=I2C_ReadEnum;

	//Reset flag
	I2C_struct.flag=0;

	BlockedI2CTask=xTaskGetCurrentTaskHandle();

	//Enable I2C's IRQ
	NVIC_EnableIRQ(I2C2_EV_IRQn);

	//Generate a start condition and turn on ACKs
	I2C2->CR1|=I2C_CR1_START|I2C_CR1_ACK;

	//Block task until I2C communication is over
	vTaskSuspend(BlockedI2CTask);

	//Copy all data gathered from the I2C bus
	for(i=0;i<number_of_registers;i++){
		data[i]=I2C_struct.data[i];
	}

	xSemaphoreGive(I2Csemaphore);

	return I2C_OK;
}

uint8_t writetoI2C(uint8_t slave_address, uint8_t slave_register, uint8_t data){
	uint8_t i=0;

	//All the logic here is similar to the read function

	if(xSemaphoreTake(I2Csemaphore,portMAX_DELAY)!=pdPASS){
		return I2C_ERR;
	}

	for(i=0;i<5;i++){
		if( (I2C2->SR2&I2C_SR2_BUSY)==0){
			break;
		}
	}

	if(i==5){
		xSemaphoreGive(I2Csemaphore);
		return I2C_ERR;
	}

	I2C_struct.slave_address=slave_address;
	I2C_struct.register_address=slave_register;
	I2C_struct.data[0]=data;
	I2C_struct.data_counter=0;
	I2C_struct.action_type=I2C_SendEnum;
	I2C_struct.flag=0;

	BlockedI2CTask=xTaskGetCurrentTaskHandle();

	//Enable I2C's IRQ
	NVIC_EnableIRQ(I2C2_EV_IRQn);

	//Generate a start condition and turn on ACKs
	I2C2->CR1|=I2C_CR1_START|I2C_CR1_ACK;

	//Block task until I2C communication is over
	vTaskSuspend(BlockedI2CTask);

	xSemaphoreGive(I2Csemaphore);

	return I2C_OK;
}

void I2C2_EV_IRQHandler(void){

	//Check if user wants to transmit or receive
	if(I2C_struct.action_type==I2C_ReadEnum){

		//See if SB bit is set
		if((I2C2->SR1&I2C_SR1_SB)!=0){
			if(I2C_struct.flag==0){
				//Send address with LSB reset to enter transmitter mode
				I2C2->DR= (I2C_struct.slave_address<<1);
				//Reading SR1 and writing to DR resets the SB flag
			}else{
				//This is a re-start condition, so send the device address again, now in receiver mode
				I2C2->DR= (I2C_struct.slave_address<<1) | 0x1;
				//TX is now clear because of the start condition. Enable the RX and TX interrupts to receive data
				I2C2->CR2|= (I2C_CR2_ITBUFEN);
			}
		}

		//When an address is sent to the slave, this interrupt happens
		if( (I2C2->SR1&I2C_SR1_ADDR) !=0){
			//SR2 read is necessary to reset the ADDR flag
			I2C_struct.data[0]=I2C2->SR2;
			if(I2C_struct.flag==0){
				//Send register address to read
				I2C2->DR=I2C_struct.register_address;
			}else if(I2C_struct.registers_to_read==1){
				//Just one byte to receive means the NACK must be configured in here
				I2C2->CR1 &= ~(I2C_CR1_ACK);
				I2C2->CR1 |= I2C_CR1_STOP;
			}
			//If the flag here is 1, it means that we've already sent the device address twice
			//so we can't write nothing to DR because we are about to receive data
		}

		//Register address was sent
		if( (I2C2->SR1&I2C_SR1_TXE)!= 0){
			//Generate a repeated start
			I2C2->CR1|=I2C_CR1_START;
			I2C2->DR|=0;
			//Disable TX interrupts because nothing can be written to DR
			I2C2->CR2&= ~(I2C_CR2_ITBUFEN);
			I2C_struct.flag=1;
		}

		//Data received
		if( (I2C2->SR1&I2C_SR1_RXNE)!= 0){
			I2C_struct.data[I2C_struct.data_counter]=I2C2->DR;

			if(I2C_struct.data_counter==(I2C_struct.registers_to_read-1)){
				//Last byte was sent, I2C interrupts are no longer necessary
				NVIC_DisableIRQ(I2C2_EV_IRQn);
				//A stop condition was already generated, so just resume the current task
				vTaskResume(BlockedI2CTask);
			}else if(I2C_struct.data_counter==(I2C_struct.registers_to_read-2)){
				//Prepare NACK and stop condition at second-last byte
				I2C2->CR1 &= ~(I2C_CR1_ACK);
				I2C2->CR1 |= I2C_CR1_STOP;
			}

			I2C_struct.data_counter++;

		}

	}else{ //User wants to transmit

		//Start condition
		if((I2C2->SR1&I2C_SR1_SB)!=0){

			I2C2->DR= (I2C_struct.slave_address<<1);

		}

		//Slave address sent and ACK received (ADDR bit set)
		if( (I2C2->SR1&I2C_SR1_ADDR) !=0){
			//Read SR2 just to to clear ADDR bit. data[1] is unused for sending data
			I2C_struct.data[1]=I2C2->SR2;
			//Send register to write to slave
			I2C2->DR=I2C_struct.register_address;
		}

		//Transmission buffer is empty due to ADDR clear or sent byte ACK received
		if( (I2C2->SR1&I2C_SR1_TXE)!= 0){
			if(I2C_struct.flag==0){
				//Send data
				I2C2->DR=I2C_struct.data[0];
				I2C_struct.flag=1; //Indicates that the next TXE interrupt is end of transmission
			}else{
				//ACK received, send stop condition
				I2C2->CR1 |= I2C_CR1_STOP;
				NVIC_DisableIRQ(I2C2_EV_IRQn);
				vTaskResume(BlockedI2CTask);
			}
		}
	}
	NVIC_ClearPendingIRQ(I2C2_EV_IRQn);
}
//***************************************************************






//User functions for initializing the accelerometer and getting data from it
//***************************************************************
void startupAccel(void){

	//Write 1 to CTRL_REG1 to get device out of standby mode
	writetoI2C(0x1D,0x2A,0b1);

}

void getAccelData(float* gx,float* gy,float* gz){
	uint8_t data[6];
	int16_t tmpreg;

	readFromI2C(0x1D,0x01,6,data);

	//Make shifts to get the MSBs and LSBs of X in the right place
	tmpreg = ((data[0]<<4)&0x0ff0) | ((data[1]>>4)&0x000f);

	//Perform 12 bit to 16 bit sign extension
	if((tmpreg>>11)&0x1){
		tmpreg|=0xf000;
	}

	// 2g / 2047
	*gx=-tmpreg*0.0009765625;

	tmpreg = ((data[2]<<4)&0x0ff0) | ((data[3]>>4)&0x000f);

	if((tmpreg>>11)&0x1){
		tmpreg|=0xf000;
	}

	*gy=-tmpreg*0.0009765625;

	tmpreg = ((data[4]<<4)&0x0ff0) | ((data[5]>>4)&0x000f);

	if((tmpreg>>11)&0x1){
		tmpreg|=0xf000;
	}

	*gz=-tmpreg*0.0009765625;

}
//***************************************************************




//USART2 interrupt handler and user-level functions
//***************************************************************
BaseType_t USARTGetChar(char* USARTchar,TickType_t xTicksToWait){
	return xQueueReceive(RXQueue,USARTchar,xTicksToWait);
}

BaseType_t USARTPutString(char* str){
	uint8_t i=0;

	if(xSemaphoreTake(USARTSemaphore,portMAX_DELAY)!=pdPASS){
		return pdFAIL;
	}

	while(str[i]){
		if(xQueueSendToBack(TXQueue,&str[i],0)!=pdPASS){

			USART2->CR1|= USART_CR1_TXEIE;
			BlockedUSART2Task=xTaskGetCurrentTaskHandle();
			vTaskSuspend(BlockedUSART2Task);
			xQueueSendToBack(TXQueue,&str[i],0);
		}
		i++;
	}

	USART2->CR1|= USART_CR1_TXEIE;

	xSemaphoreGive(USARTSemaphore);

	return pdPASS;
}

BaseType_t USARTPutChar(char USARTchar,TickType_t xTicksToWait){
	BaseType_t status;

	//The xSemaphoreTake and Give ensure that no two functions
	//try to write a character at the same time (not really a problem)
	//and also ensures that a function waits for a string to be
	//written to avoid placing a character in the middle of a string

	if(xTicksToWait==portMAX_DELAY){
		//Note: xTicksToWait are divided by the semaphore take and the queue write
		//but portMAX_DELAY cannot be divided by 2
		status=xSemaphoreTake(USARTSemaphore,portMAX_DELAY);
	}else{
		status=xSemaphoreTake(USARTSemaphore,xTicksToWait/2);
	}

	if(status!=pdPASS){
		return pdFAIL;
	}


	if(xTicksToWait==portMAX_DELAY){
		status=xQueueSendToBack(TXQueue,&USARTchar,portMAX_DELAY);
	}else{
		status=xQueueSendToBack(TXQueue,&USARTchar,xTicksToWait/2);
	}

	if(status!=pdPASS){
		xSemaphoreGive(USARTSemaphore);
		return pdFAIL;
	}

	USART2->CR1|= USART_CR1_TXEIE;

	xSemaphoreGive(USARTSemaphore);

	return pdPASS;

}

void USART2_IRQHandler(void) {

	static BaseType_t pxHigherPriorityTaskWoken;
	uint8_t data1,data2;

	//Both interrupts may occur at the same time, so separate "if"s are needed


	//RX interrupt
	if( (USART2->SR&USART_SR_RXNE) !=0 ){

		data1 = USART2->DR;
		xQueueSendToBackFromISR(RXQueue,&data1,&pxHigherPriorityTaskWoken);
		if( pxHigherPriorityTaskWoken == pdTRUE ){
			taskYIELD();
		}
	}


	//TX interrupt
	if((USART2->SR&USART_SR_TXE) !=0 ){

		if(xQueueReceiveFromISR(TXQueue,&data2,&pxHigherPriorityTaskWoken)==pdPASS){
			USART2->DR=data2;
			if( pxHigherPriorityTaskWoken == pdTRUE ){
				taskYIELD();
			}
		}else if (xQueueIsQueueEmptyFromISR(TXQueue)!=pdFALSE){
			//If the queue is empty, stop all transmition by disabling the interrupt
			USART2->CR1&= ~(USART_CR1_TXEIE);

			if(BlockedUSART2Task!=NULL){
				vTaskResume(BlockedUSART2Task);
				BlockedUSART2Task=NULL;
			}
		}
	}

}
//***************************************************************


//Independent watchdog functions
//***************************************************************
void watchdogStart(void){
	IWDG->KR|=0xCCCC;
}

void watchdogReset(void){
	IWDG->KR|=0xAAAA;
}

uint8_t systemWasResetByWatchdog(void){
	return ( (RCC->CSR&RCC_CSR_IWDGRSTF) != 0 );
}

//***************************************************************

