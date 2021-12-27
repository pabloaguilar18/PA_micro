//*****************************************************************************
//
// freertos_demo.c - Queue Sets example
//
// Copyright (c) 2019 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.0.1.11577 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h> 			 // rand()
#include "inc/hw_memmap.h"       // TIVA: Definiciones del mapa de memoria
#include "inc/hw_types.h"        // TIVA: Definiciones API
#include "inc/hw_ints.h"         // TIVA: Definiciones para configuracion de interrupciones
#include "driverlib/gpio.h"      // TIVA: Funciones API de GPIO
#include "driverlib/pin_map.h"   // TIVA: Mapa de pines del chip
#include "driverlib/rom.h"       // TIVA: Funciones API incluidas en ROM de micro (MAP_)
#include "driverlib/rom_map.h"   // TIVA: Mapeo automatico de funciones API incluidas en ROM de micro (MAP_)
#include "driverlib/sysctl.h"    // TIVA: Funciones API control del sistema
#include "driverlib/uart.h"      // TIVA: Funciones API manejo UART
#include "driverlib/interrupt.h" // TIVA: Funciones API manejo de interrupciones
#include "utils/uartstdio.h"     // TIVA: Funciones API UARTSTDIO (printf)
#include "drivers/buttons.h"     // TIVA: Funciones API manejo de botones
#include "FreeRTOS.h"            // FreeRTOS: definiciones generales
#include "task.h"                // FreeRTOS: definiciones relacionadas con tareas
#include "semphr.h"              // FreeRTOS: definiciones relacionadas con semaforos
#include "event_groups.h"        // FreeRTOS: definiciones relacionadas con grupos de eventos

// Variables globales "main"

volatile uint8_t button1Pressed=0;
volatile uint8_t button2Pressed=0;

static QueueHandle_t cola1;
static SemaphoreHandle_t semaforo1,semaforo2;
static QueueSetHandle_t grupo_colas;
/*-----------------------------------------------------------*/

#define POSICIONES_COLA 5

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}

#endif

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
	//
	// This function can not return, so loop forever.  Interrupts are disabled
	// on entry to this function, so no processor interrupts will interrupt
	// this loop.
	//
	while(1)
	{
	}
}


void vApplicationIdleHook( void )
{
	SysCtlSleep();
	//SysCtlDeepSleep();
}


// Tarea que parpadea el LED verde un numero aleatorio de veces
static portTASK_FUNCTION(GreenLedBlink,pvParameters){
	uint16_t uiIteraciones;

	//
	// Loop forever.
	//
	while(1)
	{

		uiIteraciones=rand()%20+1; //genera numero aleatorio entre 1 y 2
		//Envia a la cola de mensajes...
		xQueueSend(cola1,&uiIteraciones,portMAX_DELAY);
		while (uiIteraciones>0)
		{
			//Parpadea el n�mero de veces especificado por el numero
			uiIteraciones--;
			vTaskDelay(0.2*configTICK_RATE_HZ);
			MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
			vTaskDelay(0.2*configTICK_RATE_HZ);
			MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
		}
		vTaskDelay(1*configTICK_RATE_HZ);
	}
}


// Tarea que envia un mensaje por UART
static portTASK_FUNCTION(prvSerialTask,pvParameters){
	QueueSetMemberHandle_t  Activado;
	uint16_t dato;

	for( ;; )
	{
		//espera a que se desbloquee uno de los IPC
		Activado = xQueueSelectFromSet( grupo_colas, portMAX_DELAY);
		//Comprueba quien ha sido
		if (Activado==semaforo1)
		{
			xSemaphoreTake(semaforo1,0); //cierra el semaforo para que pueda volver a darse
			UARTprintf("Boton 1 pulsado\r\n");
		}
		else if (Activado==semaforo2)
		{
			xSemaphoreTake(semaforo2,0); //cierra el semaforo para que pueda volver a darse
			UARTprintf("Boton 2 pulsado\r\n");
		} else if (Activado==cola1)
		{
			xQueueReceive(cola1,&dato,0);	//Aqui puedo poner 0 porque seguro hay algo en la cola
			UARTprintf("He recibido %d de la cola\r\n",(uint16_t)dato);
		}

	}
}


//*****************************************************************************
//
// Initialize FreeRTOS and start the initial set of tasks.
//
//*****************************************************************************
int main(void)
{

	//
	// Set the clocking to run at 40 MHz from the PLL.
	//
	MAP_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
			SYSCTL_OSC_MAIN);

	//
	// Initialize the UART and configure it for 115,200, 8-N-1 operation.
	//
	// se usa para mandar mensajes por el puerto serie
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
	MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
	MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTStdioConfig(0, 115200, SysCtlClockGet());

	//Inicializa el puerto F (LEDs)
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
	MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);	//LEDS APAGADOS

	//Inicializa los botones (tambien en el puerto F) y habilita sus interrupciones
	ButtonsInit();
	MAP_GPIOIntTypeSet(GPIO_PORTF_BASE, ALL_BUTTONS,GPIO_FALLING_EDGE);
	MAP_GPIOIntEnable(GPIO_PORTF_BASE,ALL_BUTTONS);
	MAP_IntEnable(INT_GPIOF);

	//
	// Print demo introduction.
	//
	UARTprintf("\n\nWelcome to the TIVA EK-TM4C123GXL FreeRTOS Demo!\n");

	//
	// Create a LED task (only when #if & #endif directives are commented out)
	//


	if (xTaskCreate( GreenLedBlink, "GBli", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL )!=pdPASS)
		while (1);

	if (xTaskCreate( prvSerialTask, "Ser", 100, NULL, tskIDLE_PRIORITY+1, NULL )!=pdPASS)
	{
		while (1);
	}


	//Crea los elementos el QueueSet
	semaforo1 = xSemaphoreCreateBinary();
	if( semaforo1 == NULL )
	{
		while(1);
	}

	semaforo2 = xSemaphoreCreateBinary();
	if( semaforo2 == NULL )
	{
		while(1);
	}

	cola1 = xQueueCreate(POSICIONES_COLA,sizeof(uint16_t));
	if( cola1 == NULL )
	{
		while(1);
	}

	grupo_colas = xQueueCreateSet( POSICIONES_COLA + 1 + 1);	// El de la cola, mas uno por cada semaforo binario
	if (grupo_colas==NULL)
		while(1);

	if (xQueueAddToSet(cola1, grupo_colas)!=pdPASS)
	{
		while(1);
	}

	if (xQueueAddToSet( semaforo1, grupo_colas)!=pdPASS)
	{
		while(1);
	}

	if (xQueueAddToSet(semaforo2, grupo_colas)!=pdPASS)
		while(1);

	//
	// Start the scheduler.  This should not return.
	//
	vTaskStartScheduler();	//el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas

	//
	// In case the scheduler returns for some reason, print an error and loop
	// forever.
	//

	while(1)
	{
	}
}

// Rutinas de interrupcion
void GPIOFIntHandler(void){
	BaseType_t xHigherPriorityTaskWoken=pdFALSE;
	int32_t i32PinStatus=MAP_GPIOPinRead(GPIO_PORTF_BASE,ALL_BUTTONS);
	if (!(i32PinStatus & LEFT_BUTTON))
	{
    	//Activa semaforo 1
    	xSemaphoreGiveFromISR (semaforo1, &xHigherPriorityTaskWoken );
		MAP_GPIOIntClear(GPIO_PORTF_BASE,LEFT_BUTTON);
	}else 	if (!(i32PinStatus & RIGHT_BUTTON)){
		//Activa los flags BUTTON2a_FLAG y BUTTON2b_FLAG
    	//Activa semaforo 2
    	xSemaphoreGiveFromISR (semaforo2, &xHigherPriorityTaskWoken );
		MAP_GPIOIntClear(GPIO_PORTF_BASE,RIGHT_BUTTON);
	}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
