//*****************************************************************************
//
// Codigo de partida Practica 1.
// Autores: Eva Gonzalez, Ignacio Herrero, Jose Manuel Cano
//
//*****************************************************************************

#include<stdbool.h>
#include<stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "utils/uartstdio.h"
#include "drivers/buttons.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "utils/cpu_usage.h"


#include "drivers/rgb.h"
//#include "drivers/configADC.h"
//#include "commands.h"

//#include <remotelink.h>
//#include <serialprotocol.h>


/*Parámetros de tareas: tamaño de pila y prioridad*/
#define Movimiento_Motores_STACK (256)
#define Movimiento_Motores_PRIORITY (tskIDLE_PRIORITY+1)


/*Parámetros motores*/
#define PERIOD_PWM 12500  // Periodo de 20ms
#define COUNT_1MS_IZQ 625      // TODO: Ciclos para amplitud de pulso de 1ms (max velocidad en un sentido)
#define COUNT_1MS_DER 625      // TODO: Ciclos para amplitud de pulso de 1ms (max velocidad en un sentido)
#define STOPCOUNT_IZQ 950    // TODO: Ciclos para amplitud de pulso de parada (1.52ms)
#define STOPCOUNT_DER 950    // TODO: Ciclos para amplitud de pulso de parada (1.52ms)
#define COUNT_2MS_IZQ 1250    // TODO: Ciclos para amplitud de pulso de 2ms (max velocidad en el otro sentido)
#define COUNT_2MS_DER 1250    // TODO: Ciclos para amplitud de pulso de 2ms (max velocidad en el otro sentido)
#define NUM_STEPS_IZQ 50    // Pasos para cambiar entre el pulso de 2ms al de 1ms
#define NUM_STEPS_DER 50    // Pasos para cambiar entre el pulso de 2ms al de 1ms
#define CYCLE_INCREMENTS_IZQ 20//(abs(COUNT_1MS_IZQ-COUNT_2MS_IZQ))/NUM_STEPS_IZQ  // Variacion de amplitud tras pulsacion
#define CYCLE_INCREMENTS_DER 20//(abs(COUNT_1MS_DER-COUNT_2MS_DER))/NUM_STEPS_DER  // Variacion de amplitud tras pulsacion

//Parámetros para la gestión de mecanismos FreeRTOS

#define LED_rojo 0x0001

//Globales
uint32_t g_ui32CPUUsage;
uint32_t g_ulSystemClock;
static EventGroupHandle_t FlagsEventos;
static TimerHandle_t manejador_timer;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
// Esta funcion se llama si la biblioteca driverlib o FreeRTOS comprueban la existencia de un error (mediante
// las macros ASSERT(...) y configASSERT(...)
// Los parametros nombrefich y linea contienen informacion de en que punto se encuentra el error...
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *nombrefich, uint32_t linea)
{
    while(1) //Si la ejecucion esta aqui dentro, es que el RTOS o alguna de las bibliotecas de perifericos han comprobado que hay un error
    { //Mira el arbol de llamadas en el depurador y los valores de nombrefich y linea para encontrar posibles pistas.
    }
}
#endif

//*****************************************************************************
//
// Aqui incluimos los "ganchos" a los diferentes eventos del FreeRTOS
//
//*****************************************************************************

//Esto es lo que se ejecuta cuando el sistema detecta un desbordamiento de pila
//
void vApplicationStackOverflowHook(TaskHandle_t pxTask,  char *pcTaskName)
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

//Esto se ejecuta cada Tick del sistema. LLeva la estadistica de uso de la CPU (tiempo que la CPU ha estado funcionando)
void vApplicationTickHook( void )
{
	static uint8_t count = 0;

	if (++count == 10)
	{
		g_ui32CPUUsage = CPUUsageTick();
		count = 0;
	}
	//return;
}

//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationIdleHook (void)
{
	SysCtlSleep();
}


//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationMallocFailedHook (void)
{
	while(1);
}



/**********************TAREAS***********************/

/*TAREA PARA LA GESTIÓN DEL MOVIMIENTO DEL MOTOR*/
static portTASK_FUNCTION(Movimiento_Motores,pvParameters)
{
    //
    // Bucle infinito, las tareas en FreeRTOS no pueden "acabar", deben "matarse" con la funcion xTaskDelete().
    //IDEA: MAQUINA DE ESTADOS
    EventBits_t respuesta;

       while (1)
       {


           respuesta = xEventGroupWaitBits(FlagsEventos, LED_rojo, pdTRUE,
                  pdFALSE,portMAX_DELAY);


           if((respuesta&LED_rojo)==LED_rojo) /*Recibe información del choque y cambia de trayectoria*/
           {

               PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, COUNT_2MS_IZQ);
               PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, COUNT_1MS_DER);


               SysCtlDelay(400*(SysCtlClockGet() / 3 / 1000));

               PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, COUNT_2MS_IZQ);
               PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT_DER);


               SysCtlDelay(200*(SysCtlClockGet() / 3 / 1000));

               PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, COUNT_1MS_IZQ);
               PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, COUNT_2MS_DER);



           }



       }
}

/*************************RUTINAS DE INTERRUPCIÓN***************************/

/*Gestiona la interrupción de los botones para aumentar y decrementar la velocidad de las ruedas, así como su dirección*/
void ManejadorBotones(void)
{
    int32_t i32Status = GPIOIntStatus(GPIO_PORTF_BASE,ALL_BUTTONS);
    int32_t valor1 = PWMPulseWidthGet(PWM1_BASE, PWM_OUT_6);
    int32_t valor2 = PWMPulseWidthGet(PWM1_BASE, PWM_OUT_7);

    // Boton Izquierdo: reduce ciclo de trabajo en CYCLE_INCREMENTS para el servo conectado a PF4, hasta llegar a MINCOUNT
    if(((i32Status & LEFT_BUTTON) == LEFT_BUTTON)){
        if(valor1 > COUNT_1MS_IZQ && valor2 < COUNT_2MS_DER){
            valor1 = valor1 - CYCLE_INCREMENTS_IZQ;
            valor2 = valor2 + CYCLE_INCREMENTS_DER;

            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, valor1);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, valor2);
        }
    }
    // Boton Derecho: aumenta ciclo de trabajo en CYCLE_INCREMENTS para el servo conectado a PF4, hasta llegar a MAXCOUNT
    if(((i32Status & RIGHT_BUTTON) == RIGHT_BUTTON)){
        if(valor2 > COUNT_1MS_DER && valor1 < COUNT_2MS_IZQ){
            valor1 = valor1 + CYCLE_INCREMENTS_IZQ;
            valor2 = valor2 - CYCLE_INCREMENTS_DER;

            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, valor1);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, valor2);
        }
    }
    GPIOIntClear(GPIO_PORTF_BASE,ALL_BUTTONS);  //limpiamos flags
}

/*Gestiona la interrupción del sensor de contacto para encerder el led rojo, y avisar a la tarea correspondiente*/
void SensorContacto(void)
{
    signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;


    xEventGroupSetBitsFromISR(FlagsEventos, LED_rojo, &higherPriorityTaskWoken);
   /*Informa a la tarea del LED_rojo activado -> choque*/



    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
    xTimerStart(manejador_timer,0); /*Arranca timer para contabilizar encendido del led*/


    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_5);/*Limpia interrupción*/

    portEND_SWITCHING_ISR(higherPriorityTaskWoken);


}

/*TIMER2A_PARA_ADC0*/

void configADC0_ISR(void)
{
}


/*************************CALL_BACKS***************************/
void choqueRobot(TimerHandle_t manejador_timer)
{

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1,0); /*Apaga LED rojo tras 1s*/
}


int main(void)
{

     // Elegir reloj adecuado para los valores de ciclos sean de tamaño soportable
     SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // Reloj del sistema a 40MHz
     SysCtlPWMClockSet(SYSCTL_PWMDIV_64); // Establece la frecuencia para el pwm de los motores (40MHz/64=625kHz)

     //Consigue la velocidad del sistema: 40M
     g_ulSystemClock = SysCtlClockGet();

     //Habilita el clock gating de los perifericos durante el bajo consumo --> perifericos que se desee activos en modo Sleep
     //                                                                        deben habilitarse con SysCtlPeripheralSleepEnable
     //MAP_SysCtlPeripheralClockGating(true);DUDA!!!!!!

     // Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
     // Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER3 (ultimo parametro que se pasa a la funcion)
     // (y por tanto este no se deberia utilizar para otra cosa).
     //CPUUsageInit(g_ulSystemClock, configTICK_RATE_HZ / 10, 3);DUDA!!!!!!


    /*Se habilitan perifericos
    * (introducir aquí SysCtlPeripheralSleepEnable, en caso de perifericos con bajo consumo)
    */

     SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  //Habilita modulo PWM
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //Habilita puerto salida para señal PWM
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //Habilita puerto B para el sensor de contacto

     /*Configuración de PWM para motores*/
     GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);    // PF2 como salida PWM
     GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);    // PF3 como salida PWM
     GPIOPinConfigure(GPIO_PF2_M1PWM6);              // del módulo PWM1
     GPIOPinConfigure(GPIO_PF3_M1PWM7);              // del módulo PWM1


     // Configura pulsadores placa TIVA (int. por flanco de bajada)
     ButtonsInit();
     GPIOIntTypeSet(GPIO_PORTF_BASE, ALL_BUTTONS,GPIO_RISING_EDGE);
     GPIOIntEnable(GPIO_PORTF_BASE,ALL_BUTTONS);
     IntEnable(INT_GPIOF);


     // Configuracion ondas PWM: frecuencia 50Hz, anchura inicial= valor STOPCOUNT, 1540us
     //para salida por PF2, y COUNT_1MS (o COUNT_2MS ) para salida por PF3(puedes ponerlo
     //inicialmente a PERIOD_PWM/10)
     PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);   // Módulo PWM contara hacia abajo
     PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, PERIOD_PWM);    // Carga la cuenta que establece la frecuencia de la señal PWM

     PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, COUNT_1MS_IZQ);  // Establece el periodo (en este caso, un porcentaje del valor máximo)
     PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true); // Habilita la salida de la señal
     PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, COUNT_2MS_DER);  // Establece el periodo (en este caso, un porcentaje del valor máximo)
     PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true); // Habilita la salida de la señal
     PWMGenEnable(PWM1_BASE, PWM_GEN_3); //Habilita/pone en marcha el generador PWM


     /*Habilita interrupciones del puerto B (sensor de contacto)*/
     GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
     IntEnable(INT_GPIOB);

     //
     // Set each of the button GPIO pins as an input with a pull-up.
     //
     ROM_GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_DIR_MODE_IN); //Direccionamiento: establece resistencia de pull-up para sensor de contacto
     MAP_GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);//Establece corriente para sensor de contacto


     /*Configuración de los LEDs en modo GPIO*/
     MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_1);
     GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_2, 0);
     GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_3, 0);




	/**************************CREACIÓN DE TAREAS***************************/

    if((xTaskCreate(Movimiento_Motores, (portCHAR *)"Movimiento_Motores", Movimiento_Motores_STACK,NULL, Movimiento_Motores_PRIORITY, NULL) != pdTRUE))
    {
        while(1);
    }

    FlagsEventos = xEventGroupCreate();

    if(FlagsEventos==NULL)
    {

      while(1);
    }


    manejador_timer = xTimerCreate("LED_rojo_Timer",configTICK_RATE_HZ,pdTRUE,NULL,choqueRobot);

    if(manejador_timer == NULL)
    {
        UARTprintf("Error al crear timer \r\n");
    }



	//
	// Arranca el  scheduler.  Pasamos a ejecutar las tareas que se hayan activado.
	//
	vTaskStartScheduler();	//el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas
	//De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.

	while(1)
	{
		//Si llego aqui es que algo raro ha pasado
	}
}

