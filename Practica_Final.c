//*****************************************************************************
//
// Codigo de práctica autónoma
// Autores: Daniel Torres Viudes y Pablo Aguilar Pérez
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

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

/*Parámetros de tareas: tamaño de pila y prioridad*/
#define Sensado_Distancia_STACK (256)
#define Sensado_Distancia_PRIORITY (tskIDLE_PRIORITY+1)

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
#define sensor_medio 0x0001
#define encoder 0x0010
#define LED_rojo 0x0100

//Variables globales
uint32_t g_ui32CPUUsage;
uint32_t g_ulSystemClock;
static EventGroupHandle_t FlagsEventos;
static TimerHandle_t manejador_timer;
static QueueHandle_t cola_adc;
static QueueHandle_t cola_encoder;
static TaskHandle_t manejador_sensado_distancia;
static int avance = 18;
static int avance_corto = 12;

/*Constantes globales*/
#define distancia_ruedas 8.5

/*Cabeceras*/
int mover_robot(int distancia);
int girar_robot(int grados);
void mueve(int inc_der, int inc_izq);
void mueve_derecha(int giro);

void movimiento()
{
   case 1:
       break;

   case 2:
       break;
}

void giro(){

}

/**********************TAREAS***********************/

/*TAREA PARA LA GESTIÓN DE LA CONMUTACIÓN DE LEDs POR ENCODER*/
static portTASK_FUNCTION(Sensado_Distancia,pvParameters)
{
    //IDEA: MAQUINA DE ESTADOS
    int inc_izq = 0.0;
    int inc_der = 0.0;
    volatile int giro = 0.0;

    while (1)
    {
         inc_izq = mover_robot(avance);
         inc_der = mover_robot(avance);
         mueve(inc_der, inc_izq);

         giro = girar_robot(90);
         giro++;
         mueve_derecha(giro);

         inc_izq = mover_robot(avance_corto);
         inc_der = mover_robot(avance_corto);
         mueve(inc_der, inc_izq);

         giro = girar_robot(90);
         mueve_derecha(giro);

         inc_izq = mover_robot(avance);
         inc_der = mover_robot(avance);
         mueve(inc_der, inc_izq);

         giro = girar_robot(90);
         giro++;
         mueve_derecha(giro);

         inc_izq = mover_robot(avance_corto);
         inc_der = mover_robot(avance_corto);
         mueve(inc_der, inc_izq);

         giro = girar_robot(90);
         mueve_derecha(giro);
   }
}

/*************************RUTINAS DE INTERRUPCIÓN***************************/

void ManejadorBotones(void)
{
    signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    int32_t i32Status = GPIOIntStatus(GPIO_PORTF_BASE,ALL_BUTTONS);

    // Boton izquierdo: suspende la tarea
    if(((i32Status & LEFT_BUTTON) == LEFT_BUTTON))
    {
        vTaskSuspend(manejador_sensado_distancia);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT_IZQ);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT_DER);
    }
    // Boton derecho: da comienzo a la tarea
    if((i32Status & RIGHT_BUTTON) == RIGHT_BUTTON)
    {
        vTaskResume(manejador_sensado_distancia);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, COUNT_2MS_IZQ);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, COUNT_1MS_DER);
    }
    GPIOIntClear(GPIO_PORTF_BASE,ALL_BUTTONS);  //limpiamos flags
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
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

/*Gestiona la interrupción del pin gpio asociado a los encoders*/
void Encoder(void)
{
    signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;
    //uint32_t dato = GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_2 | GPIO_PIN_6);
    uint32_t dato;

    dato = GPIOIntStatus(GPIO_PORTD_BASE,true);

    xQueueSendFromISR(cola_encoder, &dato, &higherPriorityTaskWoken); //Guardamos en la cola
    xEventGroupSetBitsFromISR(FlagsEventos,encoder, &higherPriorityTaskWoken);

    GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_2|GPIO_INT_PIN_6);
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

void configADC0_ISR(void)
{
  signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    uint32_t dato[4];
    uint32_t dato_final;

    ADCIntClear(ADC0_BASE, 1); //LIMPIAMOS EL FLAG DE INTERRUPCIONES

    ADCSequenceDataGet(ADC0_BASE, 1, (uint32_t*)dato); //COGEMOS EL DATO GUARDADO

    dato_final = dato[0];

    xQueueSendFromISR(cola_adc, &dato_final, &higherPriorityTaskWoken); //Guardamos en la cola
    xEventGroupSetBitsFromISR(FlagsEventos,sensor_medio, &higherPriorityTaskWoken);

    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

/***************************FUNCIONES_AUXILIARES********************/
int busqueda_distancia(uint32_t A[], uint32_t key, uint32_t imin, uint32_t imax)
{
  int imid;

  while (imin < imax)
    {
      imid= (imin+imax)>>1;
      if (A[imid] < key)
        imin = imid + 1;
      else
        imax = imid;
    }
    return imax;    //Al final imax=imin y en dicha posicion hay un numero mayor o igual que el buscado
}

int mover_robot(int distancia)
{
   volatile double num_sectores = 0;
   volatile double angulo = 0;

   angulo = distancia/2.8;
   num_sectores = angulo/(M_PI/9);
   num_sectores--;

   return num_sectores;
}


int girar_robot(int grados)
{
   volatile double num_sectores = 0;
   volatile double rad = 0;
   volatile double distancia_recorrida = 0;

   rad = (grados*2*M_PI)/360;
   distancia_recorrida = rad*distancia_ruedas;
   num_sectores = distancia_recorrida/0.977;

   return num_sectores;
}

void mueve(int inc_der, int inc_izq)
{
    uint32_t dato;

    while (inc_der >= 0 && inc_izq >= 0)
    {
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, COUNT_2MS_IZQ);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, COUNT_1MS_DER);

        xQueueReceive(cola_encoder, (void*) &dato, portMAX_DELAY);

        if (dato == 4)
        {
            inc_izq--;
        }
        if (dato == 64)
        {
            inc_der--;
        }
        if (dato == 68)
        {
            inc_izq--;
            inc_der--;
        }
    }
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT_IZQ);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT_DER);
}

void mueve_derecha(int giro)
{
    uint32_t dato;

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT_DER);

    while (giro >= 0)
    {
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, COUNT_2MS_IZQ);

        xQueueReceive(cola_encoder, (void*) &dato, portMAX_DELAY);

        if (dato == 4)
        {
            giro--;
        }
    }
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT_IZQ);
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

     SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  //Habilita modulo PWM
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //Habilita puerto salida para señal PWM
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //Habilita puerto B para el sensor de contacto
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); //Habilita puerto D para el encoder

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

     PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT_IZQ);  // Establece el periodo (en este caso, un porcentaje del valor máximo)
     PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true); // Habilita la salida de la señal
     PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT_DER);  // Establece el periodo (en este caso, un porcentaje del valor máximo)
     PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true); // Habilita la salida de la señal
     PWMGenEnable(PWM1_BASE, PWM_GEN_3); //Habilita/pone en marcha el generador PWM

     /*Habilita interrupciones del puerto B (sensor de contacto)*/
     GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
     IntEnable(INT_GPIOB);

     // Set each of the button GPIO pins as an input with a pull-up.
     ROM_GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_DIR_MODE_IN); //Direccionamiento: establece resistencia de pull-up para sensor de contacto
     MAP_GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);//Establece corriente para sensor de contacto


     /*Configuración de los LEDs RDG en modo GPIO*/
     //MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
     GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_1, 0);
     GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_2, 0);
     GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_3, 0);

     /*Configuración de los pines de enconder*/
     GPIOPinTypeGPIOInput(GPIO_PORTD_BASE,GPIO_PIN_2|GPIO_PIN_6);
     GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_6, GPIO_BOTH_EDGES);
     GPIOIntEnable(GPIO_PORTD_BASE,GPIO_INT_PIN_2|GPIO_PIN_6);
     IntEnable(INT_GPIOD);

     /*************************CONFIGURACIÓN_ADC0*****************************/

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    //HABILITAMOS EL GPIOE
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // Enable pin PE3 for ADC AIN0
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    //CONFIGURAR SECUENCIADOR 0
    ADCSequenceDisable(ADC0_BASE, 1);

    //Configuramos la velocidad de conversion al maximo (1MS/s)
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_FULL, 1);

    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_TIMER, 0); //Disparo software (processor trigger)
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0| ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 1);

    //Habilita las interrupciones
    ADCIntClear(ADC0_BASE, 1);
    ADCIntEnable(ADC0_BASE, 1);
    IntPrioritySet(INT_ADC0SS1, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntEnable(INT_ADC0SS1);

    /*****************************TIMER_ADC0**************************/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);/*Habilita interrupciones del timer2*/
    TimerConfigure(TIMER2_BASE,TIMER_CFG_A_PERIODIC);/*Configura el timer2A (32bits), de forma periódica*/
    TimerLoadSet(TIMER2_BASE,TIMER_A,SysCtlClockGet());/*Carga valor de cuenta (1seg inicialmente), con 40*10^6 ciclos de reloj*/
    TimerEnable(TIMER2_BASE,TIMER_A);/*Comienza a contar*/
    TimerControlTrigger(TIMER2_BASE, TIMER_A, true);/*Dispara las interrupciones en los ADCs*/

    /**************************CREACIÓN DE TAREAS Y MECANISMOS FreeRTOS***************************/
    if((xTaskCreate(Sensado_Distancia, (portCHAR *)"Sensado_Distancia", Sensado_Distancia_STACK,NULL, Sensado_Distancia_PRIORITY, &manejador_sensado_distancia) != pdTRUE))
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

    /*Creación de la cola de ADC0*/
    cola_adc = xQueueCreate(8, sizeof(uint32_t));
    if (cola_adc == NULL)
    {
        while (1);
    }

    /*Creación de la cola del encoder*/
    cola_encoder = xQueueCreate(8, sizeof(uint32_t));
    if (cola_encoder == NULL)
    {
        while (1);
    }


    SysCtlDelay(5000*(SysCtlClockGet() / 3 / 1000));

	vTaskStartScheduler();	//el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas

	while(1)
	{
		//Si llego aqui es que algo raro ha pasado
	}
}

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
