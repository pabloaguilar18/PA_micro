//*****************************************************************************
// Codigo de práctica autónoma
// Autores: Daniel Torres Viudes y Pablo Aguilar Pérez
//*****************************************************************************

//PB5 SENSOR DE CONTACTO (WHISKER)
//PD2 Y PD6 ENCODERS RUEDAS
//PD1 Y PD7 ENCODERS SEGUIDORES DE LÍNEA
//PE3 ADC0 1 (MEDIA DISTANCIA S41)
//PE2 ADC0 2 (LARGA DISTANCIA S21)

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
#include "drivers/rgb.h"

/*Parámetros de tareas: tamaño de pila y prioridad*/
#define Maquina_Estados_STACK (256)
#define Maquina_Estados_PRIORITY (tskIDLE_PRIORITY+1)
#define Girar_STACK (256)
#define Girar_PRIORITY (tskIDLE_PRIORITY+1)
//#define Sensado_Distancia_STACK (256)
//#define Sensado_Distancia_PRIORITY (tskIDLE_PRIORITY+1)

/*Parámetros motores*/
#define PERIOD_PWM 12500 //Periodo de 20ms
#define COUNT_1MS_IZQ 625 //Ciclos para amplitud de pulso de 1ms (max velocidad en un sentido)
#define COUNT_1MS_DER 625 //Ciclos para amplitud de pulso de 1ms (max velocidad en un sentido)
#define STOPCOUNT_IZQ 950 //Ciclos para amplitud de pulso de parada (1.52ms)
#define STOPCOUNT_DER 950 //Ciclos para amplitud de pulso de parada (1.52ms)
#define COUNT_2MS_IZQ 1250 //Ciclos para amplitud de pulso de 2ms (max velocidad en el otro sentido)
#define COUNT_2MS_DER 1250 //Ciclos para amplitud de pulso de 2ms (max velocidad en el otro sentido)

//Parámetros para la gestión de mecanismos FreeRTOS
#define encoder 0x00001
#define sensor_contacto 0x00010
#define caja_localizada 0x00100
#define palo_localizado 0x01000
#define sensor_linea 0x10000

//Variables globales
uint32_t g_ui32CPUUsage;
uint32_t g_ulSystemClock;
static EventGroupHandle_t FlagsEventos;
static QueueHandle_t cola_adc;
static QueueHandle_t cola_encoder;
static TaskHandle_t Manejador_Maquina_Estados;
static TaskHandle_t Manejador_Girar;
//static TaskHandle_t Manejador_Sensado_Distancia;


/*Constantes globales*/
#define distancia_ruedas 8.5
const uint32_t TMuestras_SM[]   = {598, 623, 697, 770, 840, 990, 1107, 1255, 1454, 1817, 2302};
const uint32_t TDistancias_SM[] = { 26,  24,  22,  20,  18,  16,   14,   12,   10,    8,    6};
const uint32_t TMuestras_SL[]   = {452, 480, 500, 528, 550, 575, 602, 624, 648, 673, 698, 720, 744, 794, 822, 872, 923, 970, 1000, 1063, 1114};
const uint32_t TDistancias_SL[] = { 80,  76,  72,  68,  64,  60,  58,  56,  54,  52,  50,  48,  46,  44,  42,  40,  38,  36,   34,   32,   30};
const uint32_t TAMAN = 11;
const uint32_t TAMA = 21;

/*Definicion de tipos*/
typedef enum{
    BARRIDO,
    CAJA_LOCALIZADA,
    APROXIMACIÓN_CAJA,
}Estado;

Estado est = BARRIDO;
int pablo =0;


/*Cabeceras*/
int calculo_sectores_recta(int distancia);
int calculo_sectores_giro(int grados);
void avanzar(int inc_der, int inc_izq);
void girar(int giro);

/**********************TAREAS***********************/

static portTASK_FUNCTION(Maquina_Estados,pvParameters){
    EventBits_t respuesta;
    int avance = 18;
    int avance_corto = 12;
    int inc_izq = 0;
    int inc_der = 0;
    int giro = 0;

    while (1){
        //respuesta = xEventGroupWaitBits(FlagsEventos, encoder | sensor_contacto | caja_localizada | palo_localizado, pdTRUE, pdFALSE, portMAX_DELAY);

        switch(est){
            case BARRIDO:
               // if(pablo < 4)
                vTaskResume(Manejador_Girar);


                break;
            case CAJA_LOCALIZADA:
                    //giro = calculo_sectores_giro(360);
                    //girar(giro);
//                if((respuesta & ADC_0) == ADC_0){ //He encontrado una caja
//                    //Cambio de estado -> estado = 1
//                }


                 inc_izq = calculo_sectores_recta(avance_corto);
                 inc_der = calculo_sectores_recta(avance_corto);
                 avanzar(inc_der, inc_izq);

                 est = BARRIDO;
                break;
        }
//         inc_izq = calculo_sectores_recta(avance);
//         inc_der = calculo_sectores_recta(avance);
//         avanzar(inc_der, inc_izq);
//
//         giro = calculo_sectores_giro(90);
//         giro++;
//         girar(giro);
//
//         inc_izq = calculo_sectores_recta(avance_corto);
//         inc_der = calculo_sectores_recta(avance_corto);
//         avanzar(inc_der, inc_izq);
//
//         giro = calculo_sectores_giro(90);
//         girar(giro);
//
//         inc_izq = calculo_sectores_recta(avance);
//         inc_der = calculo_sectores_recta(avance);
//         avanzar(inc_der, inc_izq);
//
//         giro = calculo_sectores_giro(90);
//         giro++;
//         girar(giro);
//
//         inc_izq = calculo_sectores_recta(avance_corto);
//         inc_der = calculo_sectores_recta(avance_corto);
//         avanzar(inc_der, inc_izq);
//
//         giro = calculo_sectores_giro(90);
//         girar(giro);
   }
}


static portTASK_FUNCTION(Girar,pvParameters)
{

        int giro = 0;
        EventBits_t respuesta;
        uint32_t dato;

        while (1)
        {

            giro = calculo_sectores_giro(360);//barrido

            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT_DER);

            while (giro >= 0)
            {
                PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, COUNT_2MS_IZQ);

                respuesta = xEventGroupWaitBits(FlagsEventos,
                                                sensor_linea | encoder | caja_localizada,
                                                pdTRUE, pdFALSE, 10);

                if ((respuesta & sensor_linea) == sensor_linea)
                {

                        giro = -1;
                        est = CAJA_LOCALIZADA;


                }
                else if ((respuesta & caja_localizada) == caja_localizada)
                {

                       giro = -1;
                       est = CAJA_LOCALIZADA;

                }
                else if ((respuesta & encoder) == encoder)
                {

                       giro--;

                }

            }

            pablo++;
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT_IZQ);
            vTaskSuspend(Manejador_Girar);

        }


}






/*************************RUTINAS DE INTERRUPCIÓN***************************/

void ManejadorBotones(void){
    signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    int32_t i32Status = GPIOIntStatus(GPIO_PORTF_BASE,ALL_BUTTONS);

    if((i32Status & LEFT_BUTTON) == LEFT_BUTTON){ //Boton izquierdo: suspende la tarea
        vTaskSuspend(Manejador_Maquina_Estados);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT_IZQ);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT_DER);
    }
    if((i32Status & RIGHT_BUTTON) == RIGHT_BUTTON){ //Boton derecho: da comienzo a la tarea
        vTaskResume(Manejador_Maquina_Estados);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, COUNT_2MS_IZQ);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, COUNT_1MS_DER);
    }
    GPIOIntClear(GPIO_PORTF_BASE,ALL_BUTTONS); //limpiamos flags
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

void SensorContacto(void){ //Informa que ya tiene dentro la caja
    signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    xEventGroupSetBitsFromISR(FlagsEventos, sensor_contacto, &higherPriorityTaskWoken);

    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_5);/*Limpia interrupción*/
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

/*Gestiona la interrupción del pin gpio asociado a los encoders de los motores*/
void Encoder(void){
    signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;
    uint32_t dato;

    dato = GPIOIntStatus(GPIO_PORTD_BASE, true);



    if (((dato & GPIO_PIN_1) == GPIO_PIN_1) || ((dato & GPIO_PIN_7) == GPIO_PIN_7))
    {
          xEventGroupSetBitsFromISR(FlagsEventos, sensor_linea, &higherPriorityTaskWoken);
    }

    if (((dato & GPIO_PIN_2) == GPIO_PIN_2) || ((dato & GPIO_PIN_6) == GPIO_PIN_6))
    {
          xQueueSendFromISR(cola_encoder, &dato, &higherPriorityTaskWoken); //Guardamos en la cola
          xEventGroupSetBitsFromISR(FlagsEventos, encoder, &higherPriorityTaskWoken);
    }



    GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_1 | GPIO_INT_PIN_2 | GPIO_INT_PIN_6 | GPIO_INT_PIN_7);
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

void configADC0_ISR(void){                                                                                                                 //PROBAR
    signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    uint32_t dato[4];
    uint32_t dato_final;

    ADCIntClear(ADC0_BASE, 1); //LIMPIAMOS EL FLAG DE INTERRUPCIONES

    ADCSequenceDataGet(ADC0_BASE, 1, (uint32_t*)dato); //COGEMOS EL DATO GUARDADO

    dato_final = dato[0];

    xQueueSendFromISR(cola_adc, &dato_final, &higherPriorityTaskWoken); //Guardamos en la cola
    //xEventGroupSetBitsFromISR(FlagsEventos,caja_localizada, &higherPriorityTaskWoken);

    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

void configADC1_ISR(void){                                                                                                                  //PROBAR
    signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    uint32_t dato[4];
    uint32_t dato_final;

    ADCIntClear(ADC0_BASE, 2); //LIMPIAMOS EL FLAG DE INTERRUPCIONES

    ADCSequenceDataGet(ADC0_BASE, 2, (uint32_t*)dato); //COGEMOS EL DATO GUARDADO

    dato_final = dato[1];

    xQueueSendFromISR(cola_adc, &dato_final, &higherPriorityTaskWoken); //Guardamos en la cola
    //xEventGroupSetBitsFromISR(FlagsEventos,palo_localizado, &higherPriorityTaskWoken);

    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

/***************************FUNCIONES_AUXILIARES********************/
int busqueda_distancia(uint32_t A[], uint32_t key, uint32_t imin, uint32_t imax){
    int imid;

    while (imin < imax){
       imid = (imin + imax) >> 1;
       if (A[imid] < key) imin = imid + 1;
       else imax = imid;
    }
    return imax;    //Al final imax = imin y en dicha posicion hay un numero mayor o igual que el buscado
}

int calculo_sectores_recta(int distancia){
    volatile double num_sectores = 0;
    volatile double angulo = 0;

    angulo = distancia / 2.8;
    num_sectores = angulo / (M_PI / 9);
    num_sectores--;

    return num_sectores;
}

int calculo_sectores_giro(int grados){
    volatile double num_sectores = 0;
    volatile double rad = 0;
    volatile double distancia_recorrida = 0;

    rad = (grados * 2 * M_PI) / 360;
    distancia_recorrida = rad * distancia_ruedas;
    num_sectores = distancia_recorrida / 0.977;

    return num_sectores;
}

void avanzar(int inc_der, int inc_izq){
    uint32_t dato;

    while ((inc_der >= 0) && (inc_izq >= 0)){
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, COUNT_2MS_IZQ);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, COUNT_1MS_DER);

        xQueueReceive(cola_encoder, (void*) &dato, portMAX_DELAY);

//        if (((dato & GPIO_PIN_1) == GPIO_PIN_1) || ((dato & GPIO_PIN_7) == GPIO_PIN_7)){
//            inc_izq = -1;
//            inc_der = -1;
//        }
         if (dato == 68){
            inc_izq--;
            inc_der--;
        }
        else if (dato == 4) inc_izq--;
        else if (dato == 64) inc_der--;
    }
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT_IZQ);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT_DER);
}

void girar(int giro){
    uint32_t dato;
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT_DER);

    while (giro >= 0){
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, COUNT_2MS_IZQ);

        xQueueReceive(cola_encoder, (void*) &dato, portMAX_DELAY);

        if (((dato & GPIO_PIN_1) == GPIO_PIN_1) || ((dato & GPIO_PIN_7) == GPIO_PIN_7)) giro = -1;
        else if (dato == 4) giro--;
    }
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT_IZQ);
}

int main(void){
    //Elegir reloj adecuado para los valores de ciclos sean de tamaño soportable
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // Reloj del sistema a 40MHz
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64); // Establece la frecuencia para el pwm de los motores (40MHz/64=625kHz)

    //Consigue la velocidad del sistema: 40M
    g_ulSystemClock = SysCtlClockGet();

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1); //Habilita modulo PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //Habilita puerto salida para señal PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //Habilita puerto B para el sensor de contacto
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); //Habilita puerto D para el encoder

    //Configuración de PWM para motores
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2); // PF2 como salida PWM
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3); // PF3 como salida PWM
    GPIOPinConfigure(GPIO_PF2_M1PWM6); // del módulo PWM1
    GPIOPinConfigure(GPIO_PF3_M1PWM7); // del módulo PWM1

    //Configura pulsadores placa TIVA (int. por flanco de bajada)
    ButtonsInit();
    GPIOIntTypeSet(GPIO_PORTF_BASE, ALL_BUTTONS, GPIO_RISING_EDGE);
    GPIOIntEnable(GPIO_PORTF_BASE, ALL_BUTTONS);
    IntEnable(INT_GPIOF);

    //Configuracion ondas PWM: frecuencia 50Hz, anchura inicial= valor STOPCOUNT, 1540us
    //para salida por PF2, y COUNT_1MS (o COUNT_2MS ) para salida por PF3(puedes ponerlo inicialmente a PERIOD_PWM/10)
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);   // Módulo PWM contara hacia abajo
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, PERIOD_PWM);    // Carga la cuenta que establece la frecuencia de la señal PWM

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT_IZQ);  // Establece el periodo (en este caso, comienza parado)
    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true); // Habilita la salida de la señal
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT_DER);  // Establece el periodo (en este caso, comienza parado)
    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true); // Habilita la salida de la señal
    PWMGenEnable(PWM1_BASE, PWM_GEN_3); //Habilita/pone en marcha el generador PWM

    /*Habilita interrupciones del puerto B (sensor de contacto)*/
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
    IntEnable(INT_GPIOB);

    //Set each of the button GPIO pins as an input with a pull-up.
    ROM_GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_DIR_MODE_IN); //Direccionamiento: establece resistencia de pull-up para sensor de contacto
    MAP_GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); //Establece corriente para sensor de contacto

    /*Configuración de los LEDs RDG en modo GPIO*/
    GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);

    /*Configuración de los pines de enconder*/
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_6 | GPIO_PIN_7);
    GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_BOTH_EDGES);
    GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_6 | GPIO_PIN_7);
    IntEnable(INT_GPIOD);

    /*************************CONFIGURACIÓN_ADC0*****************************/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    //HABILITAMOS EL GPIOE
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // Enable pin PE3 for ADC
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2 | GPIO_PIN_3);

    //CONFIGURAR SECUENCIADOR 1 y 2
    ADCSequenceDisable(ADC0_BASE, 1);
    ADCSequenceDisable(ADC0_BASE, 2);

    //Configuramos la velocidad de conversion al maximo (1MS/s)
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_FULL, 1);
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_TIMER, 0); //Disparo software (processor trigger)
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_TIMER, 0); //Disparo software (processor trigger)
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 1);
    ADCSequenceEnable(ADC0_BASE, 2);

    //Habilita las interrupciones
    ADCIntClear(ADC0_BASE, 1);
    ADCIntClear(ADC0_BASE, 2);
    ADCIntEnable(ADC0_BASE, 1);
    ADCIntEnable(ADC0_BASE, 2);
    IntPrioritySet(INT_ADC0SS1, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntPrioritySet(INT_ADC0SS2, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntEnable(INT_ADC0SS1);
    IntEnable(INT_ADC0SS2);

    /*****************************TIMER_ADC0**************************/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2); //Habilita interrupciones del timer2
    TimerConfigure(TIMER2_BASE, TIMER_CFG_A_PERIODIC); //Configura el timer2A (32bits), de forma periódica
    TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet()); //Carga valor de cuenta (1seg inicialmente), con 40*10^6 ciclos de reloj
    TimerEnable(TIMER2_BASE, TIMER_A); //Comienza a contar
    TimerControlTrigger(TIMER2_BASE, TIMER_A, true); //Dispara las interrupciones en los ADCs

    /**************************CREACIÓN DE TAREAS Y MECANISMOS FreeRTOS***************************/
    if((xTaskCreate(Maquina_Estados, (portCHAR *)"Maquina_Estados", Maquina_Estados_STACK, NULL, Maquina_Estados_PRIORITY, &Manejador_Maquina_Estados) != pdTRUE)){
        while(1);
    }

    if((xTaskCreate(Girar, (portCHAR *)"Girar", Girar_STACK, NULL, Girar_PRIORITY, &Manejador_Girar) != pdTRUE)){
        while(1);
    }


//    if((xTaskCreate(Sensado_Distancia, (portCHAR *)"Sensado_Distancia", Sensado_Distancia_STACK,NULL, Sensado_Distancia_PRIORITY, &Manejador_Sensado_Distancia) != pdTRUE))
//    {
//        while(1);
//    }
//

 //   vTaskSuspend(Manejador_Girar);

    FlagsEventos = xEventGroupCreate();
    if(FlagsEventos == NULL){
        while(1);
    }

    cola_adc = xQueueCreate(8, sizeof(uint32_t)); //Creación de la cola de ADC0
    if (cola_adc == NULL){
        while (1);
    }

    cola_encoder = xQueueCreate(8, sizeof(uint32_t)); //Creación de la cola del encoder
    if (cola_encoder == NULL){
        while (1);
    }

    //SysCtlDelay(5000 * (SysCtlClockGet() / 3 / 1000)); //Esperamos 5 segundos antes de que empiece el programa

	vTaskStartScheduler();	//el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas

	while(1); //Si llego aqui es que algo raro ha pasado
}

#ifdef DEBUG
void __error__(char *nombrefich, uint32_t linea){
    while(1); //Si la ejecucion esta aqui dentro, es que el RTOS o alguna de las bibliotecas de perifericos han comprobado que hay un error
              //Mira el arbol de llamadas en el depurador y los valores de nombrefich y linea para encontrar posibles pistas.
}
#endif

void vApplicationStackOverflowHook(TaskHandle_t pxTask,  char *pcTaskName){ //Esto es lo que se ejecuta cuando el sistema detecta un desbordamiento de pila
    while(1);
}

void vApplicationIdleHook (void){ //Esto se ejecuta cada vez que entra a funcionar la tarea Idle
    SysCtlSleep();
}

void vApplicationMallocFailedHook (void){ //Esto se ejecuta cada vez que entra a funcionar la tarea Idle
    while(1);
}
