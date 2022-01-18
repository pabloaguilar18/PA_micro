//*****************************************************************************
// Codigo de práctica autónoma
// Autores: Daniel Torres Viudes y Pablo Aguilar Pérez
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
#include "drivers/rgb.h"

#define Avanzar_STACK (256) //Todas las tareas tienen la misma prioridad y tamaño de pila
#define Avanzar_PRIORITY (tskIDLE_PRIORITY + 1)
#define Girar_STACK (256)
#define Girar_PRIORITY (tskIDLE_PRIORITY + 1)
#define Choque_Linea_STACK (256)
#define Choque_Linea_PRIORITY (tskIDLE_PRIORITY + 1)
#define Aproximacion_Palo_STACK (256)
#define Aproximacion_Palo_PRIORITY (tskIDLE_PRIORITY + 1)

/*Parámetros motores*/
#define PERIOD_PWM 12500 //Periodo de 20ms
#define STOPCOUNT 950 //Ciclos para amplitud de pulso de parada (1.52ms)
#define VEL_MEDIA_SUP_DELANTE 1100 //980 //1025 //1100 //1250 //Velocidad media superior
#define VEL_MEDIA_INF_DELANTE 787 //925 //868 //787 //625 //Velocidad media inferior
#define VEL_MEDIA_SUP_ATRAS 980 //1025 //1100 //Velocidad media superior
#define VEL_MEDIA_INF_ATRAS 918 //868 //787 //Velocidad media inferior

//Parámetros para la gestión de mecanismos FreeRTOS
#define encoder_giro 0x00000001
#define encoder_avance 0x00000010
#define encoder_choque 0x00000100
#define encoder_caja_depositada 0x00001000
#define sensor_contacto 0x00010000
#define caja_localizada 0x00100000
#define palo_localizado 0x01000000
#define sensor_linea 0x10000000

//Variables globales
uint32_t g_ui32CPUUsage;
uint32_t g_ulSystemClock;
static EventGroupHandle_t FlagsEventos;
static QueueHandle_t cola_encoder;
static TaskHandle_t Manejador_Avanzar;
static TaskHandle_t Manejador_Girar;
static TaskHandle_t Manejador_Choque_Linea;
static TaskHandle_t Manejador_Aproximacion_Palo;
int giro_izq = 0; //Las ponemos como constantes globales porque necesito editarlas en varias tareas
int giro_der = 0;
int inc_izq = 0;
int inc_der = 0;
int inc_choque_izq = 0; //Calculo los sectores que tienen que girar las ruedas para el avance buscado
int inc_choque_der = 0;
int giro_choque_izq = 0; //Calculo los sectores que tienen que girar las ruedas para el ángulo buscado
int giro_choque_der = 0;
int avance_choque_izq = 0; //Recalculo todas las funciones de nuevo por si vuelvo a entrar en choque mientras choco (rechoque)
int avance_choque_der = 0;
int avance = 5; //Queremos retroceder 5 cm
int angulo_giro = 60; //Queremos girar 90º para evitar la línea de nuevo
int num_lineas = 0; //Variable para saber cuántas líneas he pasado siempre y cuando esté yendo en dirección al palo y poder seguir avanzando hasta depositar la caja
int comprobacion_giro = 0;
int comprobacion_avance = 0;

typedef enum{
    BARRIDO_CAJA,
    BARRIDO_PALO,
    APROXIMACIÓN_CAJA,
    APROXIMACIÓN_PALO,
    CHOQUE_LINEA,
    CAJA_COLOCADA,
}Estado;

Estado est = BARRIDO_CAJA; //Empezamos al comenzar en el estado de BARRIDO_CAJA

/*Cabeceras de las funciones*/
void calculo_variables_choque();
void calcula_avance();
int calculo_sectores_recta(int distancia);
int calculo_sectores_giro(int grados);
void ruedas_girando();
void ruedas_hacia_delante();
void ruedas_hacia_atras();

/**********************TAREAS***********************/
static portTASK_FUNCTION(Girar, pvParameters){ //Tarea del barrido de la caja y palo
    int angulo_giro_girar = 360; //Giro 360 para buscar la caja o el palo
    uint32_t dato;

    while(1){
        giro_izq = calculo_sectores_giro(angulo_giro_girar); //Calculo los sectores que tienen que girar las ruedas para el ángulo buscado
        giro_der = giro_izq;

        while((giro_der >= 0) || (giro_izq >= 0)){
            ADCIntDisable(ADC0_BASE, 1);
            TimerDisable(TIMER2_BASE, TIMER_A);
            xEventGroupWaitBits(FlagsEventos, encoder_giro, pdTRUE, pdFALSE, portMAX_DELAY);

            xQueueReceive(cola_encoder, (void*) &dato, portMAX_DELAY);

            if (dato == 68){
                giro_izq--;
                giro_der--;
            }
            else if(dato == 4) giro_izq--;
            else if(dato == 64) giro_der--;
            if((comprobacion_giro == 0) && (giro_izq < 0) && (giro_der < 0)){ //Si llego al final del giro y no encuentro nada, avanzo un poco para volver a buscar posteriormente
                //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT);
                //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT);
                est = APROXIMACIÓN_CAJA;
                comprobacion_avance = 0;
                ruedas_hacia_delante();
                TimerDisable(TIMER2_BASE, TIMER_A);
            }
        }
    }
}

static portTASK_FUNCTION(Avanzar, pvParameters){ //Tarea de aproximación a cajas y palo
    uint32_t dato;
    int avance_avanzar = 20; //Avanzo 20 cm en línea recta

    while(1){
        inc_izq = calculo_sectores_recta(avance_avanzar); //Calculo los sectores que tienen que girar las ruedas para el avance buscado
        inc_der = inc_izq;

        while((inc_der >= 0) || (inc_izq >= 0)){
            xEventGroupWaitBits(FlagsEventos, encoder_avance, pdTRUE, pdFALSE, portMAX_DELAY);

            xQueueReceive(cola_encoder, (void*) &dato, portMAX_DELAY);

            if(dato == 68){
                inc_izq--;
                inc_der--;
            }
            else if(dato == 4) inc_izq--;
            else if(dato == 64) inc_der--;
            if((comprobacion_avance == 0) && (inc_izq < 0) && (inc_der < 0)){ //Si llego al final del avance y no encuentro la caja, vuelvo al estado de barrido caja para volver a localizarla
                TimerEnable(TIMER2_BASE, TIMER_A);
                IntDisable(INT_GPIOB);
                comprobacion_giro = 0;
                est = BARRIDO_CAJA;
                ruedas_girando();
            }
        }
   }
}

static portTASK_FUNCTION(Aproximacion_Palo, pvParameters){ //Tarea de aproximación a cajas y palo
    int avance_palo = 50; //Avanzo 20 cm en línea recta

    while(1){
        inc_izq = calculo_sectores_recta(avance_palo); //Calculo los sectores que tienen que girar las ruedas para el avance buscado
        inc_der = inc_izq;

        while(num_lineas < 3){
            xEventGroupWaitBits(FlagsEventos, encoder_caja_depositada, pdTRUE, pdFALSE, portMAX_DELAY);

            calcula_avance();
        }

        avance_palo = 5;
        inc_izq = calculo_sectores_recta(avance_palo); //Calculo los sectores que tienen que girar las ruedas para el avance buscado
        inc_der = inc_izq;

        while((inc_der >= 0) || (inc_izq >= 0)){
            xEventGroupWaitBits(FlagsEventos, encoder_caja_depositada, pdTRUE, pdFALSE, portMAX_DELAY);

            calcula_avance();
        }

        est = CAJA_COLOCADA;
        ruedas_hacia_atras();
        avance = 20;
        inc_izq = calculo_sectores_recta(avance); //Calculo los sectores que tienen que girar las ruedas para el avance buscado
        inc_der = inc_izq;

        while(num_lineas > 0){
            xEventGroupWaitBits(FlagsEventos, encoder_caja_depositada, pdTRUE, pdFALSE, portMAX_DELAY);

            calcula_avance();
        }
        calculo_variables_choque();
        est = CHOQUE_LINEA;
        ruedas_hacia_atras();
   }
}

static portTASK_FUNCTION(Choque_Linea, pvParameters){
    uint32_t dato;

    while(1){
        xEventGroupWaitBits(FlagsEventos, encoder_choque, pdTRUE, pdFALSE, portMAX_DELAY);

        xQueueReceive(cola_encoder, (void*) &dato, portMAX_DELAY);

        if((inc_choque_izq >= 0) || (inc_choque_der >= 0)){ //Aquí compruebo que tengo que seguir retrocediendo
            if(dato == 68){
                inc_choque_izq--;
                inc_choque_der--;
            }
            else if(dato == 4) inc_choque_izq--;
            else if(dato == 64) inc_choque_der--;
            if((inc_choque_izq < 0) && (inc_choque_der < 0)){
                ruedas_girando();
            }
        }
        else if((giro_choque_izq >= 0) || (giro_choque_der >= 0)){ //Si ya he retrocedido lo suficiente, tengo que ponerme a girar
            if(dato == 68){
                giro_choque_izq--;
                giro_choque_der--;
            }
            else if(dato == 4) giro_choque_izq--;
            else if(dato == 64) giro_choque_der--;
            if((giro_choque_izq < 0) && (giro_choque_der < 0)){
                est = APROXIMACIÓN_CAJA; //Cuando he terminado de girar, vuelvo a barrer para encontrar la siguiente caja
                comprobacion_avance = 0;
                ruedas_hacia_delante();
            }
        }
    }
}

/*************************RUTINAS DE INTERRUPCIÓN***************************/
/*Gestiona la interrupción del pin gpio asociado a los encoders de los motores*/
void Encoder(void){
    signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;
    uint32_t dato;

    dato = GPIOIntStatus(GPIO_PORTD_BASE, true); //Veo de qué encoder me llega la interrupción

    if(((dato & GPIO_PIN_3) == GPIO_PIN_3) || ((dato & GPIO_PIN_7) == GPIO_PIN_7)){ //Si son los encoders de línea mando tal flag
        if(est == APROXIMACIÓN_PALO){
            num_lineas++;
        }
        else if(est == CAJA_COLOCADA){
            num_lineas--;
        }
        else{
            IntDisable(INT_GPIOB);
            TimerDisable(TIMER2_BASE, TIMER_A);
            giro_der = -1;
            giro_izq = -1;
            inc_der = -1;
            inc_izq = -1;
            comprobacion_avance = 1;
            comprobacion_giro = 1;
            calculo_variables_choque();

            est = CHOQUE_LINEA;
            ruedas_hacia_atras();
        }
    }

    else if(((dato & GPIO_PIN_2) == GPIO_PIN_2) || ((dato & GPIO_PIN_6) == GPIO_PIN_6)){ //Si son los encoders de las ruedas, miro en qué estado estoy y envío tal flag
        xQueueSendFromISR(cola_encoder, &dato, &higherPriorityTaskWoken); //Guardamos en la cola

        if(est == CHOQUE_LINEA) xEventGroupSetBitsFromISR(FlagsEventos, encoder_choque, &higherPriorityTaskWoken);
        else if(est == APROXIMACIÓN_CAJA) xEventGroupSetBitsFromISR(FlagsEventos, encoder_avance, &higherPriorityTaskWoken);
        else if((est == APROXIMACIÓN_PALO) || (est == CAJA_COLOCADA)) xEventGroupSetBitsFromISR(FlagsEventos, encoder_caja_depositada, &higherPriorityTaskWoken);
        else xEventGroupSetBitsFromISR(FlagsEventos, encoder_giro, &higherPriorityTaskWoken);
    }

    GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_2 | GPIO_INT_PIN_3 | GPIO_INT_PIN_6 | GPIO_INT_PIN_7);
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

void configADC0_ISR(void){
    signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    uint32_t dato[4];

    ADCSequenceDataGet(ADC0_BASE, 1, (uint32_t*)dato); //Cogemos el dato guardado

    if((est == BARRIDO_CAJA) && (dato[0] <= 1454 && dato[0] >= 697)){
        TimerDisable(TIMER2_BASE, TIMER_A);
        IntEnable(INT_GPIOB);
        giro_izq = -1;
        giro_der = -1;
        comprobacion_giro = 1;
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT);
        SysCtlDelay(1000 * (SysCtlClockGet() / 3 / 1000));
        est = APROXIMACIÓN_CAJA;
        ruedas_hacia_delante();
    }

    ADCIntClear(ADC0_BASE, 1); //Limpiamos el flag de interrupciones

    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

void configADC1_ISR(void){                                                                                                                  //PROBAR
    signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    uint32_t dato[4];

    ADCIntClear(ADC0_BASE, 2); //Limpiamos el flag de interrupciones

    ADCSequenceDataGet(ADC0_BASE, 2, (uint32_t*)dato); //Cogemos el dato guardado

    if((est == BARRIDO_PALO) && (dato[0] <= 1114 && dato[0] >= 528)){
        TimerDisable(TIMER2_BASE, TIMER_A);
        giro_izq = -1;
        giro_der = -1;
        comprobacion_giro = 1;
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT);
        SysCtlDelay(1000 * (SysCtlClockGet() / 3 / 1000));
        est = APROXIMACIÓN_PALO;
        ruedas_hacia_delante();
    }

    ADCIntClear(ADC0_BASE, 2); //Limpiamos el flag de interrupciones

    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

void SensorContacto(void){ //Informa que ya tiene dentro la caja
    signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_4); /*Limpia interrupción*/

    inc_izq = -1;                                      //Dejo de girar, deshabilito el sensor de contacto y habilito el ADC de larga distancia
    inc_der = -1;
    comprobacion_avance = 1;
    IntDisable(INT_GPIOB);
    TimerEnable(TIMER2_BASE, TIMER_A);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT);
    SysCtlDelay(1000 * (SysCtlClockGet() / 3 / 1000));
    est = BARRIDO_PALO;
    ruedas_girando();

    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_4); /*Limpia interrupción*/
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

void ManejadorBotones(void){
    signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    TimerDisable(TIMER2_BASE, TIMER_A);
    IntDisable(INT_GPIOD);
    IntDisable(INT_GPIOB);
    vTaskDelete(Manejador_Girar); //Si pulso cualquiera de los dos botones, entonces suspendo todas las tareas, si las quiero reanudar se pulsará el botón de RESET
    vTaskDelete(Manejador_Avanzar);
    vTaskDelete(Manejador_Choque_Linea);
    vTaskDelete(Manejador_Aproximacion_Palo);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT);

    GPIOIntClear(GPIO_PORTF_BASE, ALL_BUTTONS); //limpiamos flags
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

/***************************FUNCIONES_AUXILIARES********************/
void calcula_avance(){
    uint32_t dato;

    xQueueReceive(cola_encoder, (void*) &dato, portMAX_DELAY);

    if(dato == 68){
        inc_izq--;
        inc_der--;
    }
    else if(dato == 4) inc_izq--;
    else if(dato == 64) inc_der--;
}

void calculo_variables_choque(){
    inc_choque_izq = calculo_sectores_recta(avance); //Calculo los sectores que tienen que girar las ruedas para el avance buscado
    inc_choque_der = inc_choque_izq;
    giro_choque_izq = calculo_sectores_giro(angulo_giro); //Calculo los sectores que tienen que girar las ruedas para el ángulo buscado
    giro_choque_der = giro_choque_izq;
}
void ruedas_girando(){
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, VEL_MEDIA_SUP_DELANTE); //Cambio las ruedas a girar
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, VEL_MEDIA_SUP_DELANTE);
}

void ruedas_hacia_delante(){
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, VEL_MEDIA_INF_DELANTE); //Cambio las ruedas a girar
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, VEL_MEDIA_SUP_DELANTE);
}

void ruedas_hacia_atras(){
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, VEL_MEDIA_SUP_ATRAS); //Cambio las ruedas a girar
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, VEL_MEDIA_INF_ATRAS);
}

int calculo_sectores_recta(int distancia){ //Calculo el número de sectores que necesitamos para avanzar en línea recta
    volatile double num_sectores = 0;
    volatile double angulo = 0;

    angulo = distancia / 2.8;
    num_sectores = angulo / (M_PI / 9);
    num_sectores--;

    return num_sectores;
}

int calculo_sectores_giro(int grados){ //Calculo el número de sectores que necesitamos para girar
    volatile double num_sectores = 0;
    volatile double rad = 0;
    volatile double distancia_recorrida = 0;
    volatile double distancia_ruedas = 8.5;

    rad = (grados * 2 * M_PI) / ( 360 * 2); //Dividimos entre 2 porque giramos con las ruedas a la vez al girar
    distancia_recorrida = rad * distancia_ruedas;
    num_sectores = distancia_recorrida / 0.977;

    return num_sectores;
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

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT);  // Establece el periodo (en este caso, comienza parado)
    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true); // Habilita la salida de la señal
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT);  // Establece el periodo (en este caso, comienza parado)
    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true); // Habilita la salida de la señal
    PWMGenEnable(PWM1_BASE, PWM_GEN_3); //Habilita/pone en marcha el generador PWM

    /*Habilita interrupciones del puerto B (sensor de contacto)*/
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_4);
    IntEnable(INT_GPIOB);

    //Set each of the button GPIO pins as an input with a pull-up.
    ROM_GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN); //Direccionamiento: establece resistencia de pull-up para sensor de contacto
    MAP_GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); //Establece corriente para sensor de contacto

    /*Configuración de los LEDs RDG en modo GPIO*/
    GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);

    /*Configuración de los pines de enconder*/
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7);
    GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_3 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_BOTH_EDGES);
    GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7);
    IntEnable(INT_GPIOD);

    /*************************CONFIGURACIÓN_ADC0*****************************/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0); //PRUEBA

    //HABILITAMOS EL GPIOE
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // Enable pin PE3 for ADC
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2 | GPIO_PIN_3);

    //CONFIGURAR SECUENCIADOR 1 y 2
    ADCSequenceDisable(ADC0_BASE, 1);
    ADCSequenceDisable(ADC0_BASE, 2);

    //Configuramos la velocidad de conversion al maximo (1MS/s)
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_FULL, 1);
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_TIMER, 0); //Disparo software (processor trigger)       //PUEDE SER EL ERRORRRR??????????
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
    TimerControlTrigger(TIMER2_BASE, TIMER_A, true); //Dispara las interrupciones en los ADCs
    TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet() * 0.1); //Carga valor de cuenta (1seg inicialmente), con 40*10^6 ciclos de reloj
    TimerEnable(TIMER2_BASE, TIMER_A); //Comienza a contar

    /**************************CREACIÓN DE TAREAS Y MECANISMOS FreeRTOS***************************/
    if((xTaskCreate(Avanzar, (portCHAR *)"Avanzar", Avanzar_STACK, NULL, Avanzar_PRIORITY, &Manejador_Avanzar) != pdTRUE)) while(1);

    if((xTaskCreate(Girar, (portCHAR *)"Girar", Girar_STACK, NULL, Girar_PRIORITY, &Manejador_Girar) != pdTRUE)) while(1);

    if((xTaskCreate(Choque_Linea, (portCHAR *)"Choque_Linea", Choque_Linea_STACK, NULL, Choque_Linea_PRIORITY, &Manejador_Choque_Linea) != pdTRUE)) while(1);

    if((xTaskCreate(Aproximacion_Palo, (portCHAR *)"Aproximacion_Palo", Aproximacion_Palo_STACK, NULL, Aproximacion_Palo_PRIORITY, &Manejador_Aproximacion_Palo) != pdTRUE)) while(1);

    FlagsEventos = xEventGroupCreate();
    if(FlagsEventos == NULL) while(1);

    cola_encoder = xQueueCreate(8, sizeof(uint32_t)); //Creación de la cola del encoder
    if (cola_encoder == NULL) while(1);

    SysCtlDelay(3000 * (SysCtlClockGet() / 3 / 1000)); //Esperamos 3 segundos antes de que empiece el programa

    IntDisable(INT_GPIOB); //Deshabilito el pulsador, ya que no lo necesito al comenzar

    ruedas_girando();

	vTaskStartScheduler();	//el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas

	while(1); //Si llego aqui es que algo raro ha pasado
}

#ifdef DEBUG
void __error__(char *nombrefich, uint32_t linea){
    while(1); //Si la ejecucion esta aqui dentro, es que el RTOS o alguna de las bibliotecas de perifericos han comprobado que hay un error
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
