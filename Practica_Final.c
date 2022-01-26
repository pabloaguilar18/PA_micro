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
#define VEL_GIRO_LENTO 960 //Velocidad lenta para el giro buscando el palo
#define VEL_GIRO_RAPIDO 970 //Velocidad algo más rápida para buscar las cajas y demás giros que se realizan
#define VEL_MEDIA_SUP_DELANTE 980 //Velocidad media superior para ir hacia delante
#define VEL_MEDIA_INF_DELANTE 925 //Velocidad media inferior para ir hacia delante
#define VEL_MEDIA_SUP_ATRAS 980 //Velocidad media superior para ir hacia atrás
#define VEL_MEDIA_INF_ATRAS 918 //Velocidad media inferior para ir hacia atrás

//Parámetros para la gestión de mecanismos FreeRTOS
#define encoder_giro 0x00000001
#define encoder_avance 0x00000010
#define encoder_choque 0x00000100
#define encoder_caja_depositada 0x00001000
#define sensor_contacto 0x00010000
#define sensor_linea 0x00100000

//Variables globales
uint32_t g_ulSystemClock;
static EventGroupHandle_t FlagsEventos; //Manejadores de los mecanismos de FreeRTOS
static QueueHandle_t cola_encoder;
static TaskHandle_t Manejador_Avanzar;
static TaskHandle_t Manejador_Girar;
static TaskHandle_t Manejador_Choque_Linea;
static TaskHandle_t Manejador_Aproximacion_Palo;

//Las ponemos como variables globales porque necesito editarlas en varias tareas
int giro_izq = 0; //calculo el número de sectores que tengo que contar cuando voy a girar
int giro_der = 0;
int inc_izq = 0; //calculo el número de sectores que tengo que contar cuando voy avanzando
int inc_der = 0;
int inc_izq_palo = 0; //calculo el número de sectores que tengo que contar cuando voy en dirección al palo (hasta que cruce las 3 líneas
int inc_der_palo = 0;
int inc_choque_izq = 0; //calculo el número de sectores que tengo que contar para avanzar cuando choque
int inc_choque_der = 0;
int giro_choque_izq = 0; //calculo el número de sectores que tengo que contar para dar el giro del choque
int giro_choque_der = 0;
int avance = 5; //Queremos retroceder 5 cm cuando choque
int angulo_giro = 30; //Queremos girar 30º para evitar la línea de nuevo
int num_lineas = 0; //Variable para saber cuántas líneas he pasado siempre y cuando esté yendo en dirección al palo y poder seguir avanzando hasta depositar la caja

typedef enum{
    BARRIDO_CAJA,
    BARRIDO_PALO,
    APROXIMACIÓN_CAJA,
    APROXIMACIÓN_PALO,
    CHOQUE_LINEA,
    CAJA_COLOCADA,
} Estado; //Son los diferentes estados que tenemos

typedef struct{
    uint32_t dato_1;
    uint32_t dato_2;
    uint32_t dato_3;
    uint32_t dato_4;
} Muestras; //Son los cuatro datos que recibimos en el ADC

Estado est = BARRIDO_CAJA; //Empezamos al comenzar en el estado de BARRIDO_CAJA

/*Cabeceras de las funciones*/
void calcula_avance();
int calculo_sectores_recta(int distancia);
int calculo_sectores_giro(int grados);
void ruedas_girando_rapido();
void ruedas_girando_lento();
void ruedas_hacia_delante();
void ruedas_hacia_atras();

/**********************TAREAS***********************/
static portTASK_FUNCTION(Girar, pvParameters){ //Tarea del barrido de la caja y palo
    int angulo_giro_girar = 360; //Giro 360 para buscar la caja o el palo
    uint32_t dato;

    while(1){
        giro_izq = calculo_sectores_giro(angulo_giro_girar); //Calculo los sectores que tienen que girar las ruedas para el ángulo buscado
        giro_der = giro_izq;

        while((giro_der >= 0) || (giro_izq >= 0)){ //Siempre que no haya terminado de girar, sigo girando
            xEventGroupWaitBits(FlagsEventos, encoder_giro, pdTRUE, pdFALSE, portMAX_DELAY); //Espero a que me llegue la flag del encoder de giro

            xQueueReceive(cola_encoder, (void*) &dato, portMAX_DELAY); //Recibo el dato de qué rueda ha girado para decrementar su cuenta

            if (dato == 68){
                giro_izq--;
                giro_der--;
            }
            else if(dato == 4) giro_izq--;
            else if(dato == 64) giro_der--;
            if((est == BARRIDO_CAJA) && (giro_izq < 0) && (giro_der < 0)){ //Si llego al final del giro y no encuentro nada, avanzo un poco para volver a buscar posteriormente
                est = APROXIMACIÓN_CAJA;
                ruedas_hacia_delante();
            }
        }
    }
}

static portTASK_FUNCTION(Avanzar, pvParameters){ //Tarea de aproximación a cajas y palo
    uint32_t dato;
    int avance_avanzar = 25; //Avanzo 25 cm en línea recta

    while(1){
        inc_izq = calculo_sectores_recta(avance_avanzar); //Calculo los sectores que tienen que girar las ruedas para el avance buscado
        inc_der = inc_izq;

        while((inc_der >= 0) || (inc_izq >= 0)){ //Siempre que no haya terminado de avanzar, sigo avanzando
            xEventGroupWaitBits(FlagsEventos, encoder_avance, pdTRUE, pdFALSE, portMAX_DELAY); //Espero a que me llegue la flag del encoder de avance

            xQueueReceive(cola_encoder, (void*) &dato, portMAX_DELAY); //Recibo el dato de qué rueda ha girado para decrementar su cuenta

            if(dato == 68){
                inc_izq--;
                inc_der--;
            }
            else if(dato == 4) inc_izq--;
            else if(dato == 64) inc_der--;
            if((est == APROXIMACIÓN_CAJA) && (inc_izq < 0) && (inc_der < 0)){ //Si llego al final del avance y no encuentro la caja, vuelvo al estado de barrido caja para volver a localizarla
                est = BARRIDO_CAJA;
                ruedas_girando_rapido();
            }
        }
   }
}

static portTASK_FUNCTION(Aproximacion_Palo, pvParameters){ //Tarea de aproximación a cajas y palo
    int avance_palo = 50; //Avanzo 50 cm en línea recta

    while(1){
        inc_izq_palo = calculo_sectores_recta(avance_palo); //Calculo los sectores que tienen que girar las ruedas para el avance buscado
        inc_der_palo = inc_izq;

        while(num_lineas < 3){ //Mientras no esté dentro del cuadrado negro interno, sigue avanzando
            xEventGroupWaitBits(FlagsEventos, encoder_caja_depositada, pdTRUE, pdFALSE, portMAX_DELAY); //Espero a que me llegue la flag del encoder de caja depositada

            calcula_avance(); //Recibo el encoder y reduzco la variable
        }

        avance_palo = 2;
        inc_izq_palo = calculo_sectores_recta(avance_palo); //Calculo los sectores que tienen que girar las ruedas para el avance buscado
        inc_der_palo = inc_izq;

        while((inc_der_palo >= 0) || (inc_izq_palo >= 0)){
            xEventGroupWaitBits(FlagsEventos, encoder_caja_depositada, pdTRUE, pdFALSE, portMAX_DELAY);

            calcula_avance(); //Recibo el encoder y reduzco la variable
        }

        est = CAJA_COLOCADA; //Si ya he llegado, cambio el estado a CAJA_COLOCADA y pongo las ruedas hacia atrás
        ruedas_hacia_atras();
        inc_izq_palo = calculo_sectores_recta(avance_palo); //Calculo los sectores que tienen que girar las ruedas para el avance buscado
        inc_der_palo = inc_izq;

        while(num_lineas > 0){ //Mientras no me salga del cuadrado negro externo, sigue retrocediendo
            xEventGroupWaitBits(FlagsEventos, encoder_caja_depositada, pdTRUE, pdFALSE, portMAX_DELAY); //Espero a que me llegue la flag del encoder de caja depositada

            calcula_avance(); //Recibo el encoder y reduzco la variable
        }

        inc_choque_izq = calculo_sectores_recta(avance); //Calculo los sectores que tienen que girar las ruedas para el avance de 5 cm
        inc_choque_der = inc_choque_izq;
        giro_choque_izq = calculo_sectores_giro(angulo_giro); //Calculo los sectores que tienen que girar las ruedas 30º
        giro_choque_der = giro_choque_izq;
        est = CHOQUE_LINEA; //Pongo el estado en CHOQUE_LÍNEA para simular como si me hubiera chocado en la última línea y pongo las ruedas a ir marcha atrás
        ruedas_hacia_atras();
   }
}

static portTASK_FUNCTION(Choque_Linea, pvParameters){
    uint32_t dato;
    EventBits_t respuesta;

    while(1){
        respuesta = xEventGroupWaitBits(FlagsEventos, encoder_choque | sensor_linea, pdTRUE, pdFALSE, portMAX_DELAY); //Espero a que me llegue la flag del encoder de choque o del sensor de línea

        if((respuesta & sensor_linea) == sensor_linea){ //Si la respuesta es sensor de línea es que me voy a salir y debo evitarlo
            inc_choque_izq = calculo_sectores_recta(avance); //Calculo los sectores que tienen que girar las ruedas para el avance de 5 cm
            inc_choque_der = inc_choque_izq;
            giro_choque_izq = calculo_sectores_giro(angulo_giro); //Calculo los sectores que tienen que girar las ruedas 30º
            giro_choque_der = giro_choque_izq;

            est = CHOQUE_LINEA; //Cambio el estado a CHOQUE_LINEA para que los encoders avancen en esta tarea de choque_linea
            ruedas_hacia_atras(); //Pongo las ruedas marcha atrás y cambio todas las variables a -1 para que las demás tareas se reseteen
            giro_der = -1;
            giro_izq = -1;
            inc_der = -1;
            inc_izq = -1;
        }
        if((respuesta & encoder_choque) == encoder_choque){ //Si la respuesta es encoder de choque es que las ruedas ya han retrocedido y tengo que seguir retrocediendo
            xQueueReceive(cola_encoder, (void*) &dato, portMAX_DELAY); //Recibo el dato de qué rueda ha girado para decrementar su cuenta

            if((inc_choque_izq >= 0) || (inc_choque_der >= 0)){ //Aquí compruebo que tengo que seguir retrocediendo
                if(dato == 68){
                    inc_choque_izq--;
                    inc_choque_der--;
                }
                else if(dato == 4) inc_choque_izq--;
                else if(dato == 64) inc_choque_der--;
                if((inc_choque_izq < 0) && (inc_choque_der < 0)){ //Si ya he retrocedido lo suficiente, tengo que ponerme a girar
                    ruedas_girando_rapido();
                }
            }
            else if((giro_choque_izq >= 0) || (giro_choque_der >= 0)){ //Aquí compruebo que tengo que seguir girando
                if(dato == 68){
                    giro_choque_izq--;
                    giro_choque_der--;
                }
                else if(dato == 4) giro_choque_izq--;
                else if(dato == 64) giro_choque_der--;
                if((giro_choque_izq < 0) && (giro_choque_der < 0)){ //Cuando he terminado de girar, cambio el estado a APROXIMACIÓN_CAJA para avanzar 25 cm y pongo las ruedas hacia delante
                    est = APROXIMACIÓN_CAJA;
                    ruedas_hacia_delante();
                }
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

    if((dato & GPIO_PIN_3) == GPIO_PIN_3){ //Si es el encoder de línea mando tal flag o aumento o reduzco el número de líneas, dependiendo del estado en que esté
        if(est == APROXIMACIÓN_PALO) num_lineas++;
        else if(est == CAJA_COLOCADA) num_lineas--;
        else xEventGroupSetBitsFromISR(FlagsEventos, sensor_linea, &higherPriorityTaskWoken);
    }
    if(((dato & GPIO_PIN_2) == GPIO_PIN_2) || ((dato & GPIO_PIN_6) == GPIO_PIN_6)){ //Si son los encoders de las ruedas, miro en qué estado estoy y envío tal flag
        xQueueSendFromISR(cola_encoder, &dato, &higherPriorityTaskWoken); //Envío por la cola qué encoder es

        if(est == CHOQUE_LINEA) xEventGroupSetBitsFromISR(FlagsEventos, encoder_choque, &higherPriorityTaskWoken); //Envío la flag dependiendo del estado en el que estemos
        else if(est == APROXIMACIÓN_CAJA) xEventGroupSetBitsFromISR(FlagsEventos, encoder_avance, &higherPriorityTaskWoken);
        else if((est == APROXIMACIÓN_PALO) || (est == CAJA_COLOCADA)) xEventGroupSetBitsFromISR(FlagsEventos, encoder_caja_depositada, &higherPriorityTaskWoken);
        else xEventGroupSetBitsFromISR(FlagsEventos, encoder_giro, &higherPriorityTaskWoken);
    }

    GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_2 | GPIO_INT_PIN_3 | GPIO_INT_PIN_6); //Limpiamos la flag de interrupción
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

void configADC0_ISR(void){
    signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;
    Muestras ADC;

    ADCSequenceDataGet(ADC0_BASE, 1, (void*)& ADC); //Cogemos los datos del ADC y los guardo en la estructura

    if((est == BARRIDO_CAJA) && (ADC.dato_1 <= 1400 && ADC.dato_1 >= 750)){ //Si estamos en el barrido de la caja y el dato se encuentra entre esos dos datos de distancia
        est = APROXIMACIÓN_CAJA; //Cambio el estado a APROXIMACIÓN_CAJA a espera de encontrarla y pongo las ruedas hacia delante y cambio las variables del giro a -1
        ruedas_hacia_delante();
        giro_izq = -1;
        giro_der = -1;
    }
    else if((est == BARRIDO_PALO) && (ADC.dato_4 <= 1200 && ADC.dato_4 >= 850)){ //Si estamos en el barrido del palo y el dato se encuentra entre esos dos datos de distancia
        est = APROXIMACIÓN_PALO; //Cambio el estado a APROXIMACIÓN_PALO a espera de dejar la caja y pongo las ruedas hacia delante y cambio las variables del giro a -1
        ruedas_hacia_delante();
        giro_izq = -1;
        giro_der = -1;
    }

    ADCIntClear(ADC0_BASE, 1); //Limpiamos el flag de interrupción

    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}


void SensorContacto(void){ //Informa que ya tiene dentro la caja
    signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    if((est == APROXIMACIÓN_CAJA) || (est == CHOQUE_LINEA)){ //Si estamos aproximándonos a la caja o avanzando si he chocado y se ha activado el whisker
        est = BARRIDO_PALO; //cambio el estado a BARRIDO_PALO y pongo las ruedas a girar lento para buscar el palo
        ruedas_girando_lento();
        inc_izq = -1; //Dejo de girar para resetear otras tareas
        inc_der = -1;
    }

    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_4); /*Limpia interrupción*/
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

void ManejadorBotones(void){ //Si pulso cualquiera de los dos botones, entonces elimino todas las tareas, si las quiero reanudar se pulsará el botón de RESET
    signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    TimerDisable(TIMER2_BASE, TIMER_A);
    IntDisable(INT_GPIOB);
    IntDisable(INT_GPIOD);
    vTaskDelete(Manejador_Girar);
    vTaskDelete(Manejador_Avanzar);
    vTaskDelete(Manejador_Choque_Linea);
    vTaskDelete(Manejador_Aproximacion_Palo);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT);
    GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_1 | GPIO_INT_PIN_2 | GPIO_INT_PIN_3 | GPIO_INT_PIN_6);
    ADCIntClear(ADC0_BASE, 1);

    GPIOIntClear(GPIO_PORTF_BASE, ALL_BUTTONS); //limpiamos flags
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

/***************************FUNCIONES_AUXILIARES********************/
void calcula_avance(){
    uint32_t dato;

    xQueueReceive(cola_encoder, (void*) &dato, portMAX_DELAY); //Recibo el dato del encoder y reduzco la variable

    if(dato == 68){
        inc_izq_palo--;
        inc_der_palo--;
    }
    else if(dato == 4) inc_izq_palo--;
    else if(dato == 64) inc_der_palo--;
}

void ruedas_girando_rapido(){
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, VEL_GIRO_RAPIDO); //Cambio las ruedas a girar rápido
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, VEL_GIRO_RAPIDO);
}

void ruedas_girando_lento(){
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, VEL_GIRO_LENTO); //Cambio las ruedas a girar lento
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, VEL_GIRO_LENTO);
}

void ruedas_hacia_delante(){
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, VEL_MEDIA_INF_DELANTE); //Cambio las ruedas para avanzar
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, VEL_MEDIA_SUP_DELANTE);
}

void ruedas_hacia_atras(){
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, VEL_MEDIA_SUP_ATRAS); //Cambio las ruedas para ir hacia atrás
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
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6);
    GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6, GPIO_BOTH_EDGES);
    GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6);
    IntEnable(INT_GPIOD);

    /*************************CONFIGURACIÓN_ADC0*****************************/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0); //PRUEBA
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // Enable pin PE2 and PE3 for ADC
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2 | GPIO_PIN_3);

    //CONFIGURAR SECUENCIADOR 1
    ADCSequenceDisable(ADC0_BASE, 1);

    //Configuramos la velocidad de conversion al maximo (1MS/s)
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_FULL, 1);

    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_TIMER, 0); //Disparo software (processor trigger)

    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);

    ADCSequenceEnable(ADC0_BASE, 1);

    //Habilita las interrupciones
    ADCIntClear(ADC0_BASE, 1);
    ADCIntEnable(ADC0_BASE, 1);
    IntPrioritySet(INT_ADC0SS1, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntEnable(INT_ADC0SS1);

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

    FlagsEventos = xEventGroupCreate(); //Me creo una flag de evngtos
    if(FlagsEventos == NULL) while(1);

    cola_encoder = xQueueCreate(8, sizeof(uint32_t)); //Creación de la cola del encoder
    if (cola_encoder == NULL) while(1);

    GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_2 | GPIO_INT_PIN_3 | GPIO_INT_PIN_6);
    GPIOIntClear(GPIO_PORTF_BASE, ALL_BUTTONS); //limpiamos flags
    ADCIntClear(ADC0_BASE, 1); //Limpiamos el flag de interrupciones

    SysCtlDelay(3000 * (SysCtlClockGet() / 3 / 1000)); //Esperamos 3 segundos antes de que empiece el programa

    ruedas_girando_rapido();

    vTaskStartScheduler();  //el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas

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
