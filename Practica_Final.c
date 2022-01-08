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

#define Avanzar_STACK (256) //Todas las tareas tienen la misma prioridad y tamaño de pila
#define Avanzar_PRIORITY (tskIDLE_PRIORITY + 1)
#define Girar_STACK (256)
#define Girar_PRIORITY (tskIDLE_PRIORITY + 1)
#define Choque_Linea_STACK (256)
#define Choque_Linea_PRIORITY (tskIDLE_PRIORITY + 1)
#define Sensado_Distancia_STACK (256)
#define Sensado_Distancia_PRIORITY (tskIDLE_PRIORITY + 1)

/*Parámetros motores*/
#define PERIOD_PWM 12500 //Periodo de 20ms
#define COUNT_1MS 625 //Ciclos para amplitud de pulso de 1ms (max velocidad en un sentido)
#define STOPCOUNT 950 //Ciclos para amplitud de pulso de parada (1.52ms)
#define COUNT_2MS 1250 //Ciclos para amplitud de pulso de 2ms (max velocidad en el otro sentido)
#define VEL_MEDIA_SUP 1100 //Velocidad media superior
#define VEL_MEDIA_INF 787 //Velocidad media inferior

//Parámetros para la gestión de mecanismos FreeRTOS
#define encoder_giro 0x0000001
#define encoder_avance 0x0000010
#define encoder_choque 0x0000100
#define sensor_contacto 0x0001000
#define caja_localizada 0x0010000
#define palo_localizado 0x0100000
#define sensor_linea 0x1000000

//Variables globales
uint32_t g_ui32CPUUsage;
uint32_t g_ulSystemClock;
static EventGroupHandle_t FlagsEventos;
static QueueHandle_t cola_adc;
static QueueHandle_t cola_encoder;
static TaskHandle_t Manejador_Avanzar;
static TaskHandle_t Manejador_Girar;
static TaskHandle_t Manejador_Choque_Linea;
static TaskHandle_t Manejador_Sensado_Distancia;
int giro_izq = 0; //Las ponemos como constantes globales porque necesito editarlas en varias tareas
int giro_der = 0;
int inc_izq = 0;
int inc_der = 0;
int num_lineas = 0; //Variable para saber cuántas líneas he pasado siempre y cuando esté yendo en dirección al palo y poder seguir avanzando hasta depositar la caja

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
int calculo_sectores_recta(int distancia);
int calculo_sectores_giro(int grados);
int busqueda_distancia(uint32_t A[], uint32_t key, uint32_t imin, uint32_t imax);

/**********************TAREAS***********************/
static portTASK_FUNCTION(Girar, pvParameters){ //Tarea del barrido de la caja y palo
    int angulo_giro = 360; //Giro 360 para buscar la caja o el palo
    EventBits_t respuesta;
    uint32_t dato;

    while(1){
        giro_izq = (calculo_sectores_giro(angulo_giro)); //Calculo los sectores que tienen que girar las ruedas para el ángulo buscado
        giro_der = (calculo_sectores_giro(angulo_giro));

        while((giro_der >= 0) || (giro_izq >= 0)){
            respuesta = xEventGroupWaitBits(FlagsEventos, encoder_giro | caja_localizada | palo_localizado, pdTRUE, pdFALSE, portMAX_DELAY);

            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, VEL_MEDIA_SUP); //Pongo las ruedas a girar de nuevo, por si no lo estaban
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, VEL_MEDIA_SUP);

            if((respuesta & caja_localizada) == caja_localizada){ //Si localizo una caja, cambio de estado y paro de girar, habilito el sensor de contacto
                giro_izq = -1;
                giro_der = -1;
                ADCIntDisable(ADC0_BASE, 1);
                IntEnable(INT_GPIOB);
                est = APROXIMACIÓN_CAJA;
            }
            else if((respuesta & palo_localizado) == palo_localizado){ //Si localizo el palo, cambio de estado y paro de girar y habilito el ADC de larga distancia
                giro_izq = -1;
                giro_der = -1;
                ADCIntDisable(ADC0_BASE, 2);
                est = APROXIMACIÓN_PALO;
            }
            else if((respuesta & encoder_giro) == encoder_giro){ //Si estoy en los estados de barrido, el encoder será este y girarán las ruedas hasta que se complete el giro
                xQueueReceive(cola_encoder, (void*) &dato, portMAX_DELAY);

                if (dato == 68){
                    giro_izq--;
                    giro_der--;
                }
                else if(dato == 4) giro_izq--;
                else if(dato == 64) giro_der--;
                if((giro_izq < 0) && (giro_der < 0)){ //Si llego al final del giro y no encuentro nada, avanzo un poco para volver a buscar posteriormente
                    ADCIntDisable(ADC0_BASE, 1);
                    est = APROXIMACIÓN_CAJA;
                }
            }
        }
    }
}

static portTASK_FUNCTION(Avanzar, pvParameters){ //Tarea de aproximación a cajas y palo
    uint32_t dato;
    int avance = 20; //Avanzo 20 cm en línea recta
    EventBits_t respuesta;

    while(1){
        inc_izq = calculo_sectores_recta(avance); //Calculo los sectores que tienen que girar las ruedas para el avance buscado
        inc_der = calculo_sectores_recta(avance);

        while((inc_der >= 0) || (inc_izq >= 0)){
            respuesta = xEventGroupWaitBits(FlagsEventos, encoder_avance | sensor_contacto, pdTRUE, pdFALSE, portMAX_DELAY);

            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, VEL_MEDIA_SUP); //Si me llega un evento pongo las ruedas a girar hacia el mismo lado
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, VEL_MEDIA_INF);

            if((respuesta & sensor_contacto) == sensor_contacto){ //Si la caja toca el sensor de contacto significa que está dentro y debe buscar el palo
                inc_izq = -1;                                      //Dejo de girar, deshabilito el sensor de contacto y habilito el ADC de larga distancia
                inc_der = -1;
                IntDisable(INT_GPIOB);
                ADCIntEnable(ADC0_BASE, 2);
                est = BARRIDO_PALO;
            }
            else if((respuesta & encoder_avance) == encoder_avance){ //Si estoy en los estados de aproximación el encoder será este y girarán las ruedas
                xQueueReceive(cola_encoder, (void*) &dato, portMAX_DELAY);
                if(num_lineas == 3){
                    inc_izq = 5; //Si acabo de pasar la línea, vaya por la cuenta que vaya, avanzo 5 cm más para depositar la caja en la zona negra
                    inc_der = 5;
                }
                if(dato == 68){
                    inc_izq--;
                    inc_der--;
                }
                else if(dato == 4) inc_izq--;
                else if(dato == 64) inc_der--;
                if((inc_izq < 0) && (inc_der < 0) && (est == APROXIMACIÓN_PALO) && (num_lineas < 3)){
                    inc_izq = calculo_sectores_recta(avance); //Calculo los sectores que tienen que girar las ruedas para el avance buscado
                    inc_der = calculo_sectores_recta(avance);
                }
                else if((inc_izq < 0) && (inc_der < 0) && (est == APROXIMACIÓN_PALO) && (num_lineas == 3)){
                    est = CAJA_COLOCADA; //Si llego a la 3º línea, dejo la caja en su sitio y ya toca retroceder)
                    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, VEL_MEDIA_SUP); //Hago que las ruedas vayan hacia el sentido contrario
                    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, VEL_MEDIA_INF);
                }
                else if((inc_izq < 0) && (inc_der < 0) && (est != APROXIMACIÓN_PALO)){ //Si llego al final del avance y no encuentro la caja, vuelvo al estado de barrido caja para volver a localizarla
                    ADCIntEnable(ADC0_BASE, 1);
                    IntDisable(INT_GPIOB);
                    est = BARRIDO_CAJA;
                }
            }
        }
   }
}

static portTASK_FUNCTION(Choque_Linea, pvParameters){
    uint32_t dato;
    EventBits_t respuesta;
    int avance = 5; //Queremos retroceder 5 cm
    int avance_caja_depositada = 15;
    int angulo_giro = 90; //Queremos girar 90º para evitar la línea de nuevo

    while(1){
        int inc_choque_izq = calculo_sectores_recta(avance); //Calculo los sectores que tienen que girar las ruedas para el avance buscado
        int inc_choque_der = calculo_sectores_recta(avance);
        int giro_choque_izq = calculo_sectores_giro(angulo_giro); //Calculo los sectores que tienen que girar las ruedas para el ángulo buscado
        int giro_choque_der = calculo_sectores_giro(angulo_giro);

        while((inc_choque_der >= 0) || (inc_choque_izq >= 0) || (giro_choque_izq >= 0) || (giro_choque_der >= 0)){
            respuesta = xEventGroupWaitBits(FlagsEventos, encoder_choque | sensor_linea, pdTRUE, pdFALSE, portMAX_DELAY);

            if(((respuesta & sensor_linea) == sensor_linea) && (est != APROXIMACIÓN_PALO)){ //Si el estado es distinto al de aproximación al palo es que me voy a salir
                giro_der = -1;                                                               //Por lo que paro las ruedas de otras tareas y retrocedo
                giro_izq = -1;
                inc_der = -1;
                inc_izq = -1;
                PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, VEL_MEDIA_SUP); //Hago que las ruedas vayan hacia el sentido contrario
                PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, VEL_MEDIA_INF);
                inc_choque_izq = calculo_sectores_recta(avance); //Recalculo todas las funciones de nuevo por si vuelvo a entrar en choque mientras choco (rechoque)
                inc_choque_der = calculo_sectores_recta(avance);
                giro_choque_izq = calculo_sectores_giro(angulo_giro);
                giro_choque_der = calculo_sectores_giro(angulo_giro);
            }
            else if(((respuesta & sensor_linea) == sensor_linea) && (est == APROXIMACIÓN_PALO)){ //Si recibo sensor de línea y estoy aproximandome al palo
                num_lineas++;                                                                     //Aumento la variable para seguir avanzando hasta él
            }
            else if((respuesta & encoder_choque) == encoder_choque){ /*Recibe información del choque y cambia de trayectoria*/
                xQueueReceive(cola_encoder, (void*) &dato, portMAX_DELAY);

                if(num_lineas == 3){ //Si entro aquí significa que acabo de dejar una caja en la zona negra
                    inc_choque_izq = calculo_sectores_recta(avance_caja_depositada); //Recalculo las funciones para retroceder 15 cm
                    inc_choque_der = calculo_sectores_recta(avance_caja_depositada);
                    num_lineas = 0;
                }
                if((inc_choque_izq > 0) && (inc_choque_der > 0)){ //Aquí compruebo que tengo que seguir retrocediendo
                    if(dato == 68){
                        inc_choque_izq--;
                        inc_choque_der--;
                    }
                    else if(dato == 4) inc_choque_izq--;
                    else if(dato == 64) inc_choque_der--;
                    if((inc_choque_izq < 0) && (inc_choque_der < 0)){
                        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, VEL_MEDIA_SUP); //Cambio las ruedas a girar
                        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, VEL_MEDIA_SUP);
                    }
                }
                else if((inc_choque_izq < 0) && (inc_choque_der < 0)){ //Si ya he retrocedido lo suficiente, tengo que ponerme a girar
                    if(dato == 68){
                        giro_choque_izq--;
                        giro_choque_der--;
                    }
                    else if(dato == 4) giro_choque_izq--;
                    else if(dato == 64) giro_choque_der--;
                    if((giro_choque_izq < 0) && (giro_choque_der < 0)){
                        ADCIntEnable(ADC0_BASE, 1);
                        est = BARRIDO_CAJA; //Cuando he terminado de girar, vuelvo a barrer para encontrar la siguiente caja
                    }
                }
            }
        }
    }
}

static portTASK_FUNCTION(Sensado_Distancia, pvParameters){
    uint32_t dato;
    int indice_mediano, indice_largo;
    uint32_t TAMAN = 11;
    uint32_t TAMA = 21;
    uint32_t TMuestras_SM[]   = {598, 623, 697, 770, 840, 990, 1107, 1255, 1454, 1817, 2302};
    uint32_t TDistancias_SM[] = { 26,  24,  22,  20,  18,  16,   14,   12,   10,    8,    6};
    uint32_t TMuestras_SL[]   = {452, 480, 500, 528, 550, 575, 602, 624, 648, 673, 698, 720, 744, 794, 822, 872, 923, 970, 1000, 1063, 1114};
    uint32_t TDistancias_SL[] = { 80,  76,  72,  68,  64,  60,  58,  56,  54,  52,  50,  48,  46,  44,  42,  40,  38,  36,   34,   32,   30};

    while(1){
        xQueueReceive(cola_adc, (void *)&dato, portMAX_DELAY); //Espero al dato, da igual de qué ADC venga (se habilitan y deshabilitan arriba)

        if(est == BARRIDO_CAJA){ //Si estoy barriendo cajas, comparo con los de media distancia y envío tal evento
            indice_mediano = busqueda_distancia(TMuestras_SM, dato, 0, TAMAN);
            if(TDistancias_SM[indice_mediano] >= 10 && TDistancias_SM[indice_mediano] <= 22) xEventGroupSetBits(FlagsEventos, caja_localizada);
        }
        else if(est == BARRIDO_PALO){ //Si estoy barriendo el palo, comparo con los de larga distancia y envío tal evento
            indice_largo = busqueda_distancia(TMuestras_SL, dato, 0, TAMA);
            if(TDistancias_SL[indice_largo] >= 40 && TDistancias_SL[indice_largo] <= 72) xEventGroupSetBits(FlagsEventos, palo_localizado);
        }
    }
}

/*************************RUTINAS DE INTERRUPCIÓN***************************/
void ManejadorBotones(void){
    signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    vTaskSuspend(Manejador_Girar); //Si pulso cualquiera de los dos botones, entonces suspendo todas las tareas, si las quiero reanudar se pulsará el botón de RESET
    vTaskSuspend(Manejador_Avanzar);
    vTaskSuspend(Manejador_Choque_Linea);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT);

    GPIOIntClear(GPIO_PORTF_BASE, ALL_BUTTONS); //limpiamos flags
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

void SensorContacto(void){ //Informa que ya tiene dentro la caja
    signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    xEventGroupSetBitsFromISR(FlagsEventos, sensor_contacto, &higherPriorityTaskWoken); //Si se activa el sensor de contacto, mando tal flag

    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_5);/*Limpia interrupción*/
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

/*Gestiona la interrupción del pin gpio asociado a los encoders de los motores*/
void Encoder(void){
    signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;
    uint32_t dato;

    dato = GPIOIntStatus(GPIO_PORTD_BASE, true); //Veo de qué encoder me llega la interrupción

    if(((dato & GPIO_PIN_1) == GPIO_PIN_1) || ((dato & GPIO_PIN_7) == GPIO_PIN_7)){ //Si son los encoders de línea mando tal flag
        if(est != APROXIMACIÓN_PALO){
            est = CHOQUE_LINEA;
        }
        xEventGroupSetBitsFromISR(FlagsEventos, sensor_linea, &higherPriorityTaskWoken);
    }

    if(((dato & GPIO_PIN_2) == GPIO_PIN_2) || ((dato & GPIO_PIN_6) == GPIO_PIN_6)){ //Si son los encoders de las ruedas, miro en qué estado estoy y envío tal flag
          xQueueSendFromISR(cola_encoder, &dato, &higherPriorityTaskWoken); //Guardamos en la cola

          if((est == BARRIDO_CAJA) || (est == BARRIDO_PALO)) xEventGroupSetBitsFromISR(FlagsEventos, encoder_giro, &higherPriorityTaskWoken);
          else if((est == APROXIMACIÓN_CAJA) || (est == APROXIMACIÓN_PALO)) xEventGroupSetBitsFromISR(FlagsEventos, encoder_avance, &higherPriorityTaskWoken);
          else if((est == CHOQUE_LINEA) || (est == CAJA_COLOCADA)) xEventGroupSetBitsFromISR(FlagsEventos, encoder_choque, &higherPriorityTaskWoken);
    }

    GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_1 | GPIO_INT_PIN_2 | GPIO_INT_PIN_6 | GPIO_INT_PIN_7);
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

void configADC0_ISR(void){                                                                                                                 //PROBAR
    signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    uint32_t dato[4];
    uint32_t dato_final;

    ADCIntClear(ADC0_BASE, 1); //Limpiamos el flag de interrupciones

    ADCSequenceDataGet(ADC0_BASE, 1, (uint32_t*)dato); //Cogemos el dato guardado

    dato_final = dato[0]; //Sólo cojo el primer dato de la cola de mensajes

    xQueueSendFromISR(cola_adc, &dato_final, &higherPriorityTaskWoken); //Enviamos el dato por la cola de mensajes

    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

void configADC1_ISR(void){                                                                                                                  //PROBAR
    signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    uint32_t dato[4];
    uint32_t dato_final;

    ADCIntClear(ADC0_BASE, 2); //Limpiamos el flag de interrupciones

    ADCSequenceDataGet(ADC0_BASE, 2, (uint32_t*)dato); //Cogemos el dato guardado

    dato_final = dato[1]; //Sólo cojo el primer dato de la cola de mensajes

    xQueueSendFromISR(cola_adc, &dato_final, &higherPriorityTaskWoken); //Enviamos el dato por la cola de mensajes

    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

/***************************FUNCIONES_AUXILIARES********************/
int busqueda_distancia(uint32_t A[], uint32_t key, uint32_t imin, uint32_t imax){ //Busco en qué posición estoy del array según el dato insertado en la función
    int imid;

    while(imin < imax){
       imid = (imin + imax) >> 1;
       if(A[imid] < key) imin = imid + 1;
       else imax = imid;
    }
    return imax;    //Al final imax = imin y en dicha posicion hay un numero mayor o igual que el buscado
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
    if((xTaskCreate(Avanzar, (portCHAR *)"Avanzar", Avanzar_STACK, NULL, Avanzar_PRIORITY, &Manejador_Avanzar) != pdTRUE)) while(1);

    if((xTaskCreate(Girar, (portCHAR *)"Girar", Girar_STACK, NULL, Girar_PRIORITY, &Manejador_Girar) != pdTRUE)) while(1);

    if((xTaskCreate(Choque_Linea, (portCHAR *)"Choque_Linea", Choque_Linea_STACK, NULL, Choque_Linea_PRIORITY, &Manejador_Choque_Linea) != pdTRUE)) while(1);

    if((xTaskCreate(Sensado_Distancia, (portCHAR *)"Sensado_Distancia", Sensado_Distancia_STACK, NULL, Sensado_Distancia_PRIORITY, &Manejador_Sensado_Distancia) != pdTRUE)) while(1);

    FlagsEventos = xEventGroupCreate();
    if(FlagsEventos == NULL) while(1);

    cola_adc = xQueueCreate(8, sizeof(uint32_t)); //Creación de la cola de ADC0
    if (cola_adc == NULL) while(1);

    cola_encoder = xQueueCreate(8, sizeof(uint32_t)); //Creación de la cola del encoder
    if (cola_encoder == NULL) while(1);

    SysCtlDelay(3000 * (SysCtlClockGet() / 3 / 1000)); //Esperamos 3 segundos antes de que empiece el programa

    IntDisable(INT_GPIOB); //Deshabilito el pulsador, ya que no lo necesito al comenzar
    ADCIntDisable(ADC0_BASE, 2); //Deshabilito el ADC del sensor de larga distancia, pues no voy a buscar el palo todavía
    ADCIntEnable(ADC0_BASE, 1); //Habilito el ADC del sensor de media distancia, pues lo voy a usar para buscar las cajas

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, VEL_MEDIA_SUP); //Comienzo haciendo que las ruedas giren
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, VEL_MEDIA_SUP);

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
