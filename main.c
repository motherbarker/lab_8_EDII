/*
 *  Lab 08 Carlos Daniel Valdez
 *
 */

//**************************************************************************************************************
// Librerías
//**************************************************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
//#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/systick.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"

//**************************************************************************************************************
// Variables Globales
//**************************************************************************************************************
uint32_t i = 0;
uint32_t ui32Period;
uint32_t rgbColor = 2;

//**************************************************************************************************************
// Prototipos de Funciones
//**************************************************************************************************************
void delay(uint32_t msec);
void delay1ms(void);
void Timer0IntHandler(void);
void InitUART0(void);
void UARTIntHandler(void);

//**************************************************************************************************************
// Función Principal
//**************************************************************************************************************
int main(void){
    // Configuración del oscilador externo, usando PLL y frecuencia a 40MHz
    SysCtlClockSet(SYSCTL_OSC_MAIN|SYSCTL_USE_PLL|SYSCTL_SYSDIV_5|SYSCTL_XTAL_16MHZ);

    // Habilitar reloj para el Puerto F, Timer0 y UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configurar pines del led RGB (1,3,2)como salidas
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    // Configurar el Timer0 a 32 bits, periodico
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    // Se calcula el período para el temporizador
    ui32Period = (SysCtlClockGet()) / 2;
    // Establecer el periodo del temporizador
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1);

    // Se habilita la interrupción por el TIMER0A
    IntEnable(INT_TIMER0A);
    // Se establece que exista la interrupción por Timeout
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Se habilitan las interrupciones Globales
    IntMasterEnable();
    // Se habilita el Timer
    TimerEnable(TIMER0_BASE, TIMER_A);

    // Configurar UART0
    InitUART0();

    // Se habilita la interrupción por el TIMER0A
    IntEnable(INT_UART0);

    //**********************************************************************************************************
    // Loop Principal
    //**********************************************************************************************************
    while(1){
    }

}

//**************************************************************************************************************
// Función para hacer delay en milisegundos
//**************************************************************************************************************
void delay(uint32_t msec){
    for (i = 0; i < msec; i++){
        delay1ms();
    }
}
//**************************************************************************************************************
// Función para hacer delay de 1 milisegundos
//**************************************************************************************************************
void delay1ms(void){
    SysTickDisable();
    SysTickPeriodSet(40000);
    SysTickEnable();

    while ((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT) == 0); //Pg. 138

}

//**************************************************************************************************************
// Handler de la interrupción del TIMER 0 - Recordar modificar el archivo tm4c123ght6pm_startup_css.c
//**************************************************************************************************************
void Timer0IntHandler(void)
{
    // Clear the timer interrupt
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // If all RGB are off, load RBG color, else turn off
    if (!GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1)
            && !GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2)
            && !GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3))
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, rgbColor);
    }
    else
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
    }
}

//**************************************************************************************************************
// Inicialización de UART0
//**************************************************************************************************************
void InitUART0(void)
{
    /*Enable the GPIO Port A*/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    /*Enable the peripheral UART Module 0*/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    /* Make the UART pins be peripheral controlled. */
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Sets the configuration of a UART. */
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);

    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}

void UARTIntHandler(void)
{
    char cReceived = 0;  // Variable para guardar el dato, inicializada en 0
    UARTIntClear(UART0_BASE, UART_INT_RX | UART_INT_RT);  // Limpiar bandera de interrupción para recepción y transmisión

    // Solo proceder si hay caracteres disponibles
    if(UARTCharsAvail(UART0_BASE))
    {
        cReceived = UARTCharGetNonBlocking(UART0_BASE);  // Guardar el dato en una variable
        UARTCharPutNonBlocking(UART0_BASE, cReceived);  // Devuelve el dato recibido al transmisor
    }

   switch(cReceived){
    case 'r':
        rgbColor = (rgbColor==2)?0:2;
        break;
    case 'g':
        rgbColor = (rgbColor==8)?0:8;
        break;
    case 'b':
        rgbColor = (rgbColor==4)?0:4;
        break;
   }
}
