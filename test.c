#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_pwm.h"
#include "lpc17xx.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_exti.h"
#include <stdio.h>

/*
 *     	ADC: medimos 6 potenciometros y un sensor de distancia
        DAC: señal de audio variable en frecuencia
        DMA: transferencia de datos M2P a DAC de la señal de audio
        Timers: interrupción para largar conversión ADC
        PWM: servomotores (5)
        Hw de comunicación: Ethernet
 */

/*
 * 1. ADC: tendremos que configurar AD0 a AD5 como entrada analogica (Quedara inhabilitado el uso del PIN AOUT - AD3)
 * 			    - tendremos que utilizar un rate que permita realizar la conversion (200 kHZ no funciona)
 * 			    - mediante el timer extraeremos la muestra
 */

/*
    EINT0 - controlamos modos
   Modo 0 controlado manualmente
   Modo 1 mediante EINT1 ejecuta el movimiento y por DMA manda al DAC una señal de audio por 10 seg

*/

/*
    FALTA MODO 1 y probar MODO 0

*/

void configPin(void);
void configPWM(void);
void configADC(void);
void configUART(void);
void servo_write(uint8_t servo_number, float value);
void configEINT0(void);
void homeState(void);

// Variables globales para ver las conversiones de ADC
uint32_t AD0Value = 0;
uint32_t AD1Value = 0;
uint32_t AD2Value = 0;
uint32_t AD4Value = 0;
uint32_t AD5Value = 0;
uint32_t AD6Value = 0;
uint32_t pwmCount1;
uint32_t pwmCount2;
uint32_t pwmCount3;
uint32_t pwmCount4;
uint32_t pwmCount5;
uint32_t pwmCount6;

int main(void)
{
    configPin();
    configPWM();
    homeState();
    configADC();
    while (1);
    return 0;
}


void homeState(void) {
    // Home State del Robot
    NVIC_EnableIRQ(ADC_IRQn);
    LPC_PWM1 -> MR1 = 1850;
    LPC_PWM1 -> MR2 = 1850;
    LPC_PWM1 -> MR3 = 2500;
    LPC_PWM1 -> MR4 = 1600;
    LPC_PWM1 -> MR5 = 1220;
    LPC_PWM1 -> MR6 = 1800;
    LPC_PWM1->LER = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6);
    return;
}




void configUART(void) {
    PINSEL_CFG_Type UART_pins_config;
    UART_pins_config.Portnum = 0;
    UART_pins_config.Pinnum = 2;
    UART_pins_config.Funcnum = 1;
    UART_pins_config.Pinmode = PINSEL_PINMODE_TRISTATE;
    UART_pins_config.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&UART_pins_config);
    UART_CFG_Type UART_config;
    UART_FIFO_CFG_Type UART_fifo_config;
    UART_ConfigStructInit(&UART_config);
    UART_Init(LPC_UART0, &UART_config);
    UART_FIFOConfigStructInit(&UART_fifo_config);
    UART_FIFOConfig(LPC_UART0, &UART_fifo_config);
    UART_TxCmd(LPC_UART0, ENABLE);
}

void configPin(void)
{
    // Configuracion (P2.0 - PWM1.1 - P2.5 - PWM1.6)
    PINSEL_CFG_Type PWM_pins_config;
    PWM_pins_config.Portnum = 2;
    PWM_pins_config.Pinmode = PINSEL_PINMODE_TRISTATE;
    PWM_pins_config.Funcnum = 1;
    PWM_pins_config.OpenDrain = PINSEL_PINMODE_NORMAL;
    for (uint8_t i = 0; i < 6; i++)
    {
        PWM_pins_config.Pinnum = i;
        PINSEL_ConfigPin(&PWM_pins_config);
    }
    return;
}

void configPWM(void)
{
    // Configuracion
    CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_PWM1, CLKPWR_PCLKSEL_CCLK_DIV_1);
    LPC_PWM1->PCR = 0x0; // Single edge PWM para 6 CH
    LPC_PWM1->PR = 99;
    LPC_PWM1->MR0 = 20000; 
    LPC_PWM1->MR1 = 1850; 
    LPC_PWM1->MR2 = 1850; 
    LPC_PWM1->MR3 = 2500; 
    LPC_PWM1->MR4 = 1600; 
    LPC_PWM1->MR5 = 1220; 
    LPC_PWM1->MR6 = 1800;
    LPC_PWM1->MCR = (1 << 1);                                                                   // Reset PWM TC on PWM1MR0 match
    LPC_PWM1->LER = (1 << 1) | (1 << 0) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6); // update values in MR0 and MR1
    LPC_PWM1->PCR = (1 << 9) | (1 << 10) | (1 << 11) | (1 << 12) | (1 << 13) | (1 << 14);       // enable PWM outputs
    LPC_PWM1->TCR = 3;                                                                          // Reset PWM TC & PR
    LPC_PWM1->TCR &= ~(1 << 1);                                                                 // libera la cuenta
    LPC_PWM1->TCR |= (1 << 3);                                                                  // enable counters and PWM Mode
}

void configADC(void)
{
    // CONNFIGURACION DE PINES P0.23, 24, 25, 26, P1.30 y P1.31 como pines ANALOGICOS
    /*
     * P0.23 -> AD0
     * P0.24 -> AD1
     * P0.25 -> AD2
     * P1.30 -> AD4
     * P1.31 -> AD5
     * P0.2  -> AD6
     */
    PINSEL_CFG_Type potenciometros;

    // configuring port 0 pins, 23 24 25 y 2 - AD0, AD1, AD2 y AD6
    potenciometros.Portnum = 0;
    potenciometros.Pinnum = 23;
    potenciometros.Funcnum = 1;
    potenciometros.Pinmode = PINSEL_PINMODE_TRISTATE;
    potenciometros.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&potenciometros);
    potenciometros.Pinnum = 24;
    PINSEL_ConfigPin(&potenciometros);
    potenciometros.Pinnum = 25;
    PINSEL_ConfigPin(&potenciometros);
    potenciometros.Pinnum = 26;
    PINSEL_ConfigPin(&potenciometros);
    potenciometros.Funcnum = 2;
    potenciometros.Pinnum = 3;
    PINSEL_ConfigPin(&potenciometros);

    // configuring port 1 pins, 30 y 31 - AD4 y AD5
    potenciometros.Portnum = 1;
    potenciometros.Pinnum = 30;
    potenciometros.Funcnum = 3;
    potenciometros.Pinmode = PINSEL_PINMODE_TRISTATE;
    potenciometros.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&potenciometros);
    potenciometros.Pinnum = 31;
    PINSEL_ConfigPin(&potenciometros);

    // ---------------- CONFIGURACION DE ADC  ---------------------
    //      Channels a utilizar son los 0, 1, 2, 4, 5 y 6

    // Configuracion del CLOCK del ADC
    LPC_SC->PCLKSEL0 |= (3 << 24); // PCLK_ADC = CCLK/8

    // fs = 120 Hz , Ts = 8.33 mseg -> 6 Channels cada 50 ms, sampling rate -> 1/20 (20 muestras por segundo)
    ADC_Init(LPC_ADC, 120);

    // ADC interrupt configuration
    ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_ADINTEN1, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_ADINTEN2, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_ADINTEN4, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_ADINTEN5, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_ADINTEN6, ENABLE);

    // Enable ADC channel number
    ADC_ChannelCmd(LPC_ADC, 0, ENABLE);
    ADC_ChannelCmd(LPC_ADC, 1, ENABLE);
    ADC_ChannelCmd(LPC_ADC, 2, ENABLE);
    ADC_ChannelCmd(LPC_ADC, 4, ENABLE);
    ADC_ChannelCmd(LPC_ADC, 5, ENABLE);
    ADC_ChannelCmd(LPC_ADC, 6, ENABLE);

    // ADC Burst mode setting
    ADC_BurstCmd(LPC_ADC, 1);

    // Enable interupts for ADC
    NVIC_EnableIRQ(ADC_IRQn);
}

void ADC_IRQHandler(void)
{
    float value_volt;
    // Verifico estado de DONE de cada canal
    if (ADC_ChannelGetStatus(LPC_ADC, 0, 1))
    {
        AD0Value = ADC_ChannelGetData(LPC_ADC, 0);
        pwmCount1 = (uint32_t)((AD0Value / 1.7) + 700);
        // LPC_PWM1 -> MR1 = (uint32_t)((AD0Value/1.7)+700);
        LPC_PWM1->MR1 = pwmCount1;
        char buffer[32];
        sprintf(buffer, "A%d\r\n", pwmCount1);
        UART_Send(LPC_UART0, (uint8_t *)buffer, strlen(buffer), BLOCKING);
    }
    else if (ADC_ChannelGetStatus(LPC_ADC, 1, 1))
    {
        AD1Value = ADC_ChannelGetData(LPC_ADC, 1);
        pwmCount2 = (uint32_t)((AD1Value / 1.9) + 800);
        LPC_PWM1->MR2 = pwmCount2;
        char buffer[32];
        sprintf(buffer, "B%d\r\n", pwmCount2);
        UART_Send(LPC_UART0, (uint8_t *)buffer, strlen(buffer), BLOCKING);
        // LPC_PWM1 -> MR2 = (uint32_t)((AD1Value/1.9)+800);
    }
    else if (ADC_ChannelGetStatus(LPC_ADC, 2, 1))
    {
        AD2Value = ADC_ChannelGetData(LPC_ADC, 2);
        pwmCount3 = (uint32_t)((AD2Value / 1.6) + 900);
        LPC_PWM1->MR3 = pwmCount3;
        char buffer[32];
        sprintf(buffer, "C%d\r\n", pwmCount3);
        UART_Send(LPC_UART0, (uint8_t *)buffer, strlen(buffer), BLOCKING);
        // LPC_PWM1 -> MR3 = (uint32_t)((AD2Value/1.6)+900);
    }
    else if (ADC_ChannelGetStatus(LPC_ADC, 4, 1))
    {
        AD4Value = ADC_ChannelGetData(LPC_ADC, 4);
        pwmCount4 = (uint32_t)((AD4Value / 1.6) + 1400);
        LPC_PWM1->MR4 = pwmCount4;
        char buffer[32];
        sprintf(buffer, "D%d\r\n", pwmCount4);
        UART_Send(LPC_UART0, (uint8_t *)buffer, strlen(buffer), BLOCKING);
        // LPC_PWM1 -> MR4 = 1600;
    }
    else if (ADC_ChannelGetStatus(LPC_ADC, 5, 1))
    {
        AD5Value = ADC_ChannelGetData(LPC_ADC, 5);
        pwmCount5 = (uint32_t)((AD5Value / 2.4) + 740);
        LPC_PWM1->MR5 = pwmCount5;
        char buffer[32];
        sprintf(buffer, "E%d\r\n", pwmCount5);
        UART_Send(LPC_UART0, (uint8_t *)buffer, strlen(buffer), BLOCKING);
        // LPC_PWM1 -> MR5 = (uint32_t)((AD5Value/2.4)+740);
    }
    else if (ADC_ChannelGetStatus(LPC_ADC, 6, 1))
    {
        AD6Value = ADC_ChannelGetData(LPC_ADC, 6);
        pwmCount6 = (uint32_t)((AD6Value / 2.04) + 1800);
        LPC_PWM1->MR6 = pwmCount6;
        char buffer[32];
        sprintf(buffer, "F%d\r\n", pwmCount6);
        UART_Send(LPC_UART0, (uint8_t *)buffer, strlen(buffer), BLOCKING);
        // LPC_PWM1 -> MR6 = (uint32_t)((AD6Value/2.04)+1800);
    }

    LPC_PWM1->LER = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6);
    return;
}

void servo_write(uint8_t servo_number, float value)
{
    // Conversion volt a PWM
    float constante = 303.03;
    uint32_t convertido = (uint32_t)(value * constante + 1000);
    switch (servo_number)
    {
    case 1:
        LPC_PWM1->MR1 = convertido;
        break;
    case 2:
        LPC_PWM1->MR2 = convertido;
        break;
    case 3:
        LPC_PWM1->MR3 = convertido;
        break;
    case 4:
        LPC_PWM1->MR4 = convertido;
        break;
    case 5:
        LPC_PWM1->MR5 = convertido;
        break;
    case 6:
        LPC_PWM1->MR6 = convertido;
        break;
    default:
        break;
    }
    LPC_PWM1->LER = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6);

    return;
}


// Codigo que lee una parte de memoria y la manda por DMA al DAC para reproducir una señal de audio
// Necesito crear el .py que genere el codigo. Ademas declarar un buffer de 1024 bytes que tenga la señal de audio de una cancion (Tengo que observar el formato de la señal de audio)
void confDMA(void){
    DMA_InitTypeDef DMA_InitStruct;
    DMA_InitStruct.DMA_Channel = 0;
    DMA_InitStruct.DMA_DestAddr = (uint32_t) &(LPC_DAC->DACR);
    DMA_InitStruct.DMA_SrcAddr = (uint32_t) & (buffer);

    return;
}

// Configuracion de EINT0 para controlar los modos
void confEINT0(void){

    // En este codigo tengo que configurar el EINT0 para que cuando se presione el boton de la placa, se cambie el modo de funcionamiento del robot 
    // de mmanual a automatico y viceversa.

    return;
}

void confEINT1(void){

    // En este codigo tengo que configurar el EINT1 para que cuando se presione el boton de la placa, se ejecute el movimiento del robot y se mande la señal de audio por DMA al DAC

    return;
}

void EINT0_IRQHandler(void){

    // En este codigo tengo que configurar el EINT0 para que cuando se presione el boton de la placa, se cambie el modo de funcionamiento del robot 
    // de manual a automatico y viceversa. Con una variable global puedo saber en que modo esta el robot, si esta en cero es manual, si esta en uno es automatico.
    // Modo manual: se mueve el robot con los potenciometros y el ADC esta activo. DMA e EINT1 desactivados.
    // Modo automatico: se mueve el robot con la activacion de EINT1 y el DMA esta activo. 

    return;
}