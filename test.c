#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_pwm.h"
#include "lpc17xx.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_exti.h"
#include "lpc17xx_timer.h"
#include <stdio.h>
#include <string.h>

#define dmaSize 60
#define numSineSample 60
#define sineFreqHz 4000
#define pclkMhz 25
GPDMA_Channel_CFG_Type dmaConfig;
GPDMA_LLI_Type DMA_LLI_Struct;

uint8_t alertMessage1[] = "Pinza cerrada, trabajar con precaucion!";

void configPin(void);
void configPWM(void);
void configADC(void);
void configDAC(void);
void configDMA(void);
void confEINT(void);
void confIntGpio(void);
void configUART(void);
uint8_t servoMapDegrees(uint32_t);
void createSineSignal(void);

#define DMA_SIZE 60
#define NUM_SINE_SAMPLE 60
#define SINE_FREQ_IN_HZ 4000
#define PCLK_DAC_IN_MHZ 25 // CCLK divided by 4

// Variables globales para ver las conversiones de ADC
uint32_t AD0Value = 0;
uint32_t AD1Value = 0;
uint32_t AD2Value = 0;
uint32_t AD4Value = 0;
uint32_t AD5Value = 0;
uint32_t AD6Value = 0;

// logica para promedio movil ADC
uint32_t AD0ValuePromedio = 262; // este es el valor correspondiente al MR1 de
uint32_t AD0ArraySum;
uint32_t AD0Array[20] = {262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262};
uint8_t cont0 = 0;

uint32_t AD1ValuePromedio = 138; // este es el valor correspondiente al MR2 de
uint32_t AD1ArraySum;
uint32_t AD1Array[20] = {138, 138, 138, 138, 138, 138, 138, 138, 138, 138, 138, 138, 138, 138, 138, 138, 138, 138, 138, 138};
uint8_t cont1 = 0;

uint32_t AD2ValuePromedio = 262; // este es el valor correspondiente al MR3 de
uint32_t AD2ArraySum;
uint32_t AD2Array[20] = {262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262};
uint8_t cont2 = 0;

uint32_t AD4ValuePromedio = 200; // este es el valor correspondiente al MR4 de
uint32_t AD4ArraySum;
uint32_t AD4Array[20] = {200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200};
uint8_t cont4 = 0;

uint32_t AD5ValuePromedio = 105; // este es el valor correspondiente al MR5 de
uint32_t AD5ArraySum;
uint32_t AD5Array[30] = {105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105};
uint8_t cont5 = 0;

uint32_t AD6ValuePromedio = 250; // este es el valor correspondiente al MR6 de
uint32_t AD6ArraySum;
uint32_t AD6Array[30] = {262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262, 262};
uint8_t cont6 = 0;


void confDMA(void);
void confDac(void);
GPDMA_Channel_CFG_Type GPDMACfg;
uint32_t dac_sine_lut[NUM_SINE_SAMPLE];

int main(void)
{
    createSineSignal();
    configPin();
    configPWM();
    configADC();
    configDAC();
    configDMA();
    configUART();
    while (1)
        ;
    return 0;
}

/*
    1. createSineSignal: armado de la señal senoidal
    2. configDac: configuracion de pin AOUT e inicializacion de DAC
    3. configDMA: configuracion de DMA
    4. habilitar interrupcion de DMA, channel 0
*/

void configDAC(void)
{
    uint32_t tmp;
    DAC_CONVERTER_CFG_Type DAC_ConverterConfigStruct;
    DAC_ConverterConfigStruct.CNT_ENA = SET;
    DAC_ConverterConfigStruct.DMA_ENA = SET;
    DAC_Init(LPC_DAC);
    /* set time out for DAC*/
    tmp = (PCLK_DAC_IN_MHZ * 1000000) / (SINE_FREQ_IN_HZ * NUM_SINE_SAMPLE);
    DAC_SetDMATimeOut(LPC_DAC, tmp);
    DAC_ConfigDAConverterControl(LPC_DAC, &DAC_ConverterConfigStruct);
    return;
}

void configDMA(void)
{
    DMA_LLI_Struct.SrcAddr = (uint32_t)dac_sine_lut;
    DMA_LLI_Struct.DstAddr = (uint32_t) & (LPC_DAC->DACR);
    DMA_LLI_Struct.NextLLI = (uint32_t)&DMA_LLI_Struct;
    DMA_LLI_Struct.Control = DMA_SIZE | (2 << 18) // source width 32 bit
                             | (2 << 21)          // dest. width 32 bit
                             | (1 << 26)          // source increment
        ;

    GPDMACfg.ChannelNum = 0;
    GPDMACfg.SrcMemAddr = (uint32_t)(dac_sine_lut);
    GPDMACfg.DstMemAddr = 0;
    GPDMACfg.TransferSize = DMA_SIZE;
    GPDMACfg.TransferWidth = 0;
    GPDMACfg.TransferType = GPDMA_TRANSFERTYPE_M2P;
    GPDMACfg.SrcConn = 0;
    GPDMACfg.DstConn = GPDMA_CONN_DAC;
    GPDMACfg.DMALLI = (uint32_t)&DMA_LLI_Struct;
    GPDMA_Init();
    GPDMA_Setup(&GPDMACfg);
    LPC_GPDMA->DMACSync &= ~(1 << 4);
    return;
}

void configUART(void)
{
    UART_CFG_Type UARTConfigStruct;
    UART_FIFO_CFG_Type UARTFIFOConfigStruct;
    UART_ConfigStructInit(&UARTConfigStruct);
    UART_Init(LPC_UART2, &UARTConfigStruct);
    UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);
    UART_FIFOConfig(LPC_UART2, &UARTFIFOConfigStruct);
    UART_TxCmd(LPC_UART2, ENABLE);
    return;
}

void createSineSignal(void)
{
    uint32_t i;
    uint32_t sin_0_to_90_16_samples[16] = {
        0, 1045, 2079, 3090, 4067,
        5000, 5877, 6691, 7431, 8090,
        8660, 9135, 9510, 9781, 9945, 10000};
    for (i = 0; i < NUM_SINE_SAMPLE; i++)
    {
        if (i <= 15)
        {
            dac_sine_lut[i] = 512 + 512 * sin_0_to_90_16_samples[i] / 10000;
            if (i == 15)
                dac_sine_lut[i] = 1023;
        }
        else if (i <= 30)
        {
            dac_sine_lut[i] = 512 + 512 * sin_0_to_90_16_samples[30 - i] / 10000;
        }
        else if (i <= 45)
        {
            dac_sine_lut[i] = 512 - 512 * sin_0_to_90_16_samples[i - 30] / 10000;
        }
        else
        {
            dac_sine_lut[i] = 512 - 512 * sin_0_to_90_16_samples[60 - i] / 10000;
        }
        dac_sine_lut[i] = (dac_sine_lut[i] << 6);
    }
    return;
}

void configPin(void)
{
    // Configuracion (P2.0 - PWM1.1 - P2.5 - PWM1.6)
    PINSEL_CFG_Type pwmPins;
    pwmPins.Portnum = 2;
    pwmPins.Pinmode = PINSEL_PINMODE_TRISTATE;
    pwmPins.Funcnum = 1;
    pwmPins.OpenDrain = PINSEL_PINMODE_NORMAL;
    for (uint8_t i = 0; i < 6; i++)
    {
        pwmPins.Pinnum = i;
        PINSEL_ConfigPin(&pwmPins);
    }
    PINSEL_CFG_Type dacPin;
    dacPin.Funcnum = 2;
    dacPin.OpenDrain = 0;
    dacPin.Pinmode = 0;
    dacPin.Pinnum = 26;
    dacPin.Portnum = 0;
    PINSEL_ConfigPin(&dacPin);
    PINSEL_CFG_Type uartPins;
    uartPins.Funcnum = 1;
    uartPins.OpenDrain = 0;
    uartPins.Pinmode = 0;
    uartPins.Pinnum = 10;
    uartPins.Portnum = 0;
    PINSEL_ConfigPin(&uartPins);
    uartPins.Pinnum = 11;
    PINSEL_ConfigPin(&uartPins);
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
    LPC_PWM1->MR2 = 1350;
    LPC_PWM1->MR3 = 1850;
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
    ADC_Init(LPC_ADC, 60);

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
    LPC_ADC->ADGDR &= LPC_ADC->ADGDR;
    NVIC_EnableIRQ(ADC_IRQn);
}

void ADC_IRQHandler(void)
{
    // Verifico estado de DONE de cada canal
    if (ADC_ChannelGetStatus(LPC_ADC, 0, 1))
    {
        AD0Value = ADC_ChannelGetData(LPC_ADC, 0) / 8;
        AD0Array[cont0] = AD0Value;
        cont0++;
        if (cont0 == 20)
        {
            cont0 = 0;
        }
        for (uint8_t i = 0; i < 20; i++)
        {
            AD0ArraySum += AD0Array[i];
        }
        AD0ValuePromedio = AD0ArraySum / 20;
        AD0ArraySum = 0;
        uint32_t matchPWM1 = (uint32_t)((AD0ValuePromedio / 0.25) + 800);
        uint8_t servo1Degrees = servoMapDegrees(matchPWM1);
        LPC_PWM1->MR1 = matchPWM1;
    }
    else if (ADC_ChannelGetStatus(LPC_ADC, 1, 1))
    {

        AD1Value = ADC_ChannelGetData(LPC_ADC, 1) / 8;
        AD1Array[cont1] = AD1Value;
        cont1++;
        if (cont1 == 20)
        {
            cont1 = 0;
        }
        for (uint8_t i = 0; i < 20; i++)
        {
            AD1ArraySum += AD1Array[i];
        }
        AD1ValuePromedio = AD1ArraySum / 20;
        AD1ArraySum = 0;
        uint32_t matchPWM2 = (uint32_t)((AD1ValuePromedio / 0.25) + 800);
        uint8_t servo2Degrees = servoMapDegrees(matchPWM2);
        LPC_PWM1->MR2 = matchPWM2;
    }
    else if (ADC_ChannelGetStatus(LPC_ADC, 2, 1))
    {

        AD2Value = ADC_ChannelGetData(LPC_ADC, 2) / 8;
        AD2Array[cont2] = AD2Value;
        cont2++;
        if (cont2 == 20)
        {
            cont2 = 0;
        }
        for (uint8_t i = 0; i < 20; i++)
        {
            AD2ArraySum += AD2Array[i];
        }
        AD2ValuePromedio = AD2ArraySum / 20;
        AD2ArraySum = 0;
        uint32_t matchPWM3 = (uint32_t)((AD2ValuePromedio / 0.25) + 800);
        uint8_t servo3Degrees = servoMapDegrees(matchPWM3);
        LPC_PWM1->MR3 = matchPWM3;
    }
    else if (ADC_ChannelGetStatus(LPC_ADC, 4, 1))
    {
        AD4Value = ADC_ChannelGetData(LPC_ADC, 4) / 8;
        AD4Array[cont0] = AD4Value;
        cont4++;
        if (cont4 == 20)
        {
            cont4 = 0;
        }
        for (uint8_t i = 0; i < 20; i++)
        {
            AD4ArraySum += AD4Array[i];
        }
        AD4ValuePromedio = AD4ArraySum / 20;
        AD4ArraySum = 0;
        uint32_t matchPWM4 = (uint32_t)((AD4ValuePromedio / 0.25) + 800);
        uint8_t servo4Degrees = servoMapDegrees(matchPWM4);
        LPC_PWM1->MR4 = matchPWM4;
    }
    else if (ADC_ChannelGetStatus(LPC_ADC, 5, 1))
    {
        AD5Value = ADC_ChannelGetData(LPC_ADC, 5) / 8;

        if (AD5Value < 500)
        {
            AD5Array[cont5] = AD5Value;
            cont5++;
            if (cont5 == 30)
            {
                cont5 = 0;
            }
        }
        for (uint8_t i = 0; i < 30; i++)
        {
            AD5ArraySum += AD5Array[i];
        }
        AD5ValuePromedio = AD5ArraySum / 30;
        AD5ArraySum = 0;
        uint32_t matchPWM5 = (uint32_t)((AD5ValuePromedio / 0.25) + 800);
        uint8_t servo5Degrees = servoMapDegrees(matchPWM5);
        LPC_PWM1->MR5 = matchPWM5;
    }
    else if (ADC_ChannelGetStatus(LPC_ADC, 6, 1))
    {
        AD6Value = ADC_ChannelGetData(LPC_ADC, 6) / 8;
        AD6Array[cont0] = AD6Value;
        cont6++;
        if (cont6 == 30)
        {
            cont6 = 0;
        }
        for (uint8_t i = 0; i < 30; i++)
        {
            AD6ArraySum += AD6Array[i];
        }
        AD6ValuePromedio = AD6ArraySum / 30;
        AD6ArraySum = 0;
        uint32_t matchPWM6 = (uint32_t)((AD6ValuePromedio / 0.25) + 1200);
        uint8_t servo6Degrees = servoMapDegrees(matchPWM6);
        if (AD6ValuePromedio < 200)
        {
            GPDMA_ChannelCmd(0, ENABLE);
            UART_Send(LPC_UART2, alertMessage1, sizeof(alertMessage1), BLOCKING);
        }
        else
        {
            GPDMA_ChannelCmd(0, DISABLE);
            //UART_Send(LPC_UART2, alertMessage2, sizeof(alertMessage2), BLOCKING);
        }
        LPC_PWM1->MR6 = matchPWM6;
    }
    LPC_PWM1->LER = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6);
    LPC_ADC->ADGDR &= LPC_ADC->ADGDR;
    return;
}

uint8_t servoMapDegrees(uint32_t valor_MR)
{
    uint8_t degree = (valor_MR / 7) - 115;
    return degree;
}
