#include "LPC17xx.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_pwm.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_exti.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpdma.h"
#include <stdio.h>
#include <string.h>

#define dmaSize 60
#define numSineSample 60
#define sineFreqHz 4000
#define pclkDacMHz 25

uint32_t pwmMin = 500;
uint32_t pwmMax = 2400;
uint16_t adcMin;
uint16_t adcMax;
uint32_t dacSineLut[numSineSample];
uint32_t ADValue;
uint32_t pwmWanted;
uint32_t wantedADCCheck;
GPDMA_Channel_CFG_Type dmaConfig;

void configPin(void);
void configADC(void);
void configDMA(void);
void configDAC(void);
void configPWM(void);
void configSineDac(void);
void configEINT(void);
void autoMap(void);
void delay(uint32_t);
void writePWMPosition(uint32_t);
uint32_t pwmMap(uint32_t);

int main(void)
{
    configPin();
    configADC();
    configPWM();
    configSineDac();
    configDMA();
    configDAC();

    autoMap();
    uint32_t wantedADC = 120;	// lo cargamos por soft -> el valor del ADC despues se le hace regla de 3 p/ llegar al Vout deseado
    pwmWanted = pwmMap(wantedADC);	//da el valor de PWM que tendria que poner el pote al valor de Vout deseado
    writePWMPosition(pwmWanted);
    //writePWMPosition(pwmMin);
    //chequeo que vaya a donde quise
    delay(500);
    if (ADC_ChannelGetStatus(LPC_ADC, 0, 1))
        {
    		wantedADCCheck = ADC_ChannelGetData(LPC_ADC, 0) /16;
        }

    while (1);

    return 0;
}







void configPin(void)
{
    PINSEL_CFG_Type pwmPin;
    pwmPin.Portnum = 2;
    pwmPin.Pinmode = PINSEL_PINMODE_TRISTATE;
    pwmPin.Funcnum = 1;
    pwmPin.OpenDrain = PINSEL_PINMODE_NORMAL;
    pwmPin.Pinnum = 0;
    PINSEL_ConfigPin(&pwmPin);
    PINSEL_CFG_Type analogPin;
    analogPin.Portnum = 0;
    analogPin.Pinnum = 23;
    analogPin.Funcnum = 1;
    analogPin.Pinmode = PINSEL_PINMODE_TRISTATE;
    analogPin.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&analogPin);
    PINSEL_CFG_Type eintPin;
    eintPin.Portnum = 2;
    eintPin.Pinnum = 10;
    eintPin.Funcnum = 1;
    eintPin.Pinmode = PINSEL_PINMODE_PULLUP;
    eintPin.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&eintPin);
    PINSEL_CFG_Type dacPin;
    dacPin.Funcnum = 2;
    dacPin.OpenDrain = 0;
    dacPin.Pinmode = 0;
    dacPin.Pinnum = 26;
    dacPin.Portnum = 0;
    PINSEL_ConfigPin(&dacPin);
    return;
}

void configADC(void)
{
    ADC_Init(LPC_ADC, 2000);
    ADC_ChannelCmd(LPC_ADC, 0, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, DISABLE);
    ADC_BurstCmd(LPC_ADC, 1);
    LPC_ADC->ADGDR &= LPC_ADC->ADGDR;
    //NVIC_EnableIRQ(ADC_IRQn);
    //LPC_GPDMA->DMACSync &= ~(1 << 4);
    return;
}

void configPWM(void)
{
    CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_PWM1, CLKPWR_PCLKSEL_CCLK_DIV_1);
    LPC_PWM1->PCR = 0x0;
    LPC_PWM1->PR = 99;
    LPC_PWM1->MR0 = 20000;
    LPC_PWM1->MR1 = 1500;
    LPC_PWM1->MCR = (1 << 1);
    LPC_PWM1->LER |= (1 << 0) | (1 << 1);
    LPC_PWM1->PCR |= (1 << 9);
    LPC_PWM1->TCR = 3;
    LPC_PWM1->TCR &= ~(1 << 1);
    LPC_PWM1->TCR |= (1 << 3);
    return;
}


void configSineDac(void)
{
    uint32_t i;
    uint32_t sinFirstSamples[16] = {
        0, 1045, 2079, 3090, 4067,
        5000, 5877, 6691, 7431, 8090,
        8660, 9135, 9510, 9781, 9945, 10000};
    for (i = 0; i < numSineSample; i++)
    {
        if (i <= 15)
        {
            dacSineLut[i] = 512 + 512 * sinFirstSamples[i] / 10000;
            if (i == 15)
                dacSineLut[i] = 1023;
        }
        else if (i <= 30)
        {
            dacSineLut[i] = 512 + 512 * sinFirstSamples[30 - i] / 10000;
        }
        else if (i <= 45)
        {
            dacSineLut[i] = 512 - 512 * sinFirstSamples[i - 30] / 10000;
        }
        else
        {
            dacSineLut[i] = 512 - 512 * sinFirstSamples[60 - i] / 10000;
        }
        dacSineLut[i] = (dacSineLut[i] << 6);
    }
}


void configDMA(void)
{
    GPDMA_LLI_Type dmaLLIStruct;
    dmaLLIStruct.SrcAddr = (uint32_t)dacSineLut;
    dmaLLIStruct.DstAddr = (uint32_t) & (LPC_DAC->DACR);
    dmaLLIStruct.NextLLI = (uint32_t)&dmaLLIStruct;
    dmaLLIStruct.Control = dmaSize | (2 << 18) | (2 << 21) | (1 << 26);
    GPDMA_Init();
    dmaConfig.ChannelNum = 0;
    dmaConfig.SrcMemAddr = (uint32_t)(dacSineLut);
    dmaConfig.DstMemAddr = 0;
    dmaConfig.TransferSize = dmaSize;
    dmaConfig.TransferWidth = 0;
    dmaConfig.TransferType = GPDMA_TRANSFERTYPE_M2P;
    dmaConfig.SrcConn = 0;
    dmaConfig.DstConn = GPDMA_CONN_DAC;
    dmaConfig.DMALLI = (uint32_t)&dmaLLIStruct;
    GPDMA_Setup(&dmaConfig);
    return;
}


void configDAC(void)
{
    uint32_t timeout;
    DAC_CONVERTER_CFG_Type dacConfigStruct;
    dacConfigStruct.CNT_ENA = SET;
    dacConfigStruct.DMA_ENA = SET;
    DAC_Init(LPC_DAC);
    timeout = (pclkDacMHz * 1000000) / (sineFreqHz * numSineSample);
    DAC_SetDMATimeOut(LPC_DAC, timeout);
    DAC_ConfigDAConverterControl(LPC_DAC, &dacConfigStruct);
    return;
}



void configEINT(void)
{
    EXTI_SetMode(EXTI_EINT0, EXTI_MODE_EDGE_SENSITIVE);
    EXTI_SetPolarity(EXTI_EINT0, EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE);
    NVIC_EnableIRQ(EINT0_IRQn);
    return;
}


void autoMap(void)
{
	delay(500);
    LPC_PWM1->MR1 = pwmMin;
    LPC_PWM1->LER |= (1 << 1);
    delay(100);
    if (ADC_ChannelGetStatus(LPC_ADC, 0, 1))
    {
        adcMin = ADC_ChannelGetData(LPC_ADC, 0) / 16;
    }
    delay(500);
    LPC_PWM1->MR1 = pwmMax;
    LPC_PWM1->LER |= (1 << 1);
    delay(100);
    if (ADC_ChannelGetStatus(LPC_ADC, 0, 1))
    {
        adcMax = ADC_ChannelGetData(LPC_ADC, 0) /16;
    }
    return;
}

uint32_t pwmMap(uint32_t adcValue)
{
	static uint32_t m,b,pwm2Write;
    m = (pwmMax-pwmMin) / (adcMax - adcMin);
    b = pwmMin - m * adcMin;
    pwm2Write = (uint32_t)(m * adcValue + b);
    return pwm2Write;
}

void writePWMPosition(uint32_t pwmValue)
{
/*    if (LPC_PWM1->MR1 > pwmValue)
    {
        for (uint32_t i = LPC_PWM1->MR1; i > pwmValue; i -= 5)
        {
            LPC_PWM1->MR1 = i;
            LPC_PWM1->LER |= (1 << 1);
            delay(10);
        }
    }
    else
    {
        for (uint32_t i = pwmValue; i < LPC_PWM1->MR1; i += 5)
        {
            LPC_PWM1->MR1 = i;
            LPC_PWM1->LER |= (1 << 1);
            delay(10);
        }
    }
*/
	LPC_PWM1->MR1 = pwmValue;
	LPC_PWM1->LER |= (1 << 1);
    GPDMA_ChannelCmd(0, ENABLE);
    return;
}

void delay(uint32_t ms)
{
    uint32_t ticks = ms * 100000;
    for (uint32_t i = 0; i < ticks; i++)
        ;
    return;
}
