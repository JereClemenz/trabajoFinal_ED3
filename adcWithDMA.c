#include "LPC17xx.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpdma.h"

#define LISTSIZE 50

void configTimer(void);
void configADC(void);
void configPin(void);
void configDMA(void);

__IO uint16_t ADC0Value = 0;
__IO uint16_t listADC[LISTSIZE] = {0};
__IO uint32_t *samples_count = (__IO uint32_t *)0x2007C000;
GPDMA_LLI_Type list;
GPDMA_Channel_CFG_Type dmaCFG;

int main(){
	configTimer();
	configADC();
	configDMA();
}


void configDMA(void){
	list.SrcAddr = (uint32_t) &(LPC_ADC->ADDR0);
	list.DstAddr = (uint32_t) &listADC;
	list.NextLLI = (uint32_t) &list;
	list.Control = LISTSIZE | (2<<18) | (2<<21) | (1<<27);
	GPDMA_Init();
	dmaCFG.ChannelNum			= 0;
	dmaCFG.TransferSize			= 4095;
	dmaCFG.TransferWidth		= 0;
	dmaCFG.TransferType			= GPDMA_TRANSFERTYPE_M2P;
	dmaCFG.SrcConn				= 0;
	dmaCFG.DstConn				= GPDMA_CONN_DAC;
	dmaCFG.SrcMemAddr			= (uint32_t) listADC;
	dmaCFG.DstMemAddr			= 0;
	dmaCFG.DMALLI				= (uint32_t) &list;
	GPDMA_Init();
	GPDMA_Setup(&dmaCFG);
	GPDMA_ChannelCmd(0, ENABLE);
	return;
}

void configADC(void){
	configPin();
	ADC_Init(LPC_ADC, 200000);
	ADC_StartCmd(LPC_ADC, ADC_START_ON_MAT01);
	ADC_ChannelCmd(LPC_ADC, 0, ENABLE);
	ADC_EdgeStartConfig(LPC_ADC, ADC_START_ON_FALLING);
	ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, ENABLE);
}

void configPin(void){
	PINSEL_CFG_Type pinCfg;
	pinCfg.Funcnum = PINSEL_FUNC_1;
	pinCfg.OpenDrain = 0;
	pinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	pinCfg.Pinnum = PINSEL_PIN_23;
	pinCfg.Portnum = PINSEL_PORT_0;
	PINSEL_ConfigPin(&pinCfg);
}

void configTimer(void){
	TIM_TIMERCFG_Type configTimer;
	configTimer.PrescaleOption = TIM_PRESCALE_TICKVAL;
	configTimer.PrescaleValue = 2500; 
	TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &configTimer);
	TIM_MATCHCFG_Type configMatch;
	configMatch.ExtMatchOutputType = TIM_EXTMATCH_TOGGLE;
	configMatch.IntOnMatch = DISABLE;
	configMatch.MatchChannel = 1;
	configMatch.MatchValue = 1000;
	configMatch.ResetOnMatch = ENABLE;
	configMatch.StopOnMatch = DISABLE;
	TIM_ConfigMatch(LPC_TIM0, &configMatch);
	TIM_Cmd(LPC_TIM0, ENABLE);
}