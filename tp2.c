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

#define DMA_SIZE 60
#define NUM_SINE_SAMPLE 60
#define SINE_FREQ_IN_HZ 50 
#define PCLK_DAC_IN_MHZ 25 //CCLK divided by 4
void confPin(void);
void confDMA(void);
void confDac(void);
GPDMA_Channel_CFG_Type GPDMACfg;
uint32_t dac_sine_lut[NUM_SINE_SAMPLE];
// defino MACROS

// defino variables globales
uint32_t pwm_Vout_min = 900;
uint32_t pwm_Vout_max = 2200;

// defino funciones
void configPin(void); //
void configADC(void); //
void confEINT(void);
void delay(void);
void ConfigTIMER(void);
void configPWM(void);

// ADC
uint32_t ADValue;

// EINT
uint8_t modo_interrupcion = 0;

// Control
/*
    Por UART recibo un valor entre 0 y 512 (2^12/8 del ADC)
    Si 3.3V -> 512
    Si 0V -> 0
    El setpoint en voltajes es: setpoint = (ADValue*3.3)/512

    // Inicialmente en la prueba de control, el setpoint es 1.65V (512/2 = 256)
*/
uint16_t setpoint = 256;
int error_control = 0;

uint8_t dmaFlag = 0;

int main(void)
{
	uint32_t i;
	uint32_t sin_0_to_90_16_samples[16]={\
			0,1045,2079,3090,4067,\
			5000,5877,6691,7431,8090,\
			8660,9135,9510,9781,9945,10000\
	};
	confPin();
	confDac();
	//Prepare DAC sine look up table
	for(i=0;i<NUM_SINE_SAMPLE;i++)
	{
		if(i<=15)
		{
			dac_sine_lut[i] = 512 + 512*sin_0_to_90_16_samples[i]/10000;
			if(i==15) dac_sine_lut[i]= 1023;
		}
		else if(i<=30)
		{
			dac_sine_lut[i] = 512 + 512*sin_0_to_90_16_samples[30-i]/10000;
		}
		else if(i<=45)
		{
			dac_sine_lut[i] = 512 - 512*sin_0_to_90_16_samples[i-30]/10000;
		}
		else
		{
			dac_sine_lut[i] = 512 - 512*sin_0_to_90_16_samples[60-i]/10000;
		}
		dac_sine_lut[i] = (dac_sine_lut[i]<<6);
	}
		confDMA();


	//SystemInit();
	configPWM();
    configADC();
    confEINT();
    ConfigTIMER();

    while (1){

    }
    return 0;
}




void configPin(void)
{
	// Configuracion pin PWM (P2.1 -> PWM1.1)
	PINSEL_CFG_Type pwmPins;
	pwmPins.Portnum = 2;
	pwmPins.Pinmode = PINSEL_PINMODE_TRISTATE;
	pwmPins.Funcnum = 1;
	pwmPins.OpenDrain = PINSEL_PINMODE_NORMAL;
	pwmPins.Pinnum = 0;
	PINSEL_ConfigPin(&pwmPins);


    // Configuracion pin ADC (P0.23 -> AD0)
    PINSEL_CFG_Type potenciometro;
    potenciometro.Portnum = 0;
    potenciometro.Pinnum = 23;
    potenciometro.Funcnum = 1;
    potenciometro.Pinmode = PINSEL_PINMODE_TRISTATE; //sin resis
    potenciometro.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&potenciometro);

    // Configuracion de pin para EINT0
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 2;
    PinCfg.Pinnum = 10;
    PinCfg.Funcnum = 1;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP; //tiene pullup interno -> detecta flanco de bajada
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&PinCfg);

    // Pin para Buzzer P0.9
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 9;
    PinCfg.Funcnum = 0;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&PinCfg);

    LPC_GPIO0->FIOMASK &= ~(1<<9); // P0.9 como salida
    LPC_GPIO0->FIODIR |= (1<<9); // P0.9 como salida
    LPC_GPIO0->FIOCLR |= (1<<9); // P0.9 en 0

    PINSEL_CFG_Type PinCofg;
	/*
	 * Init DAC pin connect
	 * AOUT on P0.26
	 */
	PinCofg.Funcnum = 2;
	PinCofg.OpenDrain = 0;
	PinCofg.Pinmode = 0;
	PinCofg.Pinnum = 26;
	PinCofg.Portnum = 0;
	PINSEL_ConfigPin(&PinCofg);

    return;
}

void configADC(void){

    // Configuracion de Registro ADC
    LPC_SC->PCLKSEL0 |= (3 << 24); // PCLK_ADC = CCLK/8
    ADC_Init(LPC_ADC, 200); // 200 Hz

    ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, ENABLE); //habilito interrupcion por ADC0
    ADC_ChannelCmd(LPC_ADC, 0, ENABLE); // habilito canal 0

    // ADC Burst mode setting
    ADC_BurstCmd(LPC_ADC, 1);

    // Enable interupts for ADC
    LPC_ADC->ADGDR &= LPC_ADC->ADGDR;
    NVIC_EnableIRQ(ADC_IRQn);
    LPC_GPDMA->DMACSync &= ~(1 << 4);

    return;
}


void ADC_IRQHandler(void)
{

    // Verifico estado de DONE de cada canal
    if (ADC_ChannelGetStatus(LPC_ADC, 0, 1))
    {
        ADValue = ADC_ChannelGetData(LPC_ADC, 0)/8;
    }
    LPC_ADC->ADGDR &= LPC_ADC->ADGDR;
    return;
}

void confEINT(void){

	// For EINT0
	EXTI_SetMode(EXTI_EINT0,EXTI_MODE_EDGE_SENSITIVE);
	EXTI_SetPolarity(EXTI_EINT0,EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE);

	// Configuraciones iniciales
	modo_interrupcion = 0; // modo_interrupcion = 0 manual
	NVIC_EnableIRQ(EINT0_IRQn);

	return;
}

void EINT0_IRQHandler(void){
	delay();
    if(modo_interrupcion == 0){
        modo_interrupcion = 1;
        LPC_GPIO0->FIOSET |= (1<<9);
    }
    else{
        modo_interrupcion = 0;
        LPC_GPIO0->FIOCLR |= (1<<9);
    }

	// Limpio bandera de EINT0
	LPC_SC->EXTINT |= (1<<0);
	return;
}

void delay(void){
	for(uint32_t i = 0; i<1000000 ; i++);
}

void configPWM(void){

    // Configuracion
    CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_PWM1, CLKPWR_PCLKSEL_CCLK_DIV_1);

    LPC_PWM1->PCR = 0x0; // Single edge PWM para 6 CH
    LPC_PWM1->PR = 99;
    LPC_PWM1->MR0 = 20000;
    LPC_PWM1->MR1 = 1850;
    LPC_PWM1->MCR = (1 << 1);                                                                   // Reset PWM TC on PWM1MR0 match
    LPC_PWM1->LER |= (1 << 2) | (1 << 0);                                                       // update values in MR0 and MR1
    LPC_PWM1->PCR |= (1 << 9);                                                                  // enable PWM1 output
    LPC_PWM1->TCR = 3;                                                            // Reset PWM TC & PWM PR
    LPC_PWM1->TCR &= ~(1 << 1);                                                                 // libera la cuenta
    LPC_PWM1->TCR |= (1 << 3);                                                                  // enable counters and PWM Mode

    return;
}



void ConfigTIMER(void){
    // Configuracion de TIMER0 para control de servo. Interrupcion cada 1ms
    LPC_SC->PCONP |= (1<<1); // Enciendo Timer0
    LPC_SC->PCLKSEL0 |= (1<<2); // PCLK = CCLK

    /*
        1 ms = (MR0+1)*(PR+1)/PCLK
        -> MR0 = 1ms*PCLK/(PR+1) - 1
        PR = 0 -> MR0 = 1ms*PCLK - 1 = 100000 - 1 = 99999
    */
    LPC_TIM0->PR = 0; // Prescaler = 1000
    LPC_TIM0->MR0 = 99999; // Match Register = 1000

    LPC_TIM0->MCR |= (1<<0)|(1<<1); // Interrupcion y reset on MR0

    LPC_TIM0->TCR |= 3; // Reseteo el Timer0
    LPC_TIM0->TCR &=~(1<<1); // Habilito el Timer0

    // Limpio bandera del Timer
    LPC_TIM0->IR |= (1<<0);
    NVIC_EnableIRQ(TIMER0_IRQn);
    return;
}

void TIMER0_IRQHandler(void){
	/*
        - Este handler es el control del potenciometro y se ejecuta cada 1ms (1000Hz)
            -Si el valor sensado por el adc es menor al setpoint, se aumenta el duty cycle
            -Si el valor sensado por el adc es mayor al setpoint, se disminuye el duty cycle
        - Los valores del MR estan entre 900 y 2200 (1ms y 2ms) para que el servo se mueva entre 0 y 180 grados
        - Si el valor del ADValue-Setpoint es +- 10 (por error de medicion) no se hace nada porque llega a la
          posicion deseada
   error_control =  setpoint - ADValue ;
    /*
        Si el error es mayor a 10, se aumenta el duty cycle
        Si el error es menor a -10, se disminuye el duty cycle
    */
    if(error_control > 10){
        if(LPC_PWM1->MR1 < pwm_Vout_max){
            LPC_PWM1->MR1 += 1;
            /*if (dmaFlag) {
                GPDMA_ChannelCmd(0, DISABLE);
            }*/
            dmaFlag = 0;
            //LPC_PWM1->LER |= (1<<1);
        }
    }
    else if(error_control < -10){
        if(LPC_PWM1->MR1 > pwm_Vout_min){
            LPC_PWM1->MR1 -= 1;
            /*if (dmaFlag) {
                GPDMA_ChannelCmd(0, DISABLE);
            }
            dmaFlag = 0;*/
            //LPC_PWM1->LER |= (1<<1);
        }
    }
    else{
        // No hago nada
    	// Enable GPDMA channel 0
       /* if (!dmaFlag) {
            GPDMA_ChannelCmd(0, ENABLE);
        }
        dmaFlag = 1;*/
    }

    LPC_PWM1->LER |= (1<<1);

    // Limpio bandera del Timer
    LPC_TIM0->IR |= (1<<0);
	return;
}

void confDMA(void){
	GPDMA_LLI_Type DMA_LLI_Struct;
	//Prepare DMA link list item structure
	DMA_LLI_Struct.SrcAddr= (uint32_t)dac_sine_lut;
	DMA_LLI_Struct.DstAddr= (uint32_t)&(LPC_DAC->DACR);
	DMA_LLI_Struct.NextLLI= (uint32_t)&DMA_LLI_Struct;
	DMA_LLI_Struct.Control= DMA_SIZE
			| (2<<18) //source width 32 bit
			| (2<<21) //dest. width 32 bit
			| (1<<26) //source increment
			;
	/* GPDMA block section -------------------------------------------- */
	/* Initialize GPDMA controller */
	GPDMA_Init();
	// Setup GPDMA channel --------------------------------
	// channel 0
	GPDMACfg.ChannelNum = 0;
	// Source memory
	GPDMACfg.SrcMemAddr = (uint32_t)(dac_sine_lut);
	// Destination memory - unused
	GPDMACfg.DstMemAddr = 0;
	// Transfer size
	GPDMACfg.TransferSize = DMA_SIZE;
	// Transfer width - unused
	GPDMACfg.TransferWidth = 0;
	// Transfer type
	GPDMACfg.TransferType = GPDMA_TRANSFERTYPE_M2P;
	// Source connection - unused
	GPDMACfg.SrcConn = 0;
	// Destination connection
	GPDMACfg.DstConn = GPDMA_CONN_DAC;
	// Linker List Item - unused
	GPDMACfg.DMALLI = (uint32_t)&DMA_LLI_Struct;
	// Setup channel with given parameter
	GPDMA_Setup(&GPDMACfg);
	return;
}
void confDac(void){
	uint32_t tmp;
	DAC_CONVERTER_CFG_Type DAC_ConverterConfigStruct;
	DAC_ConverterConfigStruct.CNT_ENA =SET;
	DAC_ConverterConfigStruct.DMA_ENA = SET;
	DAC_Init(LPC_DAC);
	/* set time out for DAC*/
	tmp = (PCLK_DAC_IN_MHZ*1000000)/(SINE_FREQ_IN_HZ*NUM_SINE_SAMPLE);
	DAC_SetDMATimeOut(LPC_DAC,tmp);
	DAC_ConfigDAConverterControl(LPC_DAC, &DAC_ConverterConfigStruct);
	return;
}


