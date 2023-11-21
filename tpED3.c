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
#include <stdlib.h>

// Arreglo de valores de tension para diferentes mediciones de ADC
float voltValues[] = {
    0.0000, 0.0129, 0.0258, 0.0387, 0.0516, 0.0645, 0.0774, 0.0903,
    0.1032, 0.1161, 0.1290, 0.1419, 0.1548, 0.1677, 0.1806, 0.1935,
    0.2064, 0.2193, 0.2322, 0.2451, 0.2580, 0.2709, 0.2838, 0.2967,
    0.3096, 0.3225, 0.3354, 0.3483, 0.3612, 0.3741, 0.3870, 0.3999,
    0.4128, 0.4257, 0.4386, 0.4515, 0.4644, 0.4773, 0.4902, 0.5031,
    0.5160, 0.5289, 0.5418, 0.5547, 0.5676, 0.5805, 0.5934, 0.6063,
    0.6192, 0.6321, 0.6450, 0.6579, 0.6708, 0.6837, 0.6966, 0.7095,
    0.7224, 0.7353, 0.7482, 0.7611, 0.7740, 0.7869, 0.7998, 0.8127,
    0.8256, 0.8385, 0.8514, 0.8643, 0.8772, 0.8901, 0.9030, 0.9159,
    0.9288, 0.9417, 0.9546, 0.9675, 0.9804, 0.9933, 1.0062, 1.0191,
    1.0320, 1.0449, 1.0578, 1.0707, 1.0836, 1.0965, 1.1094, 1.1223,
    1.1352, 1.1481, 1.1610, 1.1739, 1.1868, 1.1997, 1.2126, 1.2255,
    1.2384, 1.2513, 1.2642, 1.2771, 1.2900, 1.3029, 1.3158, 1.3287,
    1.3416, 1.3545, 1.3674, 1.3803, 1.3932, 1.4061, 1.4190, 1.4319,
    1.4448, 1.4577, 1.4706, 1.4835, 1.4964, 1.5093, 1.5222, 1.5351,
    1.5480, 1.5609, 1.5738, 1.5867, 1.5996, 1.6125, 1.6254, 1.6383,
    1.6512, 1.6641, 1.6770, 1.6899, 1.7028, 1.7157, 1.7286, 1.7415,
    1.7544, 1.7673, 1.7802, 1.7931, 1.8060, 1.8189, 1.8318, 1.8447,
    1.8576, 1.8705, 1.8834, 1.8963, 1.9092, 1.9221, 1.9350, 1.9479,
    1.9608, 1.9737, 1.9866, 1.9995, 2.0124, 2.0253, 2.0382, 2.0511,
    2.0640, 2.0769, 2.0898, 2.1027, 2.1156, 2.1285, 2.1414, 2.1543,
    2.1672, 2.1801, 2.1930, 2.2059, 2.2188, 2.2317, 2.2446, 2.2575,
    2.2704, 2.2833, 2.2962, 2.3091, 2.3220, 2.3349, 2.3478, 2.3607,
    2.3736, 2.3865, 2.3994, 2.4123, 2.4252, 2.4381, 2.4510, 2.4639,
    2.4768, 2.4897, 2.5026, 2.5155, 2.5284, 2.5413, 2.5542, 2.5671,
    2.5800, 2.5929, 2.6058, 2.6187, 2.6316, 2.6445, 2.6574, 2.6703,
    2.6832, 2.6961, 2.7090, 2.7219, 2.7348, 2.7477, 2.7606, 2.7735,
    2.7864, 2.7993, 2.8122, 2.8251, 2.8380, 2.8509, 2.8638, 2.8767,
    2.8896, 2.9025, 2.9154, 2.9283, 2.9412, 2.9541, 2.9670, 2.9799,
    2.9928, 3.0057, 3.0186, 3.0315, 3.0444, 3.0573, 3.0702, 3.0831,
    3.0960, 3.1089, 3.1218, 3.1347, 3.1476, 3.1605, 3.1734, 3.1863,
    3.1992, 3.2121, 3.2250, 3.2379, 3.2508, 3.2637, 3.2766, 3.2895
};

// Valores para mapear ADC a PWM a partir de multiplos de 10 del voltaje
uint32_t adcMapValues[34] = {
	0, 7, 12, 24, 37, 46, 55, 60, 65, 70, 76, 82, 90, 95, 102, 108, 115,
	122, 129, 135, 140, 150, 155, 168, 178, 182, 189, 199, 207, 219, 235,
	248, 250, 255
};

// Valores extremos de PWM para el servomotor utilizado, MG946R de Tower Pro (DigiHi Torque)
uint32_t pwmMin = 500;
uint32_t pwmMax = 2400;

// Valores para medir extremos del recorrido en el potenciometro
uint16_t adcMin;
uint16_t adcMax;

// Buffer para transmitir por UART
char buffer[50];

// Valor de PWM requerido
uint32_t pwmWanted;

// String para recibir por UART
uint8_t info[4] = "";

// Voltaje medido
uint32_t volt;

// Variables globales para monitorizar
uint32_t mapValue;
uint32_t pwmValue;

// Extremos de tension medidos con ADC
float voltMin;
float voltMax;

// Prototipo de funciones
void configPin(void);
void configADC(void);
void configPWM(void);
void configUART(void);
void floatToString(char* buffer, float value);
void int2FloatString(char* buffer, uint32_t value);
void sendString(float a, float b);
void configEINT(void);
void autoMap(void);
void delay(uint32_t);
void writePWMPosition(uint32_t);
uint32_t pwmMap(uint32_t);

int main(void)
{
    configPin(); // Se configuran los pines de la placa
    configEINT(); // Se configura la interrupcion externa
    configADC(); // Se configura el ADC
    configPWM(); // Se configura PWM
    configUART(); // Se configura la comunicacion por UART
    // Pequeño mensaje de bienvenida
    char newMessage[50] = "Trabajo Practico";
    // Envio del mensaje de bienvenida
    UART_Send(LPC_UART3, newMessage, sizeof(newMessage), NONE_BLOCKING);
    delay(10);
    autoMap(); // Mapeo automatico del potenciometro
    while (1); // Bucle para esperar interrupciones y trabajar fuera del main()
    return 0;
}

/*
    * Funcion que configura los pines de la placa. Se configuran los pines P2.0 para PWM, P0.25 para Buzzer,
    * P0.23 para ADC, P2.10 para EINT0 y P0.1 y P0.0 para UART3
    * @param: void
    * @return: void
*/
void configPin(void)
{
    // Configuracion pin P2.0 para PWM
    PINSEL_CFG_Type pwmPin;
    pwmPin.Portnum = 2;
    pwmPin.Pinmode = PINSEL_PINMODE_TRISTATE;
    pwmPin.Funcnum = 1;
    pwmPin.OpenDrain = PINSEL_PINMODE_NORMAL;
    pwmPin.Pinnum = 0;
    PINSEL_ConfigPin(&pwmPin);

    // Configuracion pin P0.25 para Buzzer como salida
    PINSEL_CFG_Type buzzerPin;
    buzzerPin.Portnum = 0;
    buzzerPin.Pinmode = PINSEL_PINMODE_TRISTATE;
    buzzerPin.Funcnum = 0;
    buzzerPin.OpenDrain = PINSEL_PINMODE_NORMAL;
    buzzerPin.Pinnum = 25;
    LPC_GPIO0 -> FIODIR |= (1<<25);
    LPC_GPIO0 -> FIOCLR |= (1<<25);
    PINSEL_ConfigPin(&buzzerPin);

    // Configuracion pin P0.23 para ADC
    PINSEL_CFG_Type analogPin;
    analogPin.Portnum = 0;
    analogPin.Pinnum = 23;
    analogPin.Funcnum = 1;
    analogPin.Pinmode = PINSEL_PINMODE_TRISTATE;
    analogPin.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&analogPin);

    // Configuracion pin P2.10 para EINT0
    PINSEL_CFG_Type eintPin;
    eintPin.Portnum = 2;
    eintPin.Pinnum = 10;
    eintPin.Funcnum = 1;
    eintPin.Pinmode = PINSEL_PINMODE_PULLUP;
    eintPin.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&eintPin);

    // Configuracion pin P0.1 y P0.0 para UART3. P0.1 TXD3 y P0.0 RXD3
    PINSEL_CFG_Type uartPin;
    uartPin.Funcnum = 2;
	uartPin.OpenDrain = 0;
	uartPin.Pinmode = 0;
	uartPin.Pinnum = 1;
	uartPin.Portnum = 0;
	PINSEL_ConfigPin(&uartPin);
	uartPin.Pinnum = 0;
	PINSEL_ConfigPin(&uartPin);
    return;
}

/*
    * Funcion que configura UART3. Se configura para transmitir, e interrumpir en el caso de recibir 
    * @param: void
    * @return: void
*/
void configUART(void) {
    // UARTConfigStruct -> Estructura de configuracion de UART 
    UART_CFG_Type      UARTConfigStruct;
    UARTConfigStruct.Stopbits = 1; // Se usa un stopbit 1 
    UARTConfigStruct.Databits = 8; // 8 bits de datos

    // UARTFIFOConfigStruct -> Estructura de configuracion de FIFO de UART
	UART_FIFO_CFG_Type UARTFIFOConfigStruct;

    // Se inicializa la estructura de configuracion de UART
	UART_ConfigStructInit(&UARTConfigStruct);
	UART_Init(LPC_UART3, &UARTConfigStruct);
	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);
	UART_FIFOConfig(LPC_UART3, &UARTFIFOConfigStruct);
	UART_IntConfig(LPC_UART3, UART_INTCFG_RBR, ENABLE);
	UART_IntConfig(LPC_UART3, UART_INTCFG_RLS, ENABLE);

    // Se habilita la transmision por UART
	UART_TxCmd(LPC_UART3, ENABLE);
    // Se habilita la interrupcion por UART
	NVIC_EnableIRQ(UART3_IRQn); 
	return;
}

/*
    * Configura el ADC con una frecuencia de muestreo de 2000 Hz y habilita el canal 0 del ADC
    * @param: void
    * @return: void
    * @note: Se configura el ADC para que trabaje en modo burst
*/
void configADC(void)
{   
    // Configuracion inicial de ADC
    ADC_Init(LPC_ADC, 2000);
    
    // Habilitacion de CH0 del ADC
    ADC_ChannelCmd(LPC_ADC, 0, ENABLE);
    
    // No habilito interrupcion por ADC0, ya que tomamos el valor del ADC cuando se requiera
    ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, DISABLE);

    // ADC Burst mode setting
    ADC_BurstCmd(LPC_ADC, 1);

    // Limpiamos bandera de interrupcion por ADC
    LPC_ADC->ADGDR &= LPC_ADC->ADGDR;
    return;
}

/*
    * Funcion que configura PWM1. Se configura para que tenga un periodo de 20ms, y valor inicial de 0.5 ms (0°)
    Se configura para que el TC incremente cada 100 ticks de PCLK_PWM1
    * @param: void
    * @return: void
*/
void configPWM(void)
{
    // Selecccion de PCLK para PWM1. PCLK_PWM1 = CCLK/1 = 100MHz
    CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_PWM1, CLKPWR_PCLKSEL_CCLK_DIV_1);

    // Configuracion de PWM
    // PCR = 0x0 -> PWM1 single edge
    LPC_PWM1->PCR = 0x0;
    
    // Prescaler = 99 -> Por lo tanto, el TC incrementa cada 100 ticks de PCLK_PWM1 
    LPC_PWM1->PR = 99;
    // MR0 = 20000 -> PWM1 period = 20000 ticks de PCLK_PWM1
    LPC_PWM1->MR0 = 20000;
    // MR1 = 500 -> PWM1 pulse width = 500 ticks de PCLK_PWM1. Inicialmente en 0° el servomotor
    LPC_PWM1->MR1 = 500;

    // Reset on MR0: TC will be reset if MR0 matches it
    LPC_PWM1->MCR = (1 << 1);
    // Actualizacion de MR1 y MR0
    LPC_PWM1->LER |= (1 << 0) | (1 << 1);
    // Habilitacion de PWM1
    LPC_PWM1->PCR |= (1 << 9);

    // Habilitacion de contador y PWM
    LPC_PWM1->TCR = 3;
    LPC_PWM1->TCR &= ~(1 << 1);
    LPC_PWM1->TCR |= (1 << 3);
    return;
}

void UART3_IRQHandler(void) {
    // Pequeño delay para esperar a recibir todos los datos
	delay(10);
	uint32_t intsrc, tmp, tmp1;
	intsrc = UART_GetIntId(LPC_UART3);
	tmp = intsrc & UART_IIR_INTID_MASK;
	if (tmp == UART_IIR_INTID_RLS){
		tmp1 = UART_GetLineStatus(LPC_UART3);
		tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE \
				| UART_LSR_BI | UART_LSR_RXFE);
		if (tmp1) {
			while(1){};
		}
	}
	if ((tmp == UART_IIR_INTID_RDA) || (tmp == UART_IIR_INTID_CTI)){
        // Se obtiene el dato de entrada
		UART_Receive(LPC_UART3, info, sizeof(info), NONE_BLOCKING);
	}
    // Se parsea para obtener un valor de tension deseado
	char voltAux[2] = {info[0],info[2]};
	volt = (voltAux[0] - '0') * 10 + (voltAux[1] - '0');
    // Se mapea el valor con respecto a la magnitud a ver en el ADC
	mapValue = adcMapValues[volt];
	pwmValue = pwmMap(mapValue);
    // Se mueve el servo a la posicion deseada
	writePWMPosition(pwmValue);
	// Se emite una alarma sonora
    LPC_GPIO0 -> FIOSET |= (1<<25);
    // Mensaje de finalizacion
	char finalMessage[50] = "Listo!\n";
	UART_Send(LPC_UART3, finalMessage, sizeof(finalMessage), NONE_BLOCKING);
    // Se habilita la interrupcion externa para poder reiniciar la autocalibracion
	NVIC_EnableIRQ(EINT0_IRQn);
    // Se deshabilita la interrupcion por UART
    NVIC_DisableIRQ(UART3_IRQn);
	return;
}

/*
    * Funcion que configura el pin P2.10 para la interrupcion externa 0. Se configura para que sea
    * sensible a flanco de bajada
    * @param: void
    * @return: void
*/
void configEINT(void)
{
    // Se configura la interrupcion externa 0 para que sea sensible a flanco de bajada
    EXTI_SetMode(EXTI_EINT0, EXTI_MODE_EDGE_SENSITIVE);
    EXTI_SetPolarity(EXTI_EINT0, EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE);
    // Se limpia la bandera
    LPC_SC -> EXTINT |= 1;
    return;
}

/*
    * Funcion que se ejecuta cuando se presiona el pulsador. Se apaga la alarma sonora, se muestra
    * un mensaje por UART indicando que se esta reiniciando el prototipo, se realiza el procedimiento
    * de auto mapeo del ADC y se habilitan las interrupciones por UART nuevamente para que el usuario
    * pueda ingresar un nuevo valor de tension deseado.
    * @param: void
    * @return: void
*/
void EINT0_IRQHandler(void) {
	delay(100); // Antirebote
    // Se apaga la alarma sonora anteriormente encendida
	LPC_GPIO0 -> FIOCLR |= (1<<25);
    // Mensaje para indicar el reinicio del prototipo
	char newMessage[100] = "Reiniciando...\n";
	UART_Send(LPC_UART3, newMessage, sizeof(newMessage), NONE_BLOCKING);
    // Se mapea y calibra nuevamente de forma automatica
	autoMap();
    // Se habilitan las interrupciones por UART
	NVIC_EnableIRQ(UART3_IRQn);
    // Se limpia la bandera de interrupcion
	LPC_SC -> EXTINT |= 1;
	return;
}

/*
    * Funcion que realiza el procedimiento de auto mapeo del ADC. Se obtienen los valores de tension
    * minimo y maximo que se pueden obtener del ADC. Se le muestra al usuario los valores limite. 
    * El servo primero va a cero grados, se obtiene el valor del ADC y se guarda como valor minimo.
    * Luego, el servo va a 180 grados, se obtiene el valor del ADC y se guarda como valor maximo.
    * Mediante UART se le muestra al usuario los valores limite entre los cuales puede elegir.
    * @param: void
    * @return: void
*/
void autoMap(void)
{
    // Se mapea el potenciometro a nivel valores de PWM posibles de escribir en el servomotor
    // Se lleva el servo al extremo de menos tension
    LPC_PWM1->MR1 = pwmMin;
    // Se actualiza el valor del PWM
    LPC_PWM1->LER |= (1 << 1);
    delay(100);
    // Se obtiene una medicion del ADC
    if (ADC_ChannelGetStatus(LPC_ADC, 0, 1))
    {
        adcMin = ADC_ChannelGetData(LPC_ADC, 0) / 16;
    }
    delay(500);
    // Se lleva el servo al extremo de mayor tension
    LPC_PWM1->MR1 = pwmMax;
    // Se actualiza el valor de PWM
    LPC_PWM1->LER |= (1 << 1);
    delay(100);
    // Se obtiene una medicion del ADC
    if (ADC_ChannelGetStatus(LPC_ADC, 0, 1))
    {
        adcMax = ADC_ChannelGetData(LPC_ADC, 0) /16;
    }
    // Valores finales obtenidos del procedimiento de auto mapeo
    voltMin = voltValues[adcMin];
    voltMax = voltValues[adcMax];
    // Se le muestran al usuario los valores limite
    sendString(voltMin, voltMax);
    delay(100);
    return;
}

/*
    * Funcion que mapea el valor del ADC a un valor de PWM. Es una funcion lineal
    * @param adcValue: valor del ADC que se desea mapear
    * @return: valor de PWM que se debe escribir en el registro MR1
*/
uint32_t pwmMap(uint32_t adcValue)
{
	static uint32_t m,b,pwm2Write;
    // m -> pendiente de la recta
    m = (pwmMax-pwmMin) / (255 - 0);
    // b -> ordenada al origen de la recta
    b = pwmMin - m * 0;

    // pwm2Write -> valor de PWM que se debe escribir en el registro MR1
    pwm2Write = (uint32_t)(m * adcValue + b);
    return pwm2Write;
}

/*
    * Funcion que escribe el valor de PWM en el registro MR1 y realiza la actualizacion
    * @param pwmValue: valor de PWM que se desea escribir en el registro MR1
    * @return: void
*/
void writePWMPosition(uint32_t pwmValue)
{
	LPC_PWM1->MR1 = pwmValue;
	LPC_PWM1->LER |= (1 << 1);
    return;
}

/*
    * Funcion que genera un delay de ms milisegundos
    * @param ms: cantidad de milisegundos que se desea esperar
    * @return: void
*/
void delay(uint32_t ms)
{
    uint32_t ticks = ms * 100000;
    for (uint32_t i = 0; i < ticks; i++)
        ;
    return;
}

/*
    * Funcion que envia un string por UART3. El string contiene el valor minimo y maximo de tension 
    * que se obtienen del mapeo del ADC. El mensaje es "Elegi un valor entre " + "valor minimo" + " y "
    * + "valor maximo
    * @param a: valor minimo de tension
    * @param b: valor maximo de tension
    * @return: void
*/
void sendString(float a, float b) {
    // En esta funcion se paersean dos  flotantes, que representan los extremos de tension, y se los envia por UART
    
    char bufferA[20];
    char bufferB[20];
    floatToString(bufferA, a);
    floatToString(bufferB, b);
    char message[50] = "Elegi un valor entre ";
    strcat(message, bufferA);
    strcat(message, " y ");
    strcat(message, bufferB);
    strcat(message, "\n");
    UART_Send(LPC_UART3, message, sizeof(message), BLOCKING);
}

/*
    * Funcion que convierte un float a un string
    * @param buffer: string donde se va a guardar el float convertido
    * @param value: float que se desea convertir
    * @return: void
*/
void floatToString(char* buffer, float value) {
    int wholePart = (int)value;
    int decimalPart = (value - wholePart) * 100;
    itoa(wholePart, buffer, 10);
    int len = strlen(buffer);
    buffer[len] = '.';
    itoa(decimalPart, buffer + len + 1, 10);
}
