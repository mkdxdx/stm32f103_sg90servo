#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "misc.h"
#include "delay.h"
#include <stdlib.h>

#define CMDBUF_LENGTH	50

#define TIMx			TIM4
#define	TIMx_RCC		RCC_APB1Periph_TIM4
#define TIMx_Ch1_Pin	GPIO_Pin_6
#define TIMx_Ch2_Pin	GPIO_Pin_7
#define TIMx_GPIO_Port	GPIOB
#define TIMx_GPIO_PortR	RCC_APB2Periph_GPIOB

#define USARTx			USART1
#define PIN_RX			GPIO_Pin_10
#define PIN_TX			GPIO_Pin_9
#define UGPIO			GPIOA
#define GPIO_RCC		RCC_APB2Periph_GPIOA
#define USART_RCC		RCC_APB2Periph_USART1
#define AFIO_RCC		RCC_APB2Periph_AFIO
#define USARTx_IRQ		USART1_IRQn

#define SYSCLK 72000000
#define PRESCALER 72

void CtrlLoop(void);
void RCCInit(void);
void GPIOInit(void);
void TIMInit(void);
void USARTInit(uint32_t baud);
void ITInit(void);
void TokenCheck(void);
void ClrBuffer(void);
void SomeDelay(void);
void USend(char data);
void USendStr(char *data);

GPIO_InitTypeDef port;
TIM_TimeBaseInitTypeDef timer;
TIM_OCInitTypeDef timerPWM;
TIM_OCInitTypeDef timerPWM2;
USART_InitTypeDef uis;
NVIC_InitTypeDef nitd;

// commands like X:xxxx,Y:yyyy; should go here
volatile char cmdbuf[CMDBUF_LENGTH];
volatile char rxlchar;
volatile uint16_t buf_index;
char strval[5];

volatile int8_t xvel,yvel;
volatile uint16_t xpos,ypos;



void USART1_IRQHandler(void) {
	if (USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET) {
		USART_ClearITPendingBit(USARTx,USART_IT_RXNE);

		rxlchar = USART_ReceiveData(USARTx);
		cmdbuf[buf_index++] = rxlchar;
		cmdbuf[buf_index] = '\0';

		if (buf_index>=CMDBUF_LENGTH-1) {
			buf_index = 0;
			ClrBuffer();
		}
	}

	if (USART_GetITStatus(USART1,USART_IT_TC)!=RESET) {
		USART_ClearITPendingBit(USART1,USART_IT_TC);
	}
}


int main(void)
{
	SystemInit();
	Delay_Init(72);
	RCCInit();
	GPIOInit();
	TIMInit();
	ClrBuffer();
	USARTInit(9600);
	ITInit();

	xpos = 1200;
	ypos = 1200;

	xvel = 0;
	yvel = 0;

	USendStr("Init\r\n");

	while (1) {
		TokenCheck();
		CtrlLoop();
		delay_ms(10);
	}
}

void ClrBuffer(void) {
	for (buf_index = 0; buf_index<CMDBUF_LENGTH; buf_index++) {
		cmdbuf[buf_index] = '\0';
	}
	buf_index = 0;
}

void TokenCheck(void) {
	uint8_t result = 0;
	if ((rxlchar == ';') && (buf_index > 0)) {
		cmdbuf[buf_index-1] = '\0';
		char * p = (char *)strtok(cmdbuf,",");
		while (p != NULL) {
			if ((*p == 'X') || (*p == 'x')) {
				p++;
				xvel = (int8_t)atoi(p);
				result++;
			}

			if ((*p == 'Y') || (*p == 'y')) {
				p++;
				yvel = (int8_t)atoi(p);
				result++;
			}
			p = (char *)strtok(NULL,",");
		}
		ClrBuffer();
		if (result >= 2) {
			USendStr("OK\r\n");
		} else {
			USendStr("#LT1\r\n");
		}
	}
}



void CtrlLoop(void) {
	if ((xpos + xvel > 1000) && (xpos + xvel < 2000)) {
		xpos += xvel;
	}

	if ((ypos + yvel > 1000) && (ypos + yvel < 2000)) {
		ypos += yvel;
	}

	TIM_SetCompare1(TIMx,xpos);
	TIM_SetCompare2(TIMx,ypos);

}

void RCCInit(void) {
	RCC_APB2PeriphClockCmd((TIMx_GPIO_PortR | USART_RCC | GPIO_RCC | AFIO_RCC), ENABLE);
	RCC_APB1PeriphClockCmd(TIMx_RCC, ENABLE);
}

void TIMInit(void) {
	TIM_TimeBaseStructInit(&timer);
	timer.TIM_Prescaler = PRESCALER;
	timer.TIM_Period = SYSCLK / PRESCALER / 50;
	timer.TIM_ClockDivision = 0;
	timer.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIMx, &timer);

	TIM_OCStructInit(&timerPWM);
	timerPWM.TIM_Pulse = 1000;
	timerPWM.TIM_OCMode = TIM_OCMode_PWM1;
	timerPWM.TIM_OutputState = TIM_OutputState_Enable;
	timerPWM.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIMx, &timerPWM);
	TIM_OC2Init(TIMx, &timerPWM);

	TIM_Cmd(TIM4, ENABLE);
}


void GPIOInit(void) {
	GPIO_StructInit(&port);
	port.GPIO_Mode = GPIO_Mode_AF_PP;
	port.GPIO_Pin = (TIMx_Ch1_Pin | TIMx_Ch2_Pin);
	port.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(TIMx_GPIO_Port, &port);

	GPIO_StructInit(&port);

	port.GPIO_Pin = PIN_RX;
	port.GPIO_Speed = GPIO_Speed_50MHz;
	port.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(UGPIO,&port);

	port.GPIO_Pin = PIN_TX;
	port.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(UGPIO, &port);
}

void USARTInit(uint32_t baud) {
	USART_StructInit(&uis);
	uis.USART_BaudRate = baud;
	uis.USART_Mode = (USART_Mode_Rx | USART_Mode_Tx);
	uis.USART_WordLength = USART_WordLength_8b;
	uis.USART_StopBits = USART_StopBits_1;
	uis.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USARTx,&uis);

	USART_Cmd(USARTx,ENABLE);
}

void ITInit(void) {
	NVIC_Init(&nitd);
	nitd.NVIC_IRQChannel = USARTx_IRQ;
	nitd.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nitd);
	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
	__enable_irq();
}


void USend(char data) {
	USART_SendData(USARTx,data);
	while(!USART_GetFlagStatus(USARTx,USART_FLAG_TC));
}

void USendStr(char *data) {
	while (*data!='\0') {
		USend(*data++);
	}
}
