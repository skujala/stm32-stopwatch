/* 	
	Copyright (c) 2012, Sami Kujala
   	All rights reserved.
	
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met: 

	1. Redistributions of source code must retain the above copyright notice, this
	   list of conditions and the following disclaimer. 
	2. Redistributions in binary form must reproduce the above copyright notice,
	   this list of conditions and the following disclaimer in the documentation
	   and/or other materials provided with the distribution. 

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
	ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MAIN_H_3P6METYV
#define MAIN_H_3P6METYV

#define GPIOCONF(mode, cnf)	((cnf << 2) | (mode))
#define GPIOPINCONFL(pin, conf) (conf << (pin * 4))
#define GPIOPINCONFH(pin, conf) (conf << ((pin - 8) * 4))

/* LIGHT PORT CONFIGURATION

Light port 1 is connected to PB1, which is alternate function to TIM3_CH4
Light port 2 is connected to PB0, which is alternate function to TIM3_CH3

*/
#define LIGHTPORT1_PIN 0
#define LIGHTPORT2_PIN 1
#define LIGHTPORT_GPIO GPIOB
#define LIGHTPORT_GPIO_ENABLE RCC_APB2ENR_IOPBEN

#define GREEN_LED_PIN 9
#define BLUE_LED_PIN 8
#define LED_GPIO GPIOC
#define LED_GPIO_ENABLE RCC_APB2ENR_IOPCEN

#define TIMER3_GPIO_ENABLE RCC_APB1ENR_TIM3EN
#define USART1_GPIO_ENABLE RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN

#define USART_RX_GPIO	GPIOA
#define USART_RX_PIN	 10
#define USART_TX_GPIO	GPIOA
#define USART_TX_PIN	 9

#define GPIO_CNF_OUTPUT_PUSHPULL	0x0
#define GPIO_CNF_OUTPUT_OPENDRAIN	0x1
#define GPIO_CNF_AFIO_PUSHPULL		0x2
#define GPIO_CNF_AFIO_OPENDRAIN		0x3

#define GPIO_CNF_INPUT_ANALOG		0x0
#define GPIO_CNF_INPUT_FLOATING		0x1
#define GPIO_CNF_INPUT_PULLUPDOWN	0x2

#define GPIO_MODE_INPUT				0x0
#define GPIO_MODE_OUTPUT10MHz		0x1
#define GPIO_MODE_OUTPUT2MHz		0x2
#define GPIO_MODE_OUTPUT50MHz		0x3
 
#define TIMER_RESOLUTION			2000
#define TIMER_STEP					(1000 / TIMER_RESOLUTION) /* In units of ms */ 
#define TRIGGER_DEBOUNCE			10

#define STDOUT_USART				1


enum counter_states {
    COUNTER_READY = 0x0,
    COUNTER_STARTED = 0x1,
    COUNTER_STOPPED = 0x2
};

struct stopwatch {
    uint16_t time_start;
    uint16_t time_elapsed;
    enum counter_states counter;
};


#endif                          /* end of include guard: MAIN_H_3P6METYV */
