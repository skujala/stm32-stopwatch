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

#include <stm32f10x.h>

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#define STDIN_USART 				1
#define STDOUT_USART 				1
#define STDERR_USART 				1

#include "main.h"
#include "newlib_stubs.h"

volatile uint32_t ticks = 0L;

volatile struct stopwatch stopwatch;

static void stopwatch_reset(volatile struct stopwatch *sw);
static void rcc_init(void);
static void usart_init(void);
static void gpio_init(void);
static void tim_init(void);
static void delay_ms(uint32_t ival);
static void nvic_init(void);


/**
  * @brief  Main program.
  * @param  None
  * @retval : None
  */
int main(void)
{
    int i;    
    stopwatch_reset(&stopwatch);
	
    /*
     * Enable peripherals
     */
    rcc_init();
    gpio_init();
    nvic_init();
    usart_init();
    tim_init();

    delay_ms(100);
    
    _write(1,"Oheislaitteet alustettu\r\n", 25);
    LED_GPIO->BSRR = (1 << BLUE_LED_PIN);

    
    while (1) {
        if (stopwatch.counter == COUNTER_STOPPED){
            for (i = 0; i < 10; i++) {
                LED_GPIO->ODR ^= (1 << BLUE_LED_PIN);
                delay_ms(50);                
            }
            iprintf("Aika: %d ms\r\n", TIMER_COUNTS_IN_MS(stopwatch.counts_elapsed));
                
            stopwatch_reset(&stopwatch);
        }
    }

    return 0;
}


/* Function definitions */

static void stopwatch_reset(volatile struct stopwatch *sw)
{
	__disable_irq ();	
    sw->counts_start = 0;
    sw->counts_elapsed = 0xFFFF;
    sw->counter = COUNTER_READY;
	__enable_irq ();
}

static void rcc_init(void)
{
    /* GPIO Initialization - enable clocks etc */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN 	/* USART1 GPIO ENABLE */
		| RCC_APB2ENR_IOPCEN 									/* LED GPIO ENABLE */
		| RCC_APB2ENR_IOPBEN 									/* LIGHTPORT GPIO ENABLE */
		| RCC_APB2ENR_AFIOEN; 									/* ALTERNATE FUNCTION GPIO ENABLE */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; /* ENABLE TIM3 TIMER GPIO */
}

static void gpio_init(void)
{
	CONF_GPIOH(LED_GPIO, BLUE_LED_PIN, GPIO_MODE_OUTPUT50MHz, GPIO_CNF_OUTPUT_PUSHPULL);
	CONF_GPIOH(LED_GPIO, GREEN_LED_PIN, GPIO_MODE_OUTPUT50MHz, GPIO_CNF_OUTPUT_PUSHPULL);
		
	CONF_GPIOH(USART_TX_GPIO, USART_TX_PIN, GPIO_MODE_OUTPUT2MHz, GPIO_CNF_AFIO_PUSHPULL);	

	CONF_GPIOL(LIGHTPORT_GPIO, LIGHTPORT1_PIN, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOATING);
	CONF_GPIOL(LIGHTPORT_GPIO, LIGHTPORT2_PIN, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOATING);
}



static void usart_init(void)
{	    
	/* We want baudrate 115200 */	
	/* want usartdiv 24 000 000L / (115 200 * 16) = 13.0208333...   */
	/* which is 208 or 0xd0 when encoded in 12.4 fixed-point format */
	/* ERROR < 0.16 %                                               */
	USART1->BRR = 0xd0;
			
	/* Configure USART for 8 bits, 1 stop bit, no parity, no hardware flow control */
	
    /* Clear M bit => 8 bit word length, PCE = 0 => no parity*/
    USART1->CR1 &= ~(USART_CR1_M | USART_CR1_PCE);
    /* Clear stop bits => 1 stop bit */
    USART1->CR2 &= ~(USART_CR2_STOP_0 | USART_CR2_STOP_1);
    
    /* Enable USART1 transmitter: UE - usart enable & TE - transmitter enable. Do not enable the receiver, we are not interested in receiving any bytes from anybody*/
    USART1->CR1 |= USART_CR1_TE | USART_CR1_UE;
}




static void tim_init(void)
{
    uint16_t PrescalerValue = 0;

    /* Compute the prescaler value */
    PrescalerValue = (uint16_t) (SystemCoreClock / TIMER_RESOLUTION) - 1;
    /* Set Prescaler to specified value */
    TIM3->PSC = PrescalerValue;
    TIM3->ARR = 0xFFFF;

    /* ENABLE CHANNELS 3 & 4 */

    /* Configure Capture Compare Mode Register 2 
     * Input capture 3 mapped to timer input 3
     * Input capture 4 mapped to timer input 4
     * NO input filter
     * NO input prescaler */
    TIM3->CCMR2 = TIM_CCMR2_CC3S_1 | TIM_CCMR2_CC4S_1;

    /* Set polarity */
    TIM3->CCER |= TIM_CCER_CC3P | TIM_CCER_CC4P;

    /* Configure TIM3 to send interrupts when it detects transition in either channels, 3 or 4 */
    TIM3->DIER = TIM_DIER_CC3IE | TIM_DIER_CC4IE;

    /* Enable capture */
    TIM3->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;

    /* Enable timer */
    TIM3->CR1 = TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM3_IRQn);
}


static void nvic_init(void)
{
    /*  Configure SysTick to tick every ms */
    if (SysTick_Config((uint16_t) (SystemCoreClock / 1000) - 1)) {
        while (1);              /* Capture error */
    }
}

void TIM3_IRQHandler(void)
{
    /* In the interrupt handler
       - check who interrupted us: currently interesting interrupts are those arising due to capture / compare event
       - check the timer count TIM3->CNT and save it in the state machine
       - transition the state machine appropriately

       - Timing can start from whichever capture/compare (CC) channel BUT MUST end with different one
       - Remember to debounce the inputs!
     */

    uint16_t now = TIM3->CNT;
    uint16_t elapsed = 0;
	
	if (TIM3->SR & TIM_SR_CC4IF) {
		/* Clear the interrupt flag */
        TIM3->SR &= ~(TIM_SR_CC4IF);      
		
        if (stopwatch.counter == COUNTER_READY){
			/* Turn LED on to indicate stopwatch starting */
	        LED_GPIO->BSRR = (1 << GREEN_LED_PIN); 
            
            stopwatch.counter = COUNTER_STARTED;
            stopwatch.counts_start = now;
            stopwatch.counts_elapsed = 0xFFFF;
        }
	} else if (TIM3->SR & TIM_SR_CC3IF) {
        /* Clear the interrupt flags */
		TIM3->SR &= ~(TIM_SR_CC3IF);
		
		/* Compute elapsed time correctly, taking into account the possibility of TIM3->CNT overflow between start and now */
		if (now > stopwatch.counts_start) {
	        elapsed = now - stopwatch.counts_start;			
		} else {
	        elapsed = (0xFFFF - stopwatch.counts_start) + now;			
		} 
		
        if (elapsed > TRIGGER_DEBOUNCE && stopwatch.counter == COUNTER_STARTED) {
			/* Turn LED off */
	        LED_GPIO->BRR = (1 << GREEN_LED_PIN);
            
            stopwatch.counter = COUNTER_STOPPED;
            stopwatch.counts_elapsed = elapsed;
        }
    }  
}

void SysTick_Handler(void)
{
    ticks++;
}

void delay_ms(uint32_t ival)
{
    uint32_t now = ticks;

    while (ticks - now < ival) {
        asm("nop");
    }
}
