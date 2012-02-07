/**
  ******************************************************************************
  * @file     
  * @author  
  * @version 
  * @date    
  * @brief   Main program body
  ******************************************************************************
  * @copy
  *
  *
  * <h2><center>&copy; COPYRIGHT 2012 Sami Kujala </center></h2>
  */


#include <stm32f10x.h>

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#define GPIO_CNF_INPUT_ANALOG		0
#define GPIO_CNF_INPUT_FLOATING		1
#define GPIO_CNF_INPUT_PULLUPDOWN	2

#define GPIO_CNF_OUTPUT_PUSHPULL	0
#define GPIO_CNF_OUTPUT_OPENDRAIN	1
#define GPIO_CNF_AFIO_PUSHPULL		2
#define GPIO_CNF_AFIO_OPENDRAIN		3

#define GPIO_MODE_INPUT				0
#define GPIO_MODE_OUTPUT10MHz		1
#define GPIO_MODE_OUTPUT2MHz		2
#define GPIO_MODE_OUTPUT50MHz		3

#define GPIOCONF(mode, cnf)	((cnf << 2) | (mode))
#define GPIOPINCONFL(pin, conf) (conf << (pin * 4))
#define GPIOPINCONFH(pin, conf) (conf << ((pin - 8) * 4))

#define CONFMASKL(pin) ((uint32_t)~(15 << (pin * 4)))
#define CONFMASKH(pin) ((uint32_t)~(15 << ((pin - 8) * 4)))

/* LIGHT PORT CONFIGURATION

Light port 1 is connected to PB0, which is alternate function to TIM3_CH3
Light port 2 is connected to PB1, which is alternate function to TIM3_CH4

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
	  
volatile uint32_t ticks = 0L;


/* Function definitions */
static void rcc_init(void)
{
    /* GPIO Initialization - enable clocks etc */
	RCC->APB2ENR |= LIGHTPORT_GPIO_ENABLE | USART1_GPIO_ENABLE | LED_GPIO_ENABLE | RCC_APB2ENR_AFIOEN;
	RCC->APB1ENR |= TIMER3_GPIO_ENABLE;
}


static void usart_init(void)
{
	/* Enable USART1 transmitter: UE - usart enable & TE - transmitter enable */
	USART1->CR1 = USART_CR1_UE | USART_CR1_TE;
	
	/* Set baudrate */
	USART1->BRR = (SystemCoreClock / 115200);
	
}

static void gpio_init(void)
{
	LED_GPIO->CRH = (LED_GPIO->CRH & CONFMASKH(BLUE_LED_PIN)) | GPIOPINCONFH(BLUE_LED_PIN,   GPIOCONF(GPIO_MODE_OUTPUT50MHz, GPIO_CNF_OUTPUT_PUSHPULL));
    LED_GPIO->CRH = (LED_GPIO->CRH & CONFMASKH(GREEN_LED_PIN)) | GPIOPINCONFH(GREEN_LED_PIN, GPIOCONF(GPIO_MODE_OUTPUT50MHz, GPIO_CNF_OUTPUT_PUSHPULL));
	
	// USART_TX_GPIO->CRH = (USART_TX_GPIO->CRH & CONFMASKH(USART_TX_PIN)) | GPIOPINCONFH(USART_TX_PIN,   GPIOCONF(GPIO_MODE_OUTPUT2MHz, GPIO_CNF_AFIO_PUSHPULL));	
	
	LIGHTPORT_GPIO->CRL = (LIGHTPORT_GPIO->CRL & CONFMASKL(LIGHTPORT1_PIN)) | GPIOPINCONFL(LIGHTPORT1_PIN, GPIOCONF(GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULLUPDOWN));
	LIGHTPORT_GPIO->CRL = (LIGHTPORT_GPIO->CRL & CONFMASKL(LIGHTPORT2_PIN)) | GPIOPINCONFL(LIGHTPORT2_PIN, GPIOCONF(GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULLUPDOWN));

    /*  Remap TIM3_CH3 to GPIOC_Pin8 AKA the green LED 
	AFIO->MAPR = AFIO_MAPR_TIM3_REMAP;
	*/
	
	
}


static void tim_init(void)
{
    uint16_t CCR4_Val = 1000;
    uint16_t PrescalerValue = 0;

	/* 
    For example, you can measure the period (in TIMx_CCR1 register)
    and the duty cycle (in TIMx_CCR2 register) of the PWM applied on
    TI1 using the following procedure (depending on CK_INT frequency
    and prescaler value):
    ● Select the active input for TIMx_CCR1: write the CC1S bits
      to 01 in the TIMx_CCMR1 register (TI1 selected).
    ● Select the active polarity for TI1FP1 (used both for capture
      in TIMx_CCR1 and counter clear): write the CC1P bit to ‘0’
      (active on rising edge).
    ● Select the active input for TIMx_CCR2: write the CC2S bits
      to 10 in the TIMx_CCMR1 register (TI1 selected).
    ● Select the active polarity for TI1FP2 (used for capture in
      TIMx_CCR2): write the CC2P bit to ‘1’ (active on falling
      edge).
    ● Select the valid trigger input: write the TS bits to 101 in
      the TIMx_SMCR register (TI1FP1 selected).
    ● Configure the slave mode controller in reset mode: write the
      SMS bits to 100 in the TIMx_SMCR register.
    ● Enable the captures: write the CC1E and CC2E bits to ‘1’
      in the TIMx_CCER register
	*/

    /* Compute the prescaler value */
	PrescalerValue = (uint16_t) (SystemCoreClock / 500) - 1;
	/* Set Prescaler to specified value */
	TIM3->PSC = PrescalerValue;
	TIM3->ARR = 0xFFFFF;

	/* ENABLE CHANNELS 3 & 4 */
	
	/* Capture Compare Mode Register 2 
	   Input capture 3 mapped to timer input 3
	   Input capture 4 mapped to timer input 4
	   NO input filter
	   NO input prescaler */    
	TIM3->CCMR2 =  TIM_CCMR2_CC3S_1 | TIM_CCMR2_CC4S_1;


	/* Configure TIM3 to send interrupts when it detects transition in either channels, 3 or 4 */
	TIM3->DIER = TIM_DIER_CC3IE | TIM_DIER_CC4IE; 
	/* Enable capture */
    TIM3->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;
	
	/* Set polarity */
	TIM3->CCER |= TIM_CCER_CC3P | TIM_CCER_CC4P;
	
	
	/* Configure slave mode controller in reset mode */
	TIM3->SMCR |= TIM_SMCR_SMS_2;
	
/*	TIM3->DIER = TIM_DIER_UIE; */
	
	/* Enable timer */
	TIM3->CR1 = TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM3_IRQn);
}


static void nvic_init(void)
{
    
 	/*  Configure SysTick to tick every ms */
    if (SysTick_Config((uint16_t) (SystemCoreClock / 1000)  - 1)) {
        while (1);              /* Capture error */
    }
}

void TIM3_IRQHandler(void)
{
	/*if (TIM3->SR & TIM_SR_CC3OF) {
		TIM3->SR &= ~TIM_SR_CC3OF;
	} else if (TIM3->SR & TIM_SR_CC4OF) {
		TIM3->SR &= ~TIM_SR_CC3OF; 
	} else */ if (TIM3->SR & TIM_SR_CC3IF) {
		TIM3->SR &= ~TIM_SR_CC3IF; /* Clear the interrupt flag */
	  	LED_GPIO->BRR = (1 << GREEN_LED_PIN); /* toggle LED state */
	} else if (TIM3->SR & TIM_SR_CC4IF) {
		TIM3->SR &= ~TIM_SR_CC4IF; /* Clear the interrupt flag */
	  	LED_GPIO->BSRR = (1 << GREEN_LED_PIN); /* toggle LED state */
	}
}

void SysTick_Handler(void)
{
	ticks++;
}

void delay_ms(uint32_t ival)
{
    uint32_t now = ticks;

    while(ticks - now < ival) {
        asm("nop");
    }
}

/**
  * @brief  Main program.
  * @param  None
  * @retval : None
  */
int main(void)
{
    /*
     * Enable peripherals
     */
    rcc_init();
    nvic_init();
    gpio_init();
//    usart_init();
    tim_init();

    while (1) {
		LED_GPIO->ODR ^= (1 << BLUE_LED_PIN);
        delay_ms(100);
		LED_GPIO->ODR ^= (1 << BLUE_LED_PIN);
		delay_ms(700);
    }
    
    return 0;
}