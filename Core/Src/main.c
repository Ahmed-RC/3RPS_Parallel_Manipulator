/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

#include "stm32f1xx.h"
#include "stm32f1xx_hal_gpio.h"

volatile uint16_t adc_values[2]; // [X, Z]

void ADC1_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN    // Enable GPIOA
                  | RCC_APB2ENR_ADC1EN;   // Enable ADC1 clock

    // Set PA0 and PA1 as analog input (X and Z)
    GPIOA->CRL &= ~((0xF << (0 * 4)) | (0xF << (1 * 4)));  // Clear CNF & MODE
    // 00 00 = Analog mode

    // Set ADC1 in scan mode, continuous mode, enable DMA
    ADC1->CR1 |= ADC_CR1_SCAN;                 // Enable scan mode
    ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_DMA;   // Continuous + DMA
    ADC1->SQR1 = (1 << 20); // 2 conversions: L[23:20] = 1 for 2 conversions

    // Select channels for sequence
    ADC1->SQR3 = (0 << 0)  // 1st conversion: channel 0 (PA0)
               | (1 << 5); // 2nd conversion: channel 1 (PA1)

    // Enable ADC and calibrate
    ADC1->CR2 |= ADC_CR2_ADON;  // Power on
    for (volatile int i = 0; i < 1000; i++); // short delay

    ADC1->CR2 |= ADC_CR2_RSTCAL;             // Reset calibration
    while (ADC1->CR2 & ADC_CR2_RSTCAL);      // Wait

    ADC1->CR2 |= ADC_CR2_CAL;                // Start calibration
    while (ADC1->CR2 & ADC_CR2_CAL);         // Wait

    ADC1->CR2 |= ADC_CR2_ADON;               // Start conversion
}

void DMA1_Channel1_Init(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;  // Enable DMA1 clock

    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;          // Peripheral address
    DMA1_Channel1->CMAR = (uint32_t)adc_values;         // Memory address
    DMA1_Channel1->CNDTR = 2;                           // Number of transfers

    DMA1_Channel1->CCR = DMA_CCR1_MINC     // Increment memory address
                       | DMA_CCR1_PSIZE_0  // Peripheral size = 16-bit
                       | DMA_CCR1_MSIZE_0  // Memory size = 16-bit
                       | DMA_CCR1_CIRC     // Circular mode
                       | DMA_CCR1_EN;      // Enable DMA channel
}

void PWM_INIT(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    GPIOA->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6);
    GPIOA->CRL |= GPIO_CRL_MODE6_1 | GPIO_CRL_MODE6_0 | GPIO_CRL_CNF6_1;

    TIM3->PSC = 7200 - 1;       // Timer = 10 kHz
    TIM3->CCR1 = 500;           // 50% duty

    TIM3->CCMR1 |= TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE;
    TIM3->CCER |= TIM_CCER_CC1E;
    TIM3->CR1 |= TIM_CR1_CEN;
    TIM3->EGR |= TIM_EGR_UG;
}

void PWM_SET_DUTY(uint16_t adc_value) {
    uint16_t percent = ((adc_value ) / 4095); // Scale ADC value to PWM period
    TIM3->CCR1 = (percent * TIM3->ARR);
  }


int main(void) {
    ADC1_Init();
    DMA1_Channel1_Init();
    PWM_INIT();
    while (1) {
      int frequency = adc_values[0]/(4096);
      PWM_SET_DUTY(frequency);
    }
}



// int main(void){

//   RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
//   RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
//   RCC->APB2ENR |= ADC1EN;
//   RCC->APB2ENR |= TIM1EN;
//   RCC->APB2ENR |= USART1EN;
  
//   GPIOA->CRL &=0;
//   GPIOA->CRH &=0;
//   GPIOB->CRL &=0;
//   GPIOB->CRH &=0;

//   //PA0 - PA3 ANALONG INPUTS FOR THE TOUCH SCREEN 

//   ADC1->SQR1 = 0x00000000; // Set the sequence register to 0 for single channel conversion
//   ADC1->SQR3 = 0x00000000; // Set the first conversion in the sequence to channel 0 (PA0)

//   ADC1->CR2 = ADC_CR2_ADON; // Enable the ADC
//   while(!(ADC1->SR & ADC_SR_ADON)); // Wait for the ADC to be ready
//   ADC1->CR2 |=ADC_CR2_CAL; // Start the calibration process
//   while(ADC1->CR2 & ADC_CR2_CAL); // Wait for the calibration to complete
//   ADC1->CR2 |= ADC_CR2_EXTTRIG; // Set the external trigger for the ADC
//   ADC1->CR2 |= ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_0; // Select SWSTART as external trigger source
//   ADC1->CR2 |= ADC_CR2_CONT; // Enable continuous conversion mode
//   ADC1->CR2 |= ADC_CR2_DMA; // Enable DMA for the ADC


// }

