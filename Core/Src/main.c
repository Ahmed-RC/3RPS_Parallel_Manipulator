#include "stm32f10x.h"

volatile uint16_t x_pos = 0;
volatile uint16_t y_pos = 0;

// === Delay Function ===
void delay_us(uint32_t time) {
    for (volatile uint32_t i = 0; i < time * 8; i++);
}

// === ADC Initialization ===
void ADC1_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    ADC1->CR2 |= ADC_CR2_ADON;  // Power on
    for (volatile int i = 0; i < 1000; i++);

    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while (ADC1->CR2 & ADC_CR2_RSTCAL);

    ADC1->CR2 |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL);
}

uint16_t ADC1_Read_Channel(uint8_t channel) {
    ADC1->SQR3 = channel;               // Select ADC channel
    ADC1->CR2 |= ADC_CR2_ADON;          // Start conversion
    while (!(ADC1->SR & ADC_SR_EOC));   // Wait until done
    return ADC1->DR;
}

// === GPIO Setup ===
void GPIO_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;  // Enable GPIOA clock
}

// === Read X Position (Y+ = ADC, X+/X− = VCC/GND) ===
void Read_X_Position(void) {
    // X+ (PA0) = VCC
    GPIOA->CRL &= ~(0xF << (0 * 4));        // Clear bits
    GPIOA->CRL |= (0x1 << (0 * 4));         // Output mode
    GPIOA->BSRR = GPIO_BSRR_BS0;

    // X− (PA1) = GND
    GPIOA->CRL &= ~(0xF << (1 * 4));
    GPIOA->CRL |= (0x1 << (1 * 4));
    GPIOA->BRR = GPIO_BRR_BR1;

    // Y+ (PA2) = ADC input
    GPIOA->CRL &= ~(0xF << (2 * 4));        // Analog input

    delay_us(10);  // Let voltages stabilize

    x_pos = ADC1_Read_Channel(2);  // Read PA2
}

// === Read Y Position (X+ = ADC, Y+/Y− = VCC/GND) ===
void Read_Y_Position(void) {
    // Y+ (PA2) = VCC
    GPIOA->CRL &= ~(0xF << (2 * 4));
    GPIOA->CRL |= (0x1 << (2 * 4));
    GPIOA->BSRR = GPIO_BSRR_BS2;

    // Y− (PA3) = GND
    GPIOA->CRL &= ~(0xF << (3 * 4));
    GPIOA->CRL |= (0x1 << (3 * 4));
    GPIOA->BRR = GPIO_BRR_BR3;

    // X+ (PA0) = ADC input
    GPIOA->CRL &= ~(0xF << (0 * 4));  // Analog input

    delay_us(10);

    y_pos = ADC1_Read_Channel(0);  // Read PA0
}

// === PWM Initialization (TIM3_CH1 on PA6) ===
void PWM_INIT(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // PA6 = TIM3_CH1 (AF output push-pull)
    GPIOA->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6);
    GPIOA->CRL |= GPIO_CRL_MODE6_1 | GPIO_CRL_MODE6_0 | GPIO_CRL_CNF6_1;

    TIM3->PSC = 7200 - 1;       // 72 MHz / 7200 = 10 kHz timer clock
    TIM3->ARR = 1000;           // Period = 100 ms
    TIM3->CCR1 = 500;           // 50% initial duty

    TIM3->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;  // PWM mode 1
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE;                     // Preload enable
    TIM3->CCER |= TIM_CCER_CC1E;                        // Enable output
    TIM3->CR1 |= TIM_CR1_CEN;                           // Start timer
    TIM3->EGR |= TIM_EGR_UG;                            // Force update
}

// === Set PWM Duty Cycle (scaled from 0–4095 to 0–100%) ===
void PWM_SET_DUTY(uint16_t adc_value) {
    uint16_t duty = (adc_value * TIM3->ARR) / 4095;
    TIM3->CCR1 = duty;
}

// === Main Program ===
int main(void) {
    GPIO_Init();
    ADC1_Init();
    PWM_INIT();

    while (1) {
        Read_X_Position();          // Update x_pos from touchscreen
        PWM_SET_DUTY(x_pos);        // Set duty based on x_pos
        delay_us(1000);             // Optional delay
    }
}
