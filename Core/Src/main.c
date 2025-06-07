// === touchscreen.c ===

#include "stm32f10x.h"
#include <stdint.h>

// Touchscreen pin definitions (adjust based on your wiring)
#define XP_PIN 0  // PA0
#define XM_PIN 1  // PA1
#define YP_PIN 2  // PA2
#define YM_PIN 3  // PA3

// Calibrated ADC ranges (measured manually)
#define X_MIN 200
#define X_MAX 3800
#define Y_MIN 250
#define Y_MAX 3700

#define SCREEN_WIDTH_MM 80.0f
#define SCREEN_HEIGHT_MM 50.0f

// Touch position structure
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z; // optional pressure
} TSPoint;

// Real-world coordinates in mm
typedef struct {
    float x_mm;
    float y_mm;
    int16_t z;
} TSRealPoint;

// === Delay helper ===
void delay_us(uint32_t us) {
    for (volatile uint32_t i = 0; i < us * 8; i++);
}

// === ADC Initialization ===
void ADC1_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;      // Enable ADC1 clock
    ADC1->CR2 |= ADC_CR2_ADON;               // Power on
    delay_us(10);
    //SET SAMPLING RATE TO 41.5 CYCLES (ADC_SMPR2_SMP_41_5)
    ADC1->SMPR2 |= ADC_SMPR2_SMP0;           // Set sampling time for channel 0 (PA0)
    ADC1->CR2 |= ADC_CR2_RSTCAL;             // Reset calibration
    while (ADC1->CR2 & ADC_CR2_RSTCAL);
    ADC1->CR2 |= ADC_CR2_CAL;                // Start calibration
    while (ADC1->CR2 & ADC_CR2_CAL);
}

// === ADC Read (single channel) ===
uint16_t ADC1_Read_Channel(uint8_t channel) {
    ADC1->SQR3 = channel;            // Select channel
    ADC1->CR2 |= ADC_CR2_ADON;       // Start conversion
    while (!(ADC1->SR & ADC_SR_EOC)); // Wait for conversion complete
    return ADC1->DR;                 // Return result
}

// === Read X Position ===
int readTouchX(void) {
    // XP = HIGH, XM = LOW, YP = analog input, YM = Hi-Z

    // XP = PA0 → Output HIGH
    GPIOA->CRL &= ~(0xF << (XP_PIN * 4));
    GPIOA->CRL |=  (0x1 << (XP_PIN * 4));
    GPIOA->BSRR = (1 << XP_PIN);

    // XM = PA1 → Output LOW
    GPIOA->CRL &= ~(0xF << (XM_PIN * 4));
    GPIOA->CRL |=  (0x1 << (XM_PIN * 4));
    GPIOA->BRR = (1 << XM_PIN);

    // YP = PA2 → Analog input
    GPIOA->CRL &= ~(0xF << (YP_PIN * 4));

    delay_us(20);
    return 4095 - ADC1_Read_Channel(YP_PIN);  // Read from Y+ (PA2)
}

// === Read Y Position ===
int readTouchY(void) {
    // YP = HIGH, YM = LOW, XP = analog input, XM = Hi-Z

    // YP = PA2 → Output HIGH
    GPIOA->CRL &= ~(0xF << (YP_PIN * 4));
    GPIOA->CRL |=  (0x1 << (YP_PIN * 4));
    GPIOA->BSRR = (1 << YP_PIN);

    // YM = PA3 → Output LOW
    GPIOA->CRL &= ~(0xF << (YM_PIN * 4));
    GPIOA->CRL |=  (0x1 << (YM_PIN * 4));
    GPIOA->BRR = (1 << YM_PIN);

    // XP = PA0 → Analog input
    GPIOA->CRL &= ~(0xF << (XP_PIN * 4));

    delay_us(20);
    return 4095 - ADC1_Read_Channel(XP_PIN);  // Read from X+ (PA0)
}

// === Read Pressure ===
int readTouchPressure(void) {
    // XP = GND, YM = VCC, XM = analog input, YP = analog input

    // XP = PA0 → Output LOW
    GPIOA->CRL &= ~(0xF << (XP_PIN * 4));
    GPIOA->CRL |=  (0x1 << (XP_PIN * 4));
    GPIOA->BRR = (1 << XP_PIN);

    // YM = PA3 → Output HIGH
    GPIOA->CRL &= ~(0xF << (YM_PIN * 4));
    GPIOA->CRL |=  (0x1 << (YM_PIN * 4));
    GPIOA->BSRR = (1 << YM_PIN);

    // XM = PA1 → Analog input
    GPIOA->CRL &= ~(0xF << (XM_PIN * 4));

    // YP = PA2 → Analog input
    GPIOA->CRL &= ~(0xF << (YP_PIN * 4));

    delay_us(20);
    int z1 = ADC1_Read_Channel(XM_PIN);
    int z2 = ADC1_Read_Channel(YP_PIN);

    return 4095 - (z2 - z1);  // Simple pressure approximation
}

// === Combined Point Read ===
TSPoint getTouchPoint(void) {
    TSPoint p;
    p.x = readTouchX();
    delay_us(50);  // Allow voltages to settle
    p.y = readTouchY();
    p.z = readTouchPressure();
    return p;
}

// === Convert raw point to real-world coordinates (mm) ===
TSRealPoint getRealTouchPoint(void) {
    TSPoint raw = getTouchPoint();
    TSRealPoint real;

    real.x_mm = ((float)(raw.x - X_MIN) / (X_MAX - X_MIN)) * SCREEN_WIDTH_MM;
    real.y_mm = ((float)(raw.y - Y_MIN) / (Y_MAX - Y_MIN)) * SCREEN_HEIGHT_MM;
    real.z = raw.z;

    // Clamp to screen bounds
    if (real.x_mm < 0) real.x_mm = 0;
    if (real.x_mm > SCREEN_WIDTH_MM) real.x_mm = SCREEN_WIDTH_MM;
    if (real.y_mm < 0) real.y_mm = 0;
    if (real.y_mm > SCREEN_HEIGHT_MM) real.y_mm = SCREEN_HEIGHT_MM;

    return real;
}
