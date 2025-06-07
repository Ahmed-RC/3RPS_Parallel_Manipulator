#ifndef TOUCHSCREEN_H
#define TOUCHSCREEN_H

#include <stdint.h>
#include <BallBalancing.h>

// === Data Structures ===
//typedef struct {
//    int16_t x;
//    int16_t y;
//    int16_t z;  // optional pressure
//} TSPoint;

typedef struct {
    float x_mm;
    float y_mm;
    int16_t z;
} TSRealPoint;

// === Global Filter Toggle ===
extern uint8_t use_filter;

// === Public API ===
void ADC1_Init(void);
uint16_t ADC1_Read_Channel(uint8_t channel);
uint16_t averaged_ADC(uint8_t channel);

int readTouchX(void);
int readTouchY(void);
int readTouchPressure(void);

TSPoint getTouchPoint(void);
TSRealPoint getRealTouchPoint(void);

void check_filter_toggle(void);

// === Utility ===
void delay_us(uint32_t us);

#endif // TOUCHSCREEN_H
