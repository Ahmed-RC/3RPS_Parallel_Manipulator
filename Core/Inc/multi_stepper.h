#ifndef MULTI_STEPPER_H
#define MULTI_STEPPER_H

#include <stdint.h>
#include <stdbool.h>
#include <stepper_driver.h>

// ===== Constants =====
#define MULTISTEPPER_MAX_STEPPERS 3

// ===== MultiStepper Struct =====
typedef struct {
    Stepper* steppers[MULTISTEPPER_MAX_STEPPERS];
    uint8_t num_steppers;
} MultiStepper;

// ===== Public Function Prototypes =====
void MultiStepper_Init(MultiStepper* ms);
bool MultiStepper_AddStepper(MultiStepper* ms, Stepper* stepper);
void MultiStepper_MoveTo(MultiStepper* ms, long* absolute_positions);
bool MultiStepper_Run(MultiStepper* ms);
void MultiStepper_RunSpeedToPosition(MultiStepper* ms);

#endif // MULTI_STEPPER_H
