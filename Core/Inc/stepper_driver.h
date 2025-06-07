#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

// === Direction Constants ===
#define DIRECTION_CW  1  // Clockwise
#define DIRECTION_CCW 0  // Counter-clockwise

// === Stepper Struct ===
typedef struct {
    uint16_t step;
    uint16_t dir;

    long currentPos;
    long targetPos;

    float speed;
    float maxSpeed;
    float acceleration;

    float c0;
    float cn;
    float cmin;
    long n;

    uint8_t direction;
    unsigned long stepInterval;
    uint32_t lastStepTime;
} Stepper;

// === External Functions (must be implemented by user) ===
extern uint32_t micros(void);                    // System time in microseconds
extern void step_motor(bool direction);          // Physical step pulse
void initGPIO(uint8_t id);                       // GPIO setup based on ID

// === Stepper Control Functions ===
void Stepper_Init(Stepper* stepper, int id);
void Stepper_SetMaxSpeed(Stepper* stepper, float speed);
void Stepper_SetAcceleration(Stepper* stepper, float acceleration);
void Stepper_MoveTo(Stepper* stepper, long absolute);
void Stepper_RunToPosition(Stepper* stepper);
void Stepper_Stop(Stepper* stepper);
long Stepper_DistanceToGo(Stepper* stepper);
void Stepper_SetCurrentPosition(Stepper* stepper, long pos);
unsigned long Stepper_ComputeNewSpeed(Stepper* stepper);
bool Stepper_RunSpeed(Stepper* stepper);
bool Stepper_Run(Stepper* stepper);

#endif // STEPPER_DRIVER_H
