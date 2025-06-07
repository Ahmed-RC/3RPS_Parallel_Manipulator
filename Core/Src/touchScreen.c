#include <multi_stepper.h>
#include <math.h>

void MultiStepper_Init(MultiStepper* ms)
{
    ms->num_steppers = 0;
}

bool MultiStepper_AddStepper(MultiStepper* ms, Stepper* stepper)
{
    if (ms->num_steppers >= MULTISTEPPER_MAX_STEPPERS)
        return false;
    ms->steppers[ms->num_steppers++] = stepper;
    return true;
}

void MultiStepper_MoveTo(MultiStepper* ms, long* absolute_positions)
{
    float longestTime = 0.0f;

    for (uint8_t i = 0; i < ms->num_steppers; i++) {
        long distance = absolute_positions[i] - ms->steppers[i]->currentPos;
        float time = fabsf((float)distance) / ms->steppers[i]->maxSpeed;
        if (time > longestTime)
            longestTime = time;
    }

    if (longestTime > 0.0f) {
        for (uint8_t i = 0; i < ms->num_steppers; i++) {
            long distance = absolute_positions[i] - ms->steppers[i]->currentPos;
            float newSpeed = distance / longestTime;

            Stepper_MoveTo(ms->steppers[i], absolute_positions[i]);
            ms->steppers[i]->speed = newSpeed;
            ms->steppers[i]->stepInterval = (unsigned long)(1e6f / fabsf(newSpeed));
        }
    }
}

bool MultiStepper_Run(MultiStepper* ms)
{
    bool anyRunning = false;

    for (uint8_t i = 0; i < ms->num_steppers; i++) {
        if (Stepper_DistanceToGo(ms->steppers[i]) != 0) {
            Stepper_RunSpeed(ms->steppers[i]);
            anyRunning = true;
        }
    }

    return anyRunning;
}

void MultiStepper_RunSpeedToPosition(MultiStepper* ms)
{
    while (MultiStepper_Run(ms));
}
