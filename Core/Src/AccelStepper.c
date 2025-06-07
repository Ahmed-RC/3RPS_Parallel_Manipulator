#include "stm32f1xx.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stepper_driver.h>


typedef struct {
  uint16_t step;
  uint16_t dir;
} Stepp;

// System tick function prototype (user must implement)
extern uint32_t micros();
extern void step_motor(bool direction); // User must implement this per stepper

void assignPins(Stepper* stepper, uint8_t id) {
    initGPIO(id);
    if (id == 0) {
        stepper->step = 4; // PA4
        stepper->dir  = 5; // PA5
    } else if (id == 1) {
        stepper->step = 6; // PA6
        stepper->dir  = 7; // PA7
    } else if (id == 2) {
        stepper->step = 0; // PB0
        stepper->dir  = 1; // PB1
    }
}

void Stepper_Init(Stepper* stepper,int id)
{
    stepper->currentPos = 0;
    stepper->targetPos = 0;
    stepper->speed = 0.0f;
    stepper->maxSpeed = 1000.0f;
    stepper->acceleration = 200.0f;
    stepper->n = 0;
    stepper->c0 = 0.676 * sqrtf(2.0f / stepper->acceleration) * 1e6f;
    stepper->cn = stepper->c0;
    stepper->cmin = 1e6f / stepper->maxSpeed;
    stepper->direction = DIRECTION_CW;
    stepper->stepInterval = 0;
    stepper->lastStepTime = 0;
    assignPins(stepper, id);
}

void initGPIO(uint8_t id) {
    if (id == 0) {
        // Enable GPIOA clock
        RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

        // Configure PA4
        GPIOA->CRL &= ~(GPIO_CRL_MODE4 | GPIO_CRL_CNF4);
        GPIOA->CRL |= (GPIO_CRL_MODE4_1 | GPIO_CRL_CNF4_0); // Output 2 MHz

        // Configure PA5
        GPIOA->CRL &= ~(GPIO_CRL_MODE5 | GPIO_CRL_CNF5);
        GPIOA->CRL |= (GPIO_CRL_MODE5_1 | GPIO_CRL_CNF5_0);

    } else if (id == 1) {
        // Enable GPIOA clock
        RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

        // Configure PA6
        GPIOA->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6);
        GPIOA->CRL |= (GPIO_CRL_MODE6_1 | GPIO_CRL_CNF6_0);

        // Configure PA7
        GPIOA->CRL &= ~(GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
        GPIOA->CRL |= (GPIO_CRL_MODE7_1 | GPIO_CRL_CNF7_0);

    } else if (id == 2) {
        // Enable GPIOB clock
        RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

        // Configure PB0
        GPIOB->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);
        GPIOB->CRL |= (GPIO_CRL_MODE0_1 | GPIO_CRL_CNF0_0);

        // Configure PB1
        GPIOB->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1);
        GPIOB->CRL |= (GPIO_CRL_MODE1_1 | GPIO_CRL_CNF1_0);
    }
}


void Stepper_SetMaxSpeed(Stepper* stepper, float speed)
{
    if (speed < 0.0f) speed = -speed;
    stepper->maxSpeed = speed;
    stepper->cmin = 1e6f / speed;
}

void Stepper_SetAcceleration(Stepper* stepper, float acceleration)
{
    if (acceleration == 0.0f) return;
    if (acceleration < 0.0f) acceleration = -acceleration;
    stepper->acceleration = acceleration;
    stepper->c0 = 0.676 * sqrtf(2.0f / stepper->acceleration) * 1e6f;
    stepper->cn = stepper->c0;
}

void Stepper_MoveTo(Stepper* stepper, long absolute)
{
    if (stepper->targetPos != absolute)
    {
        stepper->targetPos = absolute;
        Stepper_ComputeNewSpeed(stepper);
    }
}

void Stepper_RunToPosition(Stepper* stepper)
{
    while (Stepper_Run(stepper)) {}
}

void Stepper_Stop(Stepper* stepper)
{
    if (stepper->speed != 0.0f)
    {
        long stepsToStop = (long)((stepper->speed * stepper->speed) / (2.0f * stepper->acceleration)) + 1;
        if (stepper->speed > 0)
            Stepper_MoveTo(stepper, stepper->currentPos + stepsToStop);
        else
            Stepper_MoveTo(stepper, stepper->currentPos - stepsToStop);
    }
}

long Stepper_DistanceToGo(Stepper* stepper)
{
    return stepper->targetPos - stepper->currentPos;
}

void Stepper_SetCurrentPosition(Stepper* stepper, long pos)
{
    stepper->currentPos = pos;
    stepper->targetPos = pos;
    stepper->n = 0;
    stepper->speed = 0.0f;
    stepper->stepInterval = 0;
}

unsigned long Stepper_ComputeNewSpeed(Stepper* stepper)
{
    long distance = Stepper_DistanceToGo(stepper);
    long stepsToStop = (long)((stepper->speed * stepper->speed) / (2.0f * stepper->acceleration));

    if (distance == 0 && stepsToStop <= 1)
    {
        stepper->stepInterval = 0;
        stepper->speed = 0.0f;
        stepper->n = 0;
        return 0;
    }

    if (distance > 0)
    {
        if (stepper->n > 0 && (stepsToStop >= distance || stepper->direction == DIRECTION_CCW))
            stepper->n = -stepsToStop;
        else if (stepper->n < 0 && stepper->direction == DIRECTION_CW && stepsToStop < distance)
            stepper->n = -stepper->n;
    }
    else if (distance < 0)
    {
        if (stepper->n > 0 && (stepsToStop >= -distance || stepper->direction == DIRECTION_CW))
            stepper->n = -stepsToStop;
        else if (stepper->n < 0 && stepper->direction == DIRECTION_CCW && stepsToStop < -distance)
            stepper->n = -stepper->n;
    }

    if (stepper->n == 0)
    {
        stepper->cn = stepper->c0;
        stepper->direction = (distance > 0) ? DIRECTION_CW : DIRECTION_CCW;
    }
    else
    {
        stepper->cn = stepper->cn - ((2.0f * stepper->cn) / ((4.0f * stepper->n) + 1));
        if (stepper->cn < stepper->cmin)
            stepper->cn = stepper->cmin;
    }

    stepper->n++;
    stepper->stepInterval = (unsigned long)stepper->cn;
    stepper->speed = 1e6f / stepper->cn;
    if (stepper->direction == DIRECTION_CCW)
        stepper->speed = -stepper->speed;

    return stepper->stepInterval;
}

bool Stepper_RunSpeed(Stepper* stepper)
{
    uint32_t now = micros();
    if (stepper->stepInterval == 0 || (now - stepper->lastStepTime) < stepper->stepInterval)
        return false;

    if (stepper->direction == DIRECTION_CW)
        stepper->currentPos++;
    else
        stepper->currentPos--;

    step_motor(stepper->direction);
    stepper->lastStepTime = now;
    return true;
}

bool Stepper_Run(Stepper* stepper)
{
    if (Stepper_RunSpeed(stepper))
        Stepper_ComputeNewSpeed(stepper);
    return stepper->speed != 0.0f || Stepper_DistanceToGo(stepper) != 0;
}
