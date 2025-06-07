#ifndef BALL_BALANCING_H
#define BALL_BALANCING_H

#include <stepper_driver.h>
#include <multi_stepper.h>
#include <stdbool.h>
#include <inverse_kinematics.h>

// === External Types ===
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} TSPoint;

// === Global Variables ===
extern double Xoffset;
extern double Yoffset;
extern double angToStep;
extern double ks;

extern Stepper stepperA;
extern Stepper stepperB;
extern Stepper stepperC;
extern MultiStepper steppers;

extern long steppersPosition[3];
extern double steppersSpeed[3];
extern double stepperPreviousSpeed[3];

extern double Kp_x, Ki_x, Kd_x;
extern double Kp_y, Ki_y, Kd_y;

extern double errorXY[2];
extern double previousErrorXY[2];
extern double integralXY[2];
extern double derivativeXY[2];
extern double outputXY[2];

// === Function Declarations ===
void PID(double setpointX, double setpointY);void setup(void);
void loop(void);
void PID(double setpointX, double setpointY);
void moveTo(double hz, double nx, double ny);
void moveToPID(int X, int Y, int wait);
void linePattern(double rx, int ry, int wait, int num);
void trianglePattern(int num);
void squarePattern(int num);
void pinBallPattern(int Y, int wait);
void ellipsePattern(double rx, int ry, double start, int wait, int num);
void sinusoidalPattern(double ampli, double freq, int wait);
void figure8Pattern(double r, double start, int wait, int num);
void DEMO(void);
// === External Dependencies ===
TSPoint ts_getPoint(void);          // Touchscreen read function
long Stepper_currentPosition(Stepper* stepper);
void delay(uint32_t ms);
double constrain(double val, double minVal, double maxVal);
double IK_theta(uint8_t leg, double hz, double nx, double ny);
uint32_t millis(void);

// === Optional External ===
extern bool detected;
extern double originalAngleOfThePaltform;

#endif // BALL_BALANCING_H
