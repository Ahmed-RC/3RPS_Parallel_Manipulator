#include "BallBalancing.h"
#include "stepper_driver.h"
#include "multi_stepper.h"
#include "inverse_kinematics.h"
#include "touchscreen.h"

#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

Stepper stepperA;  // Stepper A
Stepper stepperB;  // Stepper B
Stepper stepperC;  // Stepper C
MultiStepper steppers;  // MultiStepper instance
uint32_t timeII;

long steppersPosition[3] = {0, 0, 0};  // Stepper target positions
double originalAngleOfThePaltform = 206.662752199;
double steppersSpeed[3] = {0, 0, 0};
double stepperPreviousSpeed[3] = {0, 0, 0};
double ks = 20;

double Xoffset = 500;
double Yoffset = 500;

// PID constants
double Kp_x = 20, Ki_x = 2, Kd_x = 100;
double Kp_y = 10, Ki_y = 0.4, Kd_y = 120;

// Control variables
double errorXY[2] = {0, 0};
double previousErrorXY[2] = {0, 0};
double integralXY[2] = {0, 0};
double derivativeXY[2] = {0, 0};
double outputXY[2] = {0, 0};

double angToStep = 200.0 / 360.0;
bool detected = false;

void setup() {
    Stepper_Init(&stepperA, 0);
    Stepper_Init(&stepperB, 1);
    Stepper_Init(&stepperC, 2);

    MultiStepper_Init(&steppers);

    MultiStepper_AddStepper(&steppers, &stepperA);
    MultiStepper_AddStepper(&steppers, &stepperB);
    MultiStepper_AddStepper(&steppers, &stepperC);

    moveTo(4.25, 0, 0);
    MultiStepper_RunSpeedToPosition(&steppers);
}

void loop() {
    PID(0, 0);
}

void moveTo(double hz, double nx, double ny) {
    if (detected) {
        for (int i = 0; i < 3; i++) {
            steppersPosition[i] = round((originalAngleOfThePaltform - IK_theta(i, hz, nx, ny)) * angToStep);
        }

        Stepper_SetMaxSpeed(&stepperA, steppersSpeed[0]);
        Stepper_SetMaxSpeed(&stepperB, steppersSpeed[1]);
        Stepper_SetMaxSpeed(&stepperC, steppersSpeed[2]);

        Stepper_SetAcceleration(&stepperA, steppersSpeed[0] * 30);
        Stepper_SetAcceleration(&stepperB, steppersSpeed[1] * 30);
        Stepper_SetAcceleration(&stepperC, steppersSpeed[2] * 30);

        Stepper_MoveTo(&stepperA, steppersPosition[0]);
        Stepper_MoveTo(&stepperB, steppersPosition[1]);
        Stepper_MoveTo(&stepperC, steppersPosition[2]);

        Stepper_Run(&stepperA);
        Stepper_Run(&stepperB);
        Stepper_Run(&stepperC);
    } else {
        for (int i = 0; i < 3; i++) {
            steppersPosition[i] = round((originalAngleOfThePaltform - IK_theta(i, hz, 0, 0)) * angToStep);
        }

        Stepper_SetMaxSpeed(&stepperA, 800);
        Stepper_SetMaxSpeed(&stepperB, 800);
        Stepper_SetMaxSpeed(&stepperC, 800);

        MultiStepper_MoveTo(&steppers, steppersPosition);
        MultiStepper_Run(&steppers);
    }
}

void PID(double setpointX, double setpointY) {
    TSPoint p = ts_getPoint();

    if (p.x != 0) {
        detected = true;
        for (int i = 0; i < 2; i++) {
            previousErrorXY[i] = errorXY[i];
            errorXY[i] = (i == 0) * (Xoffset - p.x - setpointX) + (i == 1) * (Yoffset - p.y - setpointY);
            integralXY[i] += errorXY[i] + previousErrorXY[i];
            derivativeXY[i] = errorXY[i] - previousErrorXY[i];
            derivativeXY[i] = isnan(derivativeXY[i]) || isinf(derivativeXY[i]) ? 0 : derivativeXY[i];
            outputXY[i] = (i == 0 ? Kp_x : Kp_y) * errorXY[0] + (i == 0 ? Ki_x : Ki_y) * integralXY[0] + (i == 0 ? Kd_x : Kd_y) * derivativeXY[0];
            outputXY[i] = constrain(outputXY[i], -0.25, 0.25);
        }

        for (int i = 0; i < 3; i++) {
            stepperPreviousSpeed[i] = steppersSpeed[i];
            steppersSpeed[i] = (i == 0) * Stepper_currentPosition(&stepperA) + (i == 1) * Stepper_currentPosition(&stepperB) + (i == 2) * Stepper_currentPosition(&stepperC);
            steppersSpeed[i] = fabs(steppersSpeed[i] - steppersPosition[i]) * ks;
            steppersSpeed[i] = constrain(steppersSpeed[i], stepperPreviousSpeed[i] - 200, stepperPreviousSpeed[i] + 200);
            steppersSpeed[i] = constrain(steppersSpeed[i], 0, 1000);
        }
    } else {
        delay(10);
        p = ts_getPoint();
        if (p.x == 0) {
            detected = false;
        }
    }

    uint32_t timeI = millis();
    double out[2] = { outputXY[0], outputXY[1] };
    while (millis() - timeI < 20) {
        moveTo(4.25, -out[0], -out[1]);
    }
}

void moveToPID(int X, int Y, int wait) {
    timeII = millis();
    while (millis() - timeII < wait) {
        PID(X, Y);
    }
}

void linePattern(double rx, int ry, int wait, int num) {
    for (int i = 0; i < num; i++) {
        moveToPID(rx, ry, wait);
        moveToPID(-rx, -ry, wait);
    }
}

void trianglePattern(int num) {
    double s = 400;
    for (int i = 0; i < num; i++) {
        for (int j = 0; j < 3; j++) {
            moveToPID((1 - j) * (s / 2), j == 1 ? s * (sqrt(3) / 4) : -s * (sqrt(3) / 4), 800);
        }
    }
}

void squarePattern(int num) {
    int s = 400;
    for (int i = 0; i < num; i++) {
        moveToPID(s / 2, -s / 2, 700);
        moveToPID(s / 2, s / 2, 700);
        moveToPID(-s / 2, s / 2, 700);
        moveToPID(-s / 2, -s / 2, 700);
    }
}

void pinBallPattern(int Y, int wait) {
    Y *= -1;
    for (int X = 200; X >= -300; X -= 100) {
        moveToPID(X, Y, wait);
        Y *= -1;
    }
    for (int X = -200; X <= 300; X += 100) {
        moveToPID(X, Y, wait);
        Y *= -1;
    }
}

void ellipsePattern(double rx, int ry, double start, int wait, int num) {
    for (int i = 0; i < num; i++) {
        double theta = start;
        for (double j = 0; j <= 2 * M_PI; j += 0.1) {
            moveToPID(rx * cos(theta), ry * sin(theta), wait);
            theta += start == 0 ? 0.1 : (start == 2 * M_PI ? -0.1 : 0);
        }
    }
}

void sinusoidalPattern(double ampli, double freq, int wait) {
    for (double X = 300; X >= -300; X -= 5) {
        moveToPID(X, ampli * sin(X / freq), wait);
    }
    for (double X = -300; X <= 300; X += 5) {
        moveToPID(X, ampli * sin(X / freq), wait);
    }
}

void figure8Pattern(double r, double start, int wait, int num) {
    for (int i = 0; i < num; i++) {
        double theta = start;
        for (double j = 0; j < 2 * M_PI; j += 0.05) {
            double scale = r * (2 / (3 - cos(2 * theta)));
            moveToPID(scale * cos(theta), scale * sin(2 * theta) / 1.5, wait);
            theta += start == 0 ? -0.05 : (start == -2 * M_PI ? 0.05 : 0);
        }
    }
}

void DEMO(void) {
    moveToPID(0, 0, 8000);
    linePattern(200, 0, 1000, 1);
    linePattern(200, 0, 600, 2);
    trianglePattern(2);
    squarePattern(2);
    pinBallPattern(175, 500);
    pinBallPattern(100, 300);
    ellipsePattern(50, 50, 0, 1, 2);
    ellipsePattern(100, 100, 0, 10, 2);
    ellipsePattern(150, 150, 0, 20, 2);
    ellipsePattern(50, 150, 0, 20, 2);
    ellipsePattern(150, 50, 0, 20, 2);
    sinusoidalPattern(50, 30, 10);
    sinusoidalPattern(100, 50, 10);
    sinusoidalPattern(150, 80, 10);
    figure8Pattern(200, 0, 20, 3);
    moveToPID(0, 0, 2000);

    // Home sequence
    long pos[3];
    for (int i = 0; i < 3; i++) {
        pos[i] = round((originalAngleOfThePaltform - IK_theta(i, 4.25, 0, 0.25)) * angToStep);
    }
    Stepper_SetMaxSpeed(&stepperA, 2000);
    Stepper_SetMaxSpeed(&stepperB, 2000);
    Stepper_SetMaxSpeed(&stepperC, 2000);
    MultiStepper_MoveTo(&steppers, pos);
    MultiStepper_RunSpeedToPosition(&steppers);
    delay(1000);
    detected = false;
    moveTo(4.25, 0, 0);
    MultiStepper_RunSpeedToPosition(&steppers);

    for (int i = 0; i < 20; i++) {
        for (int j = 0; j < 3; j++) {
            pos[j] = round((originalAngleOfThePaltform - IK_theta(j, 5 - 1.25 * (i % 2 != 0), 0, 0)) * angToStep);
        }
        Stepper_SetMaxSpeed(&stepperA, 2000);
        Stepper_SetMaxSpeed(&stepperB, 2000);
        Stepper_SetMaxSpeed(&stepperC, 2000);
        MultiStepper_MoveTo(&steppers, pos);
        MultiStepper_RunSpeedToPosition(&steppers);
    }
}



