

#include "stm32f1xx.h"
#include <inverse_kinematics.h>
#include <math.h>

// Define constants (set externally)
double d=2;  //distance from the center of the base to any of its corners
double e=3.25;  //distance from the center of the platform to any of its corners
double f=1.75;  //length of link #1
double g=3.669291339;  //length of link #2
// Helper internal variables
static double nmag, nz;
static double x, y, z;
static double mag, angle;

double IK_theta(uint8_t leg, double hz, double nx, double ny) {
    nmag = sqrt(nx * nx + ny * ny + 1.0);
    nx /= nmag;
    ny /= nmag;
    nz = 1.0 / nmag;

    switch (leg) {
        case LEG_A:
            y = d + (e / 2.0) * (1.0 - ((nx * nx + 3.0 * nz * nz + 3.0 * nz) /
                 (nz + 1.0 - nx * nx + (pow(nx, 4) - 3.0 * nx * nx * ny * ny) /
                 ((nz + 1.0) * (nz + 1.0 - nx * nx)))));
            z = hz + e * ny;
            mag = sqrt(y * y + z * z);
            angle = acos(y / mag) + acos((mag * mag + f * f - g * g) / (2.0 * mag * f));
            break;

        case LEG_B:
            x = (sqrt(3.0) / 2.0) * (e * (1.0 - (nx * nx + sqrt(3.0) * nx * ny) / (nz + 1.0)) - d);
            y = x / sqrt(3.0);
            z = hz - (e / 2.0) * (sqrt(3.0) * nx + ny);
            mag = sqrt(x * x + y * y + z * z);
            angle = acos((sqrt(3.0) * x + y) / (-2.0 * mag)) + acos((mag * mag + f * f - g * g) / (2.0 * mag * f));
            break;

        case LEG_C:
            x = (sqrt(3.0) / 2.0) * (d - e * (1.0 - (nx * nx - sqrt(3.0) * nx * ny) / (nz + 1.0)));
            y = -x / sqrt(3.0);
            z = hz + (e / 2.0) * (sqrt(3.0) * nx - ny);
            mag = sqrt(x * x + y * y + z * z);
            angle = acos((sqrt(3.0) * x - y) / (2.0 * mag)) + acos((mag * mag + f * f - g * g) / (2.0 * mag * f));
            break;

        default:
            return -1.0;  // invalid leg
    }

    return angle * (180.0 / M_PI);
}
