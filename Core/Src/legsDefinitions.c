// inverse_kinematics.h

#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <stdint.h>

// Leg identifiers
#define LEG_A 0
#define LEG_B 1
#define LEG_C 2

// Calculate angle (in degrees) for a given leg and pose
// hz = height (z), nx = unit normal x-component, ny = unit normal y-component
double IK_theta(uint8_t leg, double hz, double nx, double ny);

#endif // INVERSE_KINEMATICS_H
