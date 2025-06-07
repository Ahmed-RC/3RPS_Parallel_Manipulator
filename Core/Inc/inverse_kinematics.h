#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <stdint.h>

// === Leg Identifiers ===
#define LEG_A 0
#define LEG_B 1
#define LEG_C 2

// === Global Parameters (geometry) ===
extern double d;  // Base to corner
extern double e;  // Platform to corner
extern double f;  // Link #1 length
extern double g;  // Link #2 length

// === Function Declaration ===
double IK_theta(uint8_t leg, double hz, double nx, double ny);

#endif // INVERSE_KINEMATICS_H
