/**
 * @file PID.h
 * @brief PID controller implementation with anti-windup and saturation
 */

#ifndef _PID_H
#define _PID_H

#include <stdlib.h>

// Custom data types for consistent sizing
typedef signed   char  sbyte1;
typedef signed   int   sbyte4;
typedef unsigned char  ubyte1;
typedef unsigned short ubyte2;
typedef unsigned int   ubyte4;

typedef enum {
    FALSE = 0,
    TRUE = 1
} bool;

/**
 * PID controller structure
 * Implements a discrete PID controller with anti-windup protection
 */
typedef struct _PID {
    sbyte1 Kp;               // Proportional gain
    sbyte1 Ki;               // Integral gain
    sbyte1 Kd;               // Derivative gain
    sbyte4 setpoint;         // Target value
    sbyte4 previousError;    // Last error for derivative term
    sbyte4 totalError;       // Accumulated error for integral term
    sbyte4 dH;               // Time step scaling factor
    sbyte4 output;           // Controller output
    sbyte4 proportional;     // P term
    sbyte4 integral;         // I term
    sbyte4 derivative;       // D term
    sbyte4 saturationValue;  // Output saturation limit
    bool   antiWindupFlag;   // Anti-windup status
} PID;

// Core functions
PID* PID_new(sbyte1 Kp, sbyte1 Ki, sbyte1 Kd, sbyte4 saturationValue);

// Configuration functions
void PID_updateGainValues(PID* pid, sbyte1 Kp, sbyte1 Ki, sbyte1 Kd);
void PID_setTotalError(PID* pid, sbyte4 error);
void PID_setSaturationPoint(PID *pid, sbyte4 saturationValue);
void PID_updateSetpoint(PID *pid, sbyte4 setpoint);

// Control functions
void PID_computeOutput(PID *pid, sbyte4 sensorValue);

// Status getters
sbyte1 PID_getKp(PID *pid);
sbyte1 PID_getKi(PID *pid);
sbyte1 PID_getKd(PID *pid);
sbyte4 PID_getSetpoint(PID *pid);
sbyte4 PID_getPreviousError(PID *pid);
sbyte4 PID_getTotalError(PID* pid);
sbyte4 PID_getOutput(PID *pid);
sbyte4 PID_getProportional(PID *pid);
sbyte4 PID_getIntegral(PID *pid);
sbyte4 PID_getDerivative(PID *pid);
sbyte4 PID_getSaturationValue(PID *pid);
bool   PID_getAntiWindupFlag(PID *pid);

#endif //_PID_H 