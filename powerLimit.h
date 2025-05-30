/**
 * @file powerLimit.h
 * @brief Power limiting implementation using PID controller and lookup tables
 */

#ifndef _POWERLIMIT_H
#define _POWERLIMIT_H

#include "pid.h"

// Forward declaration of motor controller structure
typedef struct _MotorController MotorController;

/**
 * Power limiting controller structure
 * Implements power limiting using PID control and lookup tables
 */
typedef struct _PowerLimit {
    PID    *pid;                        // PID controller instance
    bool    plStatus;                   // Power limiting active status
    ubyte1  plMode;                     // Limiting method (1-4)
    ubyte1  plTargetPower;              // Target power in kW
    ubyte1  plInitializationThreshold;  // Power threshold for activation
    sbyte4  plTorqueCommand;            // Output torque command (deci-Nm)
    
    // LUT corner points for debugging
    ubyte1 vFloorRFloor;
    ubyte1 vFloorRCeiling;
    ubyte1 vCeilingRFloor;
    ubyte1 vCeilingRCeiling;
} PowerLimit;

// Core functions
PowerLimit* POWERLIMIT_new();
void POWERLIMIT_setLimpModeOverride(PowerLimit* me);
void PowerLimit_calculateCommand(PowerLimit *me, MotorController *mcm);

// Power limiting methods
void POWERLIMIT_calculateTorqueCommand(PowerLimit *me, MotorController *mcm);           // LUT-based
void POWERLIMIT_calculateTorqueCommandTorqueEquation(PowerLimit *me, MotorController *mcm);  // Mechanical equation
void POWERLIMIT_calculateTorqueCommandPowerPID(PowerLimit *me, MotorController *mcm);   // PID-based

// LUT operations
sbyte4 POWERLIMIT_retrieveTorqueFromLUT(PowerLimit* me, sbyte4 noLoadVoltage, sbyte4 rpm);
ubyte1 POWERLIMIT_getTorqueFromArray(ubyte4 noLoadVoltage, ubyte4 rpm);

// Status and configuration getters
ubyte1 POWERLIMIT_getStatusCodeBlock(PowerLimit* me);
bool   POWERLIMIT_getStatus(PowerLimit* me);
ubyte1 POWERLIMIT_getMode(PowerLimit* me);
sbyte4 POWERLIMIT_getTorqueCommand(PowerLimit* me);
ubyte1 POWERLIMIT_getTargetPower(PowerLimit* me);
ubyte1 POWERLIMIT_getInitialisationThreshold(PowerLimit* me);
ubyte1 POWERLIMIT_getLUTCorner(PowerLimit* me, ubyte1 corner);

#endif //_POWERLIMIT_H 