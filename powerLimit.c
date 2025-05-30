/**
 * @file powerLimit.c
 * @brief Power limiting implementation using PID control and lookup tables
 */

#include <stdlib.h>
#include "powerlimit.h"

// Helper functions for LUT interpolation
static inline ubyte4 ubyte4_lowerStepInterval(ubyte4 val, ubyte4 step) {
    return (val / step) * step;
}

static inline ubyte4 ubyte4_upperStepInterval(ubyte4 val, ubyte4 step) {
    return ((val + step - 1) / step) * step;
}

// LUT configuration
#define VOLTAGE_STEP     5    // Voltage step size for LUT
#define RPM_STEP         160  // RPM step size for LUT

/**
 * Creates a new power limiting controller
 * Initializes with default PID gains and 80kW target
 */
PowerLimit* POWERLIMIT_new() {
    PowerLimit* me = (PowerLimit*)malloc(sizeof(PowerLimit));
    if (me == NULL) return NULL;

    // Initialize PID controller (Kp=10, Ki=0, Kd=0)
    me->pid = PID_new(10, 0, 0, 0);

    // Set default operating mode
    me->plMode = 1;  // 1=TQ eqn, 2=Power PID, 3=LUT, 4=Combined
    me->plStatus = FALSE;
    me->plTorqueCommand = 0;
    me->plTargetPower = 80;  // Target power in kW
    me->plInitializationThreshold = me->plTargetPower - 15;

    // Initialize LUT corner points
    me->vFloorRFloor = 0;
    me->vFloorRCeiling = 0;
    me->vCeilingRFloor = 0;
    me->vCeilingRCeiling = 0;

    return me;
}

/**
 * Sets limp mode parameters (placeholder for future implementation)
 */
void POWERLIMIT_setLimpModeOverride(PowerLimit* me) {
    if (me == NULL) return;
    // Implementation pending
}

/**
 * Main power limiting control function
 * Selects control method based on plMode
 */
void PowerLimit_calculateCommand(PowerLimit *me, MotorController *mcm) {
    if (me == NULL || mcm == NULL) return;

    // Update threshold based on target power
    me->plInitializationThreshold = (me->plTargetPower > 15) 
        ? (me->plTargetPower - 15) 
        : 0;

    // Select control method
    switch (me->plMode) {
        case 1:  // Torque equation method
            POWERLIMIT_calculateTorqueCommandTorqueEquation(me, mcm);
            break;
        case 2:  // Power PID method
            POWERLIMIT_calculateTorqueCommandPowerPID(me, mcm);
            break;
        case 3:  // LUT method
            POWERLIMIT_calculateTorqueCommand(me, mcm);
            break;
        case 4:  // Combined method (placeholder)
            break;
        default:
            me->plStatus = FALSE;
            break;
    }
}

/**
 * LUT-based power limiting control
 * Uses lookup table and PID for torque control
 */
void POWERLIMIT_calculateTorqueCommand(PowerLimit *me, MotorController *mcm) {
    if (me == NULL || mcm == NULL) return;

    // External function declarations
    extern sbyte4 MCM_getPower(MotorController *mcm);
    extern sbyte4 MCM_getMotorRPM(MotorController *mcm);
    extern sbyte4 MCM_getDCVoltage(MotorController *mcm);
    extern sbyte4 MCM_getDCCurrent(MotorController *mcm);
    extern sbyte4 MCM_getCommandedTorque(MotorController *mcm);
    extern void   MCM_update_PL_setTorqueCommand(MotorController *mcm, sbyte4 torque);
    extern void   MCM_set_PL_updateStatus(MotorController *mcm, bool status);

    // Check power threshold
    if ((MCM_getPower(mcm) / 1000) >= me->plInitializationThreshold) {
        me->plStatus = TRUE;

        // Get motor state
        sbyte4 motorRPM = MCM_getMotorRPM(mcm);
        sbyte4 mcmVoltage = MCM_getDCVoltage(mcm);
        sbyte4 mcmCurrent = MCM_getDCCurrent(mcm);

        // Calculate no-load voltage (IR compensation)
        sbyte4 noLoadVoltage = (mcmCurrent * 27 / 1000) + mcmVoltage;

        // Get torque setpoint from LUT
        sbyte4 pidSetpoint = POWERLIMIT_retrieveTorqueFromLUT(me, noLoadVoltage, motorRPM);

        // Fallback to torque equation if LUT out of range
        if (pidSetpoint < 0 || pidSetpoint > 231) {
            pidSetpoint = (sbyte4)(me->plTargetPower * 9549 / (motorRPM == 0 ? 1 : motorRPM));
        }

        // Update PID and compute output
        sbyte4 commandedTorque = MCM_getCommandedTorque(mcm);
        PID_updateSetpoint(me->pid, pidSetpoint);
        PID_computeOutput(me->pid, commandedTorque);

        // Apply torque command
        me->plTorqueCommand = (commandedTorque + PID_getOutput(me->pid)) * 10;
        MCM_update_PL_setTorqueCommand(mcm, me->plTorqueCommand);
        MCM_set_PL_updateStatus(mcm, me->plStatus);
    } else {
        // Below threshold - disable limiting
        me->plStatus = FALSE;
        MCM_update_PL_setTorqueCommand(mcm, -1);
        MCM_set_PL_updateStatus(mcm, me->plStatus);
    }
}

/**
 * Torque equation based power limiting
 * Uses mechanical power equation: T = P*9549/rpm
 */
void POWERLIMIT_calculateTorqueCommandTorqueEquation(PowerLimit *me, MotorController *mcm) {
    if (me == NULL || mcm == NULL) return;

    extern sbyte4 MCM_getPower(MotorController *mcm);
    extern sbyte4 MCM_getMotorRPM(MotorController *mcm);
    extern sbyte4 MCM_getCommandedTorque(MotorController *mcm);
    extern void   MCM_update_PL_setTorqueCommand(MotorController *mcm, sbyte4 torque);
    extern void   MCM_set_PL_updateStatus(MotorController *mcm, bool status);

    // Set PID saturation
    PID_setSaturationPoint(me->pid, 8000);

    if ((MCM_getPower(mcm) / 1000) >= me->plInitializationThreshold) {
        me->plStatus = TRUE;

        // Calculate torque setpoint
        sbyte4 motorRPM = MCM_getMotorRPM(mcm);
        if (motorRPM == 0) motorRPM = 1;
        sbyte4 pidSetpoint = (me->plTargetPower * 9549) / motorRPM;

        // Update PID and compute output
        sbyte4 commandedTorque = MCM_getCommandedTorque(mcm);
        PID_updateSetpoint(me->pid, pidSetpoint);
        PID_computeOutput(me->pid, commandedTorque);

        // Apply torque command
        me->plTorqueCommand = (commandedTorque + PID_getOutput(me->pid)) * 10;
        MCM_update_PL_setTorqueCommand(mcm, me->plTorqueCommand);
        MCM_set_PL_updateStatus(mcm, me->plStatus);
    } else {
        me->plStatus = FALSE;
        MCM_update_PL_setTorqueCommand(mcm, -1);
        MCM_set_PL_updateStatus(mcm, me->plStatus);
    }
}

/**
 * Power-based PID control
 * Directly controls power using PID
 */
void POWERLIMIT_calculateTorqueCommandPowerPID(PowerLimit *me, MotorController *mcm) {
    if (me == NULL || mcm == NULL) return;

    extern sbyte4 MCM_getPower(MotorController *mcm);
    extern sbyte4 MCM_getCommandedTorque(MotorController *mcm);
    extern void   MCM_update_PL_setTorqueCommand(MotorController *mcm, sbyte4 torque);
    extern void   MCM_set_PL_updateStatus(MotorController *mcm, bool status);

    // Configure PID
    PID_setSaturationPoint(me->pid, 80000);
    me->plMode = 2;

    if ((MCM_getPower(mcm) / 1000) >= me->plInitializationThreshold) {
        me->plStatus = TRUE;

        // Scale power values for PID
        sbyte4 pidTargetValue = me->plTargetPower * 1000;
        sbyte4 pidCurrentValue = MCM_getPower(mcm) / 10;
        sbyte4 commandedTorque = MCM_getCommandedTorque(mcm);

        // Update PID
        PID_updateSetpoint(me->pid, pidTargetValue);
        PID_computeOutput(me->pid, pidCurrentValue);

        // Calculate new torque command
        if (pidCurrentValue == 0) pidCurrentValue = 1;
        sbyte4 newTorque = commandedTorque + 
            (commandedTorque * PID_getOutput(me->pid) / pidCurrentValue);

        // Apply limits and update
        if (newTorque > 231) newTorque = 231;
        me->plTorqueCommand = newTorque * 10;
        MCM_update_PL_setTorqueCommand(mcm, me->plTorqueCommand);
        MCM_set_PL_updateStatus(mcm, me->plStatus);
    } else {
        me->plStatus = FALSE;
        MCM_update_PL_setTorqueCommand(mcm, -1);
        MCM_set_PL_updateStatus(mcm, me->plStatus);
    }
}

// Status getters
ubyte1 POWERLIMIT_getStatusCodeBlock(PowerLimit* me) { return me ? me->plMode : 0; }
bool   POWERLIMIT_getStatus(PowerLimit* me) { return me ? me->plStatus : FALSE; }
ubyte1 POWERLIMIT_getMode(PowerLimit* me) { return me ? me->plMode : 0; }
sbyte4 POWERLIMIT_getTorqueCommand(PowerLimit* me) { return me ? me->plTorqueCommand : 0; }
ubyte1 POWERLIMIT_getTargetPower(PowerLimit* me) { return me ? me->plTargetPower : 0; }
ubyte1 POWERLIMIT_getInitialisationThreshold(PowerLimit* me) { 
    return me ? me->plInitializationThreshold : 0; 
}

ubyte1 POWERLIMIT_getLUTCorner(PowerLimit* me, ubyte1 corner) {
    if (!me) return 0xFF;
    switch(corner) {
        case 1: return me->vFloorRFloor;
        case 2: return me->vFloorRCeiling;
        case 3: return me->vCeilingRFloor;
        case 4: return me->vCeilingRCeiling;
        default: return 0xFF;
    }
} 