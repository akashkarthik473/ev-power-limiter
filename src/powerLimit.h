/*****************************************************************************
 * powerLimit.h - Power Limiting using a PID controller & LUT
 ****************************************************************************/
#ifndef _POWERLIMIT_H
#define _POWERLIMIT_H

#include "pid.h"

/**
 * Forward-declare the MotorController type so that we do not need
 * a separate header. We'll define its details in the test file.
 */
typedef struct _MotorController MotorController;

/**
 * The PowerLimit object.
 */
typedef struct _PowerLimit {
    PID    *pid;

    // Status and mode
    bool    plStatus;                   // Are we actively limiting power?
    ubyte1  plMode;                     // 1,2,3,4 => which limiting method
    ubyte1  plTargetPower;              // in kW
    ubyte1  plInitializationThreshold;  // if (currentPower > threshold) => limit

    // The final torque command (in deci-Nm for your system).
    sbyte4  plTorqueCommand;

    // LUT corners for debugging
    ubyte1 vFloorRFloor;
    ubyte1 vFloorRCeiling;
    ubyte1 vCeilingRFloor;
    ubyte1 vCeilingRCeiling;

} PowerLimit;

/** Constructor **/
PowerLimit* POWERLIMIT_new();

/** Set limp mode if needed (placeholder example) **/
void POWERLIMIT_setLimpModeOverride(PowerLimit* me);

/**
 * @brief High-level entry point that decides which method to call 
 *        based on plMode (1: Tq eqn, 2: Power PID, 3: LUT, 4: combination).
 */
void PowerLimit_calculateCommand(PowerLimit *me, MotorController *mcm);

/**
 * @brief LUT-based method
 */
void POWERLIMIT_calculateLUTEquation(PowerLimit *me, MotorController *mcm);

/**
 * @brief Tq = kW -> mechanical eqn method
 */
void POWERLIMIT_calculateTorqueEquation(PowerLimit *me, MotorController *mcm);

/**
 * @brief Entirely power-based PID
 */
void POWERLIMIT_calculatePowerEquation(PowerLimit *me, MotorController *mcm);

/**
 * @brief Retrieve torque from LUT given noLoadVoltage and rpm
 */
sbyte4 POWERLIMIT_retrieveTorqueFromLUT(PowerLimit* me, ubyte4 noLoadVoltage, ubyte4 rpm);

/**
 * @brief The actual lookup in the array
 */
ubyte1 POWERLIMIT_getTorqueFromArray(ubyte4 noLoadVoltage, ubyte4 rpm);

// ---------- Basic Getters -------------
ubyte1 POWERLIMIT_getStatusCodeBlock(PowerLimit* me);
bool   POWERLIMIT_getStatus(PowerLimit* me);
ubyte1 POWERLIMIT_getMode(PowerLimit* me);
sbyte4 POWERLIMIT_getTorqueCommand(PowerLimit* me);
ubyte1 POWERLIMIT_getTargetPower(PowerLimit* me);
ubyte1 POWERLIMIT_getInitialisationThreshold(PowerLimit* me);
ubyte1 POWERLIMIT_getLUTCorner(PowerLimit* me, ubyte1 corner);

#endif //_POWERLIMIT_H
