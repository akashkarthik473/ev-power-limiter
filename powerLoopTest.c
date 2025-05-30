/**
 * @file powerLoopTest.c
 * @brief Test program for power limiting control system
 */

#include <stdio.h>
#include <math.h>
#include "powerlimit.h"

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

/**
 * Motor controller simulation structure
 * Represents the motor and power electronics state
 */
typedef struct _MotorController {
    sbyte4 motorRPM;         // Current motor speed
    sbyte4 power;            // Current power in watts
    sbyte4 dcVoltage;        // DC bus voltage
    sbyte4 dcCurrent;        // DC bus current
    sbyte4 commandedTorque;  // Current torque command
    bool   plStatus;         // Power limiting status
} MotorController;

// Motor controller interface functions
sbyte4 MCM_getPower(MotorController *mcm) { return mcm->power; }
sbyte4 MCM_getMotorRPM(MotorController *mcm) { return mcm->motorRPM; }
sbyte4 MCM_getDCVoltage(MotorController *mcm) { return mcm->dcVoltage; }
sbyte4 MCM_getDCCurrent(MotorController *mcm) { return mcm->dcCurrent; }
sbyte4 MCM_getCommandedTorque(MotorController *mcm) { return mcm->commandedTorque; }
void MCM_update_PL_setTorqueCommand(MotorController *mcm, sbyte4 torque) { 
    mcm->commandedTorque = torque; 
}
void MCM_set_PL_updateStatus(MotorController *mcm, bool status) { 
    mcm->plStatus = status; 
}

/**
 * Main test program
 * Simulates motor operation with power limiting control
 */
int main(void) {
    // Initialize motor controller
    MotorController mcm = {
        .motorRPM = 3000,        // Initial speed
        .power = 65000,          // Initial power (65kW)
        .dcVoltage = 360,        // DC bus voltage
        .dcCurrent = 0,          // DC bus current
        .commandedTorque = 2310, // Initial torque (231.0 Nm)
        .plStatus = 0            // Power limiting disabled
    };

    // Initialize power limiter
    PowerLimit* pl = POWERLIMIT_new();
    pl->plMode = 1;              // Use torque equation method
    pl->plTargetPower = 80;      // Target power limit (80kW)
    sbyte4 powerThreshold = pl->plTargetPower * 1000;

    printf("Starting power limiting simulation...\n");

    // Main control loop
    for (int i = 0; i < 100; i++) {
        // Run power limiting control
        PowerLimit_calculateCommand(pl, &mcm);

        // Calculate mechanical power
        double torqueNm = mcm.commandedTorque / 10.0;
        double omega = (2.0 * M_PI / 60.0) * mcm.motorRPM;
        double mechPowerW = torqueNm * omega;

        // Update motor state
        mcm.power = (sbyte4)mechPowerW;

        // Print status
        printf("Iteration %2d | RPM=%4d | Torque=%4d (dN-m) -> %.1f Nm\n",
               i, (int)mcm.motorRPM, (int)mcm.commandedTorque, torqueNm);
        printf("           | Power: %.1f W (%.2f kW)\n",
               mechPowerW, mechPowerW/1000.0);
        printf("           | Status: %d\n", (int)mcm.plStatus);
        printf("           | PID: Setpoint=%d, Output=%d (P=%d, I=%d, D=%d)\n",
               PID_getSetpoint(pl->pid),
               PID_getOutput(pl->pid),
               PID_getProportional(pl->pid),
               PID_getIntegral(pl->pid),
               PID_getDerivative(pl->pid));
        printf("\n");

        // Check for power stabilization
        if (mcm.power <= powerThreshold + 1000 && 
            mcm.power >= powerThreshold - 1000) {
            break;
        }
    }

    // Cleanup
    free(pl->pid);
    free(pl);
    return 0;
} 