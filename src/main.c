#include <stdio.h>
#ifndef M_PI
#define M_PI 3.141592653589793
#endif

#include <math.h>
#include "powerlimit.h"

/*
 * UNIT SYSTEM:
 * - Torque: deci-Nm (tenths of Newton-meter, e.g., 2310 = 231.0 Nm)
 * - Power: Watts (W) for internal calculations
 * - RPM: revolutions per minute
 * - Voltage: Volts (V)
 * - Current: Amperes (A)
 */

// MotorController struct
typedef struct _MotorController
{
    sbyte4 motorRPM;        // RPM
    sbyte4 power;           // W (Watts)
    sbyte4 dcVoltage;       // V (Volts)
    sbyte4 dcCurrent;       // A (Amperes)
    sbyte4 commandedTorque; // deci-Nm (tenths of Newton-meter)
    bool   plStatus;        // Power limiting status
} MotorController;

// Stub MCM_ calls
sbyte4 MCM_getPower(MotorController *mcm){ return mcm->power; }
sbyte4 MCM_getMotorRPM(MotorController *mcm){ return mcm->motorRPM; }
sbyte4 MCM_getDCVoltage(MotorController *mcm){ return mcm->dcVoltage; }
sbyte4 MCM_getDCCurrent(MotorController *mcm){ return mcm->dcCurrent; }
sbyte4 MCM_getCommandedTorque(MotorController *mcm){ return mcm->commandedTorque; }
void MCM_update_PL_setTorqueCommand(MotorController *mcm, sbyte4 torque){ mcm->commandedTorque = torque; }
void MCM_set_PL_updateStatus(MotorController *mcm, bool status){ mcm->plStatus = status; }

int main(void)
{
    // Create MotorController instance
    MotorController mcm;
    mcm.motorRPM = 3000;      // Fixed RPM
    mcm.power = 65000;        // Start at 65 kW (in W)
    mcm.dcVoltage = 360;      // V
    mcm.dcCurrent = 0;        // A
    mcm.commandedTorque = 3000; // deci-Nm (300.0 Nm, above setpoint)
    mcm.plStatus = 0;

    // Create PowerLimit instance
    PowerLimit* pl = POWERLIMIT_new();
    pl->plMode = 1;           // TorqueEquation mode
    pl->plTargetPower = 80;   // Limit power to 60 kW (setpoint will be lower)
    sbyte4 powerThreshold = pl->plTargetPower * 1000; // Convert to W

    printf("Starting closed-loop power limit simulation...\n");
    printf("Target Power: %d kW (%d W)\n", pl->plTargetPower, powerThreshold);
    printf("Initial Torque: %d deci-Nm (%.1f Nm)\n", mcm.commandedTorque, mcm.commandedTorque / 10.0);
    printf("Motor RPM: %d\n\n", mcm.motorRPM);

    // Loop through time steps
    for(int i=0; i<50; i++)
    {
        // Step 1: Run power limit algorithm (updates commandedTorque)
        PowerLimit_calculateCommand(pl, &mcm);

        // Step 2: Compute mechanical power from commandedTorque
        double torqueNm = mcm.commandedTorque / 10.0;  // Convert from deci-Nm to Nm
        double omega = (2.0 * M_PI / 60.0) * mcm.motorRPM; // rad/s
        double mechPowerW = torqueNm * omega; // W

        // Step 3: Apply to motor controller (feedback loop)
        mcm.power = (sbyte4) mechPowerW; 

        // Print results
        printf("Iteration %2d | RPM=%4d | Torque=%4d deci-Nm (%.1f Nm)\n",
               i, (int)mcm.motorRPM, (int)mcm.commandedTorque, torqueNm);
        printf("           | Power: %.1f W (%.2f kW) | plStatus=%d\n",
               mechPowerW, mechPowerW/1000.0, (int)mcm.plStatus);

        // Print PID details
        printf("           | PID Setpoint=%d, Output=%d (P=%d, I=%d, D=%d)\n",
            PID_getSetpoint(pl->pid),
            PID_getOutput(pl->pid),
            PID_getProportional(pl->pid),
            PID_getIntegral(pl->pid),
            PID_getDerivative(pl->pid));
        printf("\n");

        // Break if power has stabilized near the threshold
        if (mcm.power <= powerThreshold + 100 && mcm.power >= powerThreshold - 100)
            break;
    }

    // Cleanup
    free(pl->pid);
    free(pl);
    return 0;
}
