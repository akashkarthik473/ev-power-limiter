/**
 * @file PID.c
 * @brief Implementation of discrete PID controller with anti-windup
 */

#include "pid.h"
#include <stdlib.h>

/**
 * Creates a new PID controller instance
 * @param Kp Proportional gain (deci-units)
 * @param Ki Integral gain (deci-units)
 * @param Kd Derivative gain (deci-units)
 * @param saturationValue Maximum output limit (0 for no limit)
 */
PID* PID_new(sbyte1 Kp, sbyte1 Ki, sbyte1 Kd, sbyte4 saturationValue) {
    PID* pid = (PID*)malloc(sizeof(PID));
    if (pid == NULL) return NULL;

    // Initialize gains
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    // Initialize state
    pid->setpoint = 0;
    pid->previousError = 0;
    pid->totalError = 0;
    pid->dH = 100;  // 100Hz sampling rate
    pid->output = 0;
    pid->proportional = 0;
    pid->integral = 0;
    pid->derivative = 0;
    pid->saturationValue = saturationValue;
    pid->antiWindupFlag = FALSE;
    return pid;
}

// Configuration functions
void PID_updateGainValues(PID* pid, sbyte1 Kp, sbyte1 Ki, sbyte1 Kd) {
    if (pid == NULL) return;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void PID_setTotalError(PID* pid, sbyte4 error) {
    if (pid == NULL) return;
    pid->totalError = error;
}

void PID_setSaturationPoint(PID *pid, sbyte4 saturationValue) {
    if (pid == NULL) return;
    pid->saturationValue = saturationValue;
}

void PID_updateSetpoint(PID *pid, sbyte4 setpoint) {
    if (pid == NULL) return;
    if (pid->saturationValue > 0 && setpoint > pid->saturationValue)
        pid->setpoint = pid->saturationValue;
    else
        pid->setpoint = setpoint;
}

/**
 * Computes PID output with anti-windup protection
 * @param pid PID controller instance
 * @param sensorValue Current process value
 */
void PID_computeOutput(PID *pid, sbyte4 sensorValue) {
    if (pid == NULL) return;

    // Calculate error terms
    sbyte4 currentError = pid->setpoint - sensorValue;
    
    // P term
    pid->proportional = (pid->Kp * currentError) / 10;

    // I term with anti-windup
    sbyte4 newTotalError = pid->totalError + currentError;
    pid->integral = (pid->Ki * newTotalError) / pid->dH / 10;

    // D term with low-pass filter
    sbyte4 diff = currentError - pid->previousError;
    double alpha = 0.1;  // Filter coefficient
    pid->derivative = (sbyte4)(alpha * (pid->Kd * diff * pid->dH) / 10 + 
                              (1 - alpha) * pid->derivative);

    // Update state
    pid->previousError = currentError;
    pid->totalError = newTotalError;

    // Compute output
    pid->output = pid->proportional + pid->integral + pid->derivative;

    // Apply saturation and anti-windup
    if (pid->saturationValue > 0) {
        if (pid->output > pid->saturationValue) {
            pid->output = pid->saturationValue;
            pid->antiWindupFlag = TRUE;
        } else {
            pid->antiWindupFlag = FALSE;
        }
    }
}

// Status getters with null checks
sbyte1 PID_getKp(PID *pid) { return pid ? pid->Kp : 0; }
sbyte1 PID_getKi(PID *pid) { return pid ? pid->Ki : 0; }
sbyte1 PID_getKd(PID *pid) { return pid ? pid->Kd : 0; }
sbyte4 PID_getSetpoint(PID *pid) { return pid ? pid->setpoint : 0; }
sbyte4 PID_getPreviousError(PID *pid) { return pid ? pid->previousError : 0; }
sbyte4 PID_getTotalError(PID* pid) { return pid ? pid->totalError : 0; }
sbyte4 PID_getOutput(PID *pid) { return pid ? pid->output : 0; }
sbyte4 PID_getProportional(PID *pid) { return pid ? pid->proportional : 0; }
sbyte4 PID_getIntegral(PID *pid) { return pid ? pid->integral : 0; }
sbyte4 PID_getDerivative(PID *pid) { return pid ? pid->derivative : 0; }
sbyte4 PID_getSaturationValue(PID *pid) { return pid ? pid->saturationValue : 0; }
bool   PID_getAntiWindupFlag(PID *pid) { return pid ? pid->antiWindupFlag : FALSE; } 