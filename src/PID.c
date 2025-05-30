/*****************************************************************************
 * pid.c - Proportional-Integral-Derivative (PID) controller
 *****************************************************************************/

#include "pid.h"
#include <stdlib.h>

/**
 * @brief Create a PID object on the heap with specified gains and saturation
 */
PID* PID_new(sbyte1 Kp, sbyte1 Ki, sbyte1 Kd, sbyte4 saturationValue) {
    PID* pid = (PID*)malloc(sizeof(PID));
    if (pid == NULL) return NULL;  // Handle malloc failure

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->setpoint      = 0; 
    pid->previousError = 0;
    pid->totalError    = 0;

    // 100 means 100 Hz => loop dt=0.01 s if you want to interpret it that way
    pid->dH            = 100;  
    pid->output        = 0;
    pid->proportional  = 0;
    pid->integral      = 0;
    pid->derivative    = 0;
    pid->saturationValue = saturationValue;
    pid->antiWindupFlag = FALSE;
    return pid;
}

/** SETTER FUNCTIONS **/
void PID_updateGainValues(PID* pid, sbyte1 Kp, sbyte1 Ki, sbyte1 Kd){
    if (pid == NULL) return;  // Handle NULL pointer
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void PID_setTotalError(PID* pid, sbyte4 error){
    if (pid == NULL) return;  // Handle NULL pointer
    pid->totalError = error;
}

void PID_setSaturationPoint(PID *pid, sbyte4 saturationValue){
    if (pid == NULL) return;  // Handle NULL pointer
    pid->saturationValue = saturationValue;
}

void PID_updateSetpoint(PID *pid, sbyte4 setpoint) {
    if (pid == NULL) return;  // Handle NULL pointer

    // If saturationValue > 0, clamp the setpoint
    if (pid->saturationValue > 0 && setpoint > pid->saturationValue)
        pid->setpoint = pid->saturationValue;
    else
        pid->setpoint = setpoint;
}

/** COMPUTATIONS **/
void PID_computeOutput(PID *pid, sbyte4 sensorValue) {
    if (pid == NULL) return;  // Handle NULL pointer

    sbyte4 currentError = (sbyte4)(pid->setpoint - sensorValue);

    // Proportional term
    pid->proportional   = (sbyte4)(pid->Kp * currentError / 10);

    // Integral term
    sbyte4 newTotalError = pid->totalError + currentError;
    pid->integral = (sbyte4)((pid->Ki * newTotalError) / pid->dH / 10);

    // Derivative term (with low-pass filtering)
    sbyte4 diff = (sbyte4)(currentError - pid->previousError);
    double alpha = 0.1;  // Low-pass filter coefficient
    pid->derivative = (sbyte4)(alpha * (pid->Kd * diff * pid->dH) / 10 + (1 - alpha) * pid->derivative);

    pid->previousError = currentError;
    pid->totalError    = newTotalError;

    // Start with P
    pid->output = (pid->proportional + pid->integral + pid->derivative);

    // Apply saturation and anti-windup
    if (pid->saturationValue > 0) {
        if (pid->output > pid->saturationValue) {
            pid->output = pid->saturationValue;
            pid->antiWindupFlag = TRUE;
        } else if (pid->output > pid->saturationValue) {
            pid->output = pid->saturationValue;
            pid->antiWindupFlag = TRUE;
        } else {
            pid->antiWindupFlag = FALSE;
        }
    }
}

/** GETTER FUNCTIONS **/
sbyte1 PID_getKp(PID *pid)                 { return pid ? pid->Kp : 0; }
sbyte1 PID_getKi(PID *pid)                 { return pid ? pid->Ki : 0; }
sbyte1 PID_getKd(PID *pid)                 { return pid ? pid->Kd : 0; }
sbyte4 PID_getSetpoint(PID *pid)           { return pid ? pid->setpoint : 0; }
sbyte4 PID_getPreviousError(PID *pid)      { return pid ? pid->previousError : 0; }
sbyte4 PID_getTotalError(PID* pid)         { return pid ? pid->totalError : 0; }
sbyte4 PID_getOutput(PID *pid)             { return pid ? pid->output : 0; }
sbyte4 PID_getProportional(PID *pid)       { return pid ? pid->proportional : 0; }
sbyte4 PID_getIntegral(PID *pid)           { return pid ? pid->integral : 0; }
sbyte4 PID_getDerivative(PID *pid)         { return pid ? pid->derivative : 0; }
sbyte4 PID_getSaturationValue(PID *pid)    { return pid ? pid->saturationValue : 0; }
bool   PID_getAntiWindupFlag(PID *pid)     { return pid ? pid->antiWindupFlag : FALSE; }
