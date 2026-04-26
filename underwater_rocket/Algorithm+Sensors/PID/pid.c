/*
 * pid.c
 *
 *  Created on: Jan 18, 2026
 *      Author: husey
 */


#include "pid.h"

void PID_Init(PID_Config_t* pid, float p, float i, float d, float dt, float outLim) {
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->dt = dt;
    pid->integralLimit = outLim; // Genelde integral limiti output limitiyle ilişkilidir
    pid->outputLimit = outLim;
    PID_Reset(pid);
}

void PID_Reset(PID_Config_t* pid) {
    pid->integralError = 0.0f;
    pid->lastError = 0.0f;
}

float PID_Calculate(PID_Config_t* pid, float measured_value) {
    float error = pid->setpoint - measured_value;

// Proportional
    float P = pid->Kp * error;

    // Integral
    pid->integralError += error * pid->dt;

    // Anti-windup (Integral Sınırlandırma)
    if (pid->integralError > pid->integralLimit) {
        pid->integralError = pid->integralLimit;
    } else if (pid->integralError < -pid->integralLimit) {
        pid->integralError = -pid->integralLimit;
    }
    float I = pid->Ki * pid->integralError;

    // Derivative
    float derivative = (error - pid->lastError) / pid->dt;
    float D = pid->Kd * derivative;

    pid->lastError = error;

    // Output Clamping
    float output = P + I + D;
    if (output > pid->outputLimit) output = pid->outputLimit;
    else if (output < -pid->outputLimit) output = -pid->outputLimit;

    return output;
}



