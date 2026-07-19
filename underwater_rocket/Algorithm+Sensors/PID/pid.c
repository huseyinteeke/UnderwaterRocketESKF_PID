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
    pid->integralLimit = outLim;
    pid->outputLimit = outLim;
    PID_Reset(pid);
}

void PID_Reset(PID_Config_t* pid) {
    pid->integralError = 0.0f;
    pid->lastError = 0.0f;
}

float PID_Calculate(PID_Config_t* pid, float measured_value) {
    float error = pid->setpoint - measured_value;



    float P = pid->Kp * error;

    pid->integralError += error * pid->dt;

    if (pid->integralError > pid->integralLimit) {
        pid->integralError = pid->integralLimit;
    } else if (pid->integralError < -pid->integralLimit) {
        pid->integralError = -pid->integralLimit;
    }
    float I = pid->Ki * pid->integralError;

    float derivative = -(measured_value - pid->lastError) / pid->dt;
    float D = pid->Kd * derivative;

    pid->lastError = measured_value;

    float output = P + I + D;
    if (output > pid->outputLimit) output = pid->outputLimit;
    else if (output < -pid->outputLimit) output = -pid->outputLimit;

    return output;
}

float PID_Calculate_Angle(PID_Config_t* pid, float measured_value) {
    float error = pid->setpoint - measured_value;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    float P = pid->Kp * error;

    pid->integralError += error * pid->dt;

    if (pid->integralError > pid->integralLimit) {
        pid->integralError = pid->integralLimit;
    } else if (pid->integralError < -pid->integralLimit) {
        pid->integralError = -pid->integralLimit;
    }
    float I = pid->Ki * pid->integralError;

    float diff = measured_value - pid->lastError;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    float derivative = -diff / pid->dt;
    float D = pid->Kd * derivative;

    pid->lastError = measured_value;

    float output = P + I + D;
    if (output > pid->outputLimit) output = pid->outputLimit;
    else if (output < -pid->outputLimit) output = -pid->outputLimit;

    return output;
}



