/*
 * pid.h
 *
 *  Created on: Jan 18, 2026
 *      Author: husey
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float dt;
    float setpoint;
    float lastError;
    float integralError;

    float outputLimit;
    float integralLimit;
} PID_Config_t;

void PID_Init(PID_Config_t* pid, float p, float i, float d, float dt, float outLim);
float PID_Calculate(PID_Config_t* pid, float measured_value);
float PID_Calculate_Angle(PID_Config_t* pid, float measured_value);
void PID_Reset(PID_Config_t* pid);


#endif /* INC_PID_H_ */
