/*
 * pid.h
 *
 *  Created on: Jan 18, 2026
 *      Author: husey
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct {
    // --- Ayarlanabilir Katsayılar (Tuning) ---
    float Kp;
    float Ki;
    float Kd;
    float dt;
    // --- Durum Değişkenleri (State Variables) ---
    float setpoint;      // Hedef (Örn: 0 derece)
    float lastError;     // Bir önceki hata (D terimi için şart) - SENDE VAR
    float integralError; // <--- EN KRİTİK EKSİK BU! (I terimi için şart)

    // --- Güvenlik ve Limitler (Safety) ---
    float outputLimit;   // Çıkışın alabileceği max değer (Örn: Servo için 90 derece)
    float integralLimit; // Anti-Windup limiti (I terimi sonsuza gitmesin diye)

} PID_Config_t;

void PID_Init(PID_Config_t* pid, float p, float i, float d, float dt, float outLim);
float PID_Calculate(PID_Config_t* pid, float measured_value);
void PID_Reset(PID_Config_t* pid);


#endif /* INC_PID_H_ */
