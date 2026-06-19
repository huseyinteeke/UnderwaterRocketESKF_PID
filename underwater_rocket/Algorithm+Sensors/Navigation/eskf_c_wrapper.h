/*
 * ekf.h
 *
 *  Created on: Apr 2, 2026
 *      Author: husey
 */

#ifndef EKF_EKF_C_WRAPPER_H_
#define EKF_EKF_C_WRAPPER_H_


#ifdef __cplusplus
extern "C" {
#endif


void SubESKF_Init(void);

/* * TAHMİN (PREDICTION) ADIMI:
 */
void SubESKF_Predict(float current_pwm, float dt);

/* * GÜNCELLEME (UPDATE) ADIMI:
 */
void SubESKF_UpdateIMU(float measured_accel, float current_pwm);
void SubESKF_GetState(float* state_array);


#ifdef __cplusplus
}
#endif





#endif /* EKF_EKF_C_WRAPPER_H_ */
