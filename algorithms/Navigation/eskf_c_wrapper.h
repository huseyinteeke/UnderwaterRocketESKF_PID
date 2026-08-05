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

void SubESKF_Predict(float current_ax, float dt);
void SubESKF_UpdateModelVelocity(float hiz_model, float dt);
void SubESKF_GetPosition(float* position);
void SubESKF_GetVelocity(float* velocity);
float SubESKF_GetBias();


#ifdef __cplusplus
}














#endif





#endif /* EKF_EKF_C_WRAPPER_H_ */
