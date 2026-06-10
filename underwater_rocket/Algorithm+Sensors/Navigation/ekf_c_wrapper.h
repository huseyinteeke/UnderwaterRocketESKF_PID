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


void VelocityEKF_Init(void);
void VelocityEKF_Process(float* measurements , float dt);
void VelocityEKF_GetState(float* state_array);





#ifdef __cplusplus
}
#endif





#endif /* EKF_EKF_C_WRAPPER_H_ */
