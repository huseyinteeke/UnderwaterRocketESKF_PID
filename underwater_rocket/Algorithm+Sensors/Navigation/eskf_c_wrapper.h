#ifndef ESKF_C_WRAPPER_H
#define ESKF_C_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

void SubESKF_Init(void);
void SubESKF_Predict(float current_pwm, float dt);
void SubESKF_UpdateIMU(float measured_accel, float current_pwm);
void SubESKF_UpdateModelDVL(float current_pwm);
void SubESKF_UpdateZUPT(void);
void SubESKF_UpdateDecelProfile(float v0, float time_since_cutoff);
void SubESKF_GetState(float* state_array);
float SubESKF_GetPosition(void);
float SubESKF_GetVelocity(void);
void SubESKF_SetThrustCoeffs(float c2, float c1, float c0);

#ifdef __cplusplus
}
#endif

#endif // ESKF_C_WRAPPER_H
