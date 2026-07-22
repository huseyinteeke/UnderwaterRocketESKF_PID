#include "eskf_c_wrapper.h"
#include "eskf.hpp"

static SubESKF<4, 1> mySubESKF;

void SubESKF_Init(void) {
    mySubESKF.Init();

    // Set initial P matrix
    Matrix<4, 4> P_init;
    P_init(0,0) = 1.0f;
    P_init(1,1) = 1.0f;
    P_init(2,2) = 1.0f;
    P_init(3,3) = 0.1f;
    mySubESKF.setP(P_init);

    // Set Q matrix
    Matrix<4, 4> Q_init;
    Q_init(0,0) = 0.001f;
    Q_init(1,1) = 0.01f;
    Q_init(2,2) = 0.0001f;
    Q_init(3,3) = 0.00001f;
    mySubESKF.setQ(Q_init);

    // Set Measurement Noise Variances
    mySubESKF.setR_IMU(0.1f);
    mySubESKF.setR_ModelDVL(0.5f);
}

void SubESKF_Predict(float current_pwm, float dt) {
    mySubESKF.Predict(current_pwm, dt);
}

void SubESKF_UpdateIMU(float measured_accel, float current_pwm) {
    mySubESKF.UpdateIMU(measured_accel, current_pwm);
}

void SubESKF_UpdateModelDVL(float current_pwm) {
    mySubESKF.UpdateModelDVL(current_pwm);
}

void SubESKF_UpdateZUPT(void) {
    mySubESKF.UpdateZUPT();
}

void SubESKF_UpdateDecelProfile(float v0, float time_since_cutoff) {
    mySubESKF.UpdateDecelProfile(v0, time_since_cutoff);
}

void SubESKF_GetState(float* state_array) {
    mySubESKF.getState(state_array);
}

float SubESKF_GetPosition(void) {
    return mySubESKF.getPosition();
}

float SubESKF_GetVelocity(void) {
    return mySubESKF.getVelocity();
}

void SubESKF_SetThrustCoeffs(float c2, float c1, float c0) {
    mySubESKF.setThrustCoeffs(c2, c1, c0);
}
