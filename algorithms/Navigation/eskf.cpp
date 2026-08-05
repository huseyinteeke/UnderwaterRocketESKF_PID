/*
 *
 *  Created on: Apr 3, 2026
 *      Author: husey
 */

#include <eskf.hpp>
#include <eskf_c_wrapper.h>


static SubESKF<2, 1> mySubESKF;

void SubESKF_Init(void) {
    Matrix<2, 2> P;
    P(0, 0) = 0.1f;  P(0, 1) = 0.0f;  
    P(1, 0) = 0.0f;  P(1, 1) = 0.01f; // Bias belirsizliği

    Matrix<2, 2> Q;
    Q(0, 0) = 1e-3f; Q(0, 1) = 0.0f;  
    Q(1, 0) = 0.0f;  Q(1, 1) = 1e-3f;   

    Matrix<1, 1> R;
    R(0, 0) = 10.0f;

    mySubESKF.Init();
    mySubESKF.setP(P);
    mySubESKF.setQ(Q);
    mySubESKF.setR(R);
}

void SubESKF_Predict(float ax, float dt) {
    mySubESKF.Predict(ax, dt);
}

void SubESKF_UpdateModelVelocity(float hiz_model, float dt) {
    mySubESKF.Update(hiz_model, dt);
}

void SubESKF_GetPosition(float* position) {
    *position = mySubESKF.GetPosition();
}

void SubESKF_GetVelocity(float* velocity) {
    *velocity = mySubESKF.GetVelocity();
}

float SubESKF_GetBias() {
    return mySubESKF.GetBias();
}