/*
 *  Created on: Apr 3, 2026
 *      Author: husey
 */

#include <eskf.hpp>
#include <eskf_c_wrapper.h>

static SubESKF<2, 1> mySubESKF;

void SubESKF_Init(void) {
    // MATLAB kodundaki değerlere uygun P matrisi
    Matrix<2, 2> P;
    P(0, 0) = 0.01f;  P(0, 1) = 0.0f;  
    P(1, 0) = 0.0f;   P(1, 1) = 0.01f; 

    // MATLAB kodundaki değerlere uygun Q matrisi
    Matrix<2, 2> Q;
    Q(0, 0) = 0.01f;  Q(0, 1) = 0.0f;  
    Q(1, 0) = 0.0f;   Q(1, 1) = 1e-4f; 

    // MATLAB kodundaki değerlere uygun R matrisi
    Matrix<1, 1> R;
    R(0, 0) = 1e-4f; 

    mySubESKF.Init();
    mySubESKF.setP(P);
    mySubESKF.setQ(Q);
    mySubESKF.setR(R);
}

void SubESKF_Step(float hiz_model, float ax, float dt) {
    mySubESKF.Step(hiz_model, ax, dt);
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

void EskfResetPosition(void)
{
    mySubESKF.reset_pos();
}
