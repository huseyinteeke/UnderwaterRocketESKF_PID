/*
 *
 *  Created on: Apr 3, 2026
 *      Author: husey
 */


#include <eskf.hpp>
#include <eskf_c_wrapper.h>



static SubESKF<3, 1> mySubESKF;

void SubESKF_Init(void) {
    // 1. P Matrisi: Başlangıç Belirsizliği (Kovaryans)
    // Sistemin başlangıç noktasına ne kadar güvendiğimizi ifade eder.
    Matrix<3, 3> P;
    P(0, 0) = 1.0f;  P(0, 1) = 0.0f;  P(0, 2) = 0.0f;
    P(1, 0) = 0.0f;  P(1, 1) = 1.0f;  P(1, 2) = 0.0f;
    P(2, 0) = 0.0f;  P(2, 1) = 0.0f;  P(2, 2) = 1.0f; // Bias belirsizliği

    // 2. Q Matrisi: Sistem / Süreç Gürültüsü
    Matrix<3, 3> Q;
    Q(0, 0) = 0.01f; Q(0, 1) = 0.0f;  Q(0, 2) = 0.0f;  // Konum gürültüsü
    Q(1, 0) = 0.0f;  Q(1, 1) = 0.05f; Q(1, 2) = 0.0f;  // Hız gürültüsü
    Q(2, 0) = 0.0f;  Q(2, 1) = 0.0f;  Q(2, 2) = 0.001f;

    Matrix<1, 1> R;
    R(0, 0) = 0.1f;
    mySubESKF.Init();
    mySubESKF.setP(P);
    mySubESKF.setQ(Q);
    mySubESKF.setR(R);
}


void SubESKF_Predict(float current_pwm, float dt) {
    mySubESKF.Predict(current_pwm, dt);
}

void SubESKF_UpdateIMU(float measured_accel, float current_pwm) {
    mySubESKF.UpdateIMU(measured_accel, current_pwm);
}

void SubESKF_GetState(float* state_array) {
    mySubESKF.getState(state_array);
}
