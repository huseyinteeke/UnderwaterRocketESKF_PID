/*
 * ekf.cpp
 *
 *  Created on: Apr 3, 2026
 *      Author: husey
 */


#include "ekf_c_wrapper.h"
#include "ekf.hpp"


static KalmanFilter<3, 2> myEKF;



void VelocityEKF_Init(void) {

    Matrix<3 , 1> x;
    x(0 , 0) = 0.0f; x(1 , 0) = 0.0f; x(2 , 0) = 0.0f;

    Matrix<2, 3> H;
    H(0, 0) = 1.0f;  H(0, 1) = 0.0f; H(0 , 2) = 0.0f;
    H(1 ,0) = 0.0f;  H(1 ,1) = 0.0f; H(1 , 2) = 1.0f;


    Matrix<3 , 3> Q;
    Q(0, 0) = 0.01f; Q(0, 1) = 0.0f;  Q(0 , 2) = 0.0f;
    Q(1, 0) = 0.0f;  Q(1, 1) = 0.05f; Q(1 , 2) = 0.0f;
    Q(2, 0) = 0.0f;  Q(2, 1) = 0.0f; Q(2 , 2) = 0.01f;


    Matrix<2, 2> R;
    R(0 , 0) = 0.05f; R(0 , 1) = 0.0f;
    R(1 , 0) = 0.0f;  R(1 , 1) = 0.80f;


    Matrix<3, 3> P;
    P(0, 0) = 10.0f;  P(0, 1) = 0.0f; P(0 , 2) = 0.0f;
    P(1, 0) = 0.0f;  P(1, 1) = 10.0f;  P(1 , 2) = 0.0f;
    P(2, 0) = 0.0f;  P(2, 1) = 0.0f; P(2 , 2) = 10.0f;

    myEKF.KalmanInit();

    myEKF.setH(H);
    myEKF.setQ(Q);
    myEKF.setR(R);
    myEKF.setP(P);
}

void VelocityEKF_Process(float* measurements , float dt) {
    static Matrix<3, 3> A;
    A(0, 0) = 1.0f;  A(0, 1) = dt;   A(0 , 2) = (dt*dt)/2;
    A(1, 0) = 0.0f;  A(1, 1) = 1.0f; A(1 , 2) = dt;
    A(2, 0) = 0.0f;  A(2, 1) = 0.0f; A(2 , 2) = 1.0f;
    myEKF.setA(A);
    myEKF.KalmanProcess(measurements);
}

void VelocityEKF_GetState(float* state_array) {
    myEKF.KalmanStateGet(state_array);
}
