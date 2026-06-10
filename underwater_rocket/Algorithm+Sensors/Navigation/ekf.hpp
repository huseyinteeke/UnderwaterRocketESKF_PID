#ifndef EKF_EKF_HPP_
#define EKF_EKF_HPP_

#include <stdint.h>

#include "stm32f4xx_hal.h"
#ifndef __FPU_PRESENT
#define __FPU_PRESENT 1U
#endif

#ifndef ARM_MATH_CM4
#define ARM_MATH_CM4
#endif

#include "arm_math.h"



template <int ROWS, int COLS>
class Matrix {
public:
    float data[ROWS][COLS] = {};

    float& operator()(int r, int c)       { return data[r][c]; }
    float  operator()(int r, int c) const { return data[r][c]; }

    template <int P>
    Matrix<ROWS, P> operator*(const Matrix<COLS, P>& other) const {
        Matrix<ROWS, P> result;

        arm_matrix_instance_f32 A = {ROWS , COLS , (float*)this->data};
        arm_matrix_instance_f32 B = {COLS , P , (float*)other.data};
        arm_matrix_instance_f32 C = {ROWS, P,    (float*)result.data};
        arm_mat_mult_f32(&A , &B , &C);
        return result;
    }

    Matrix<ROWS, COLS> operator+(const Matrix<ROWS, COLS>& other) const {
        Matrix<ROWS, COLS> result;

        arm_matrix_instance_f32 A = {ROWS, COLS, (float*)this->data};
        arm_matrix_instance_f32 B = {ROWS, COLS, (float*)other.data};
        arm_matrix_instance_f32 C = {ROWS, COLS, (float*)result.data};

        arm_mat_add_f32(&A, &B, &C);
        return result;
    }

    Matrix<ROWS, COLS> operator-(const Matrix<ROWS, COLS>& other) const {
        Matrix<ROWS, COLS> result;

        arm_matrix_instance_f32 A = {ROWS, COLS, (float*)this->data};
        arm_matrix_instance_f32 B = {ROWS, COLS, (float*)other.data};
        arm_matrix_instance_f32 C = {ROWS, COLS, (float*)result.data};

        arm_mat_sub_f32(&A, &B, &C);
        return result;
    }

    Matrix<COLS, ROWS> transpose() const {
        Matrix<COLS, ROWS> result;
        arm_matrix_instance_f32 A = {ROWS, COLS, (float*)this->data};
        arm_matrix_instance_f32 C = {COLS, ROWS, (float*)result.data};

        arm_mat_trans_f32(&A, &C);
        return result;
    }

    // Matrix Inverse  (Only for 2x2)
    Matrix<ROWS , COLS> inverse() const {
      static_assert(ROWS == COLS, "Inverse() only implemented for square matrices");

      Matrix<ROWS, COLS> result;
      arm_matrix_instance_f32 A = {ROWS, COLS, (float*)this->data};
      arm_matrix_instance_f32 C = {ROWS, COLS, (float*)result.data};

      arm_status status = arm_mat_inverse_f32(&A, &C);

      // If determinant close to 0 -> filter converges
      if (status != ARM_MATH_SUCCESS) {
          for (int i = 0; i < ROWS; i++) {
              result.data[i][i] = 1.0f;
          }
      }
      return result;
    }
};

template <int STATE_DIM, int MEASURE_DIM>
class KalmanFilter {
private:
    Matrix<STATE_DIM, 1>             x;
    Matrix<STATE_DIM, STATE_DIM>     P, Q, A;
    Matrix<MEASURE_DIM , STATE_DIM>   H;
    Matrix<MEASURE_DIM , MEASURE_DIM> R, S;
    Matrix<STATE_DIM, MEASURE_DIM>   K;
    Matrix<MEASURE_DIM, 1>           y;
    Matrix<STATE_DIM , STATE_DIM>    I;

    void KalmanPredict() {
      x = A * x;                        //State estimation
      P = A * P * A.transpose() + Q;    //Error Covariance Estimation
    }

    void KalmanUpdate(float* measurements) {
      Matrix<MEASURE_DIM , 1> z;
      float* meas_ptr = measurements;
      for(int i = 0 ; i < MEASURE_DIM ; i++) z(i , 0) = *(meas_ptr + i);

      y = z - (H * x); //Innovation vector
      S = H * P * H.transpose() + R;
      K = P * H.transpose() * S.inverse();
      x = x + (K * y);
      P = (I - K * H) * P;
    }

public:
    KalmanFilter() {}

    void KalmanInit() {
      for(int i = 0 ; i < STATE_DIM ; i++){
        for(int j = 0 ; j < STATE_DIM ; j++){
          if(i == j) I(i , j) = 1;
        }
      }
    }

    void KalmanProcess(float* measurements) {
      KalmanPredict();
      KalmanUpdate(measurements);
    }

    void KalmanStateGet(float* state_array) {
        for(int i = 0; i < STATE_DIM; i++) {
            state_array[i] = x(i, 0);
        }
    }

    void setx(const Matrix<STATE_DIM , 1>& sys_x){x = sys_x;}
    void setA(const Matrix<STATE_DIM, STATE_DIM>& sys_A) { A = sys_A; }
    void setH(const Matrix<MEASURE_DIM, STATE_DIM>& sys_H) { H = sys_H; }
    void setQ(const Matrix<STATE_DIM, STATE_DIM>& sys_Q) { Q = sys_Q; }
    void setR(const Matrix<MEASURE_DIM, MEASURE_DIM>& sys_R) { R = sys_R; }
    void setP(const Matrix<STATE_DIM, STATE_DIM>& sys_P) { P = sys_P; }
};

#endif /*EKF_EKF_HPP_*/
