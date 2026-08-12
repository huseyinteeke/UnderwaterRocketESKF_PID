#ifndef EKF_EKF_HPP_
#define EKF_EKF_HPP_

#include <stdint.h>
#include <math.h> 
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
        arm_matrix_instance_f32 A = {ROWS, COLS, (float*)this->data};
        arm_matrix_instance_f32 B = {COLS, P,    (float*)other.data};
        arm_matrix_instance_f32 C = {ROWS, P,    (float*)result.data};
        arm_mat_mult_f32(&A, &B, &C);
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

    Matrix<ROWS, COLS> inverse() const {
        static_assert(ROWS == COLS, "Inverse() only implemented for square matrices");
        Matrix<ROWS, COLS> result;
        arm_matrix_instance_f32 A = {ROWS, COLS, (float*)this->data};
        arm_matrix_instance_f32 C = {ROWS, COLS, (float*)result.data};
        arm_status status = arm_mat_inverse_f32(&A, &C);
        if (status != ARM_MATH_SUCCESS) {
            for (int i = 0; i < ROWS; i++) {
                result.data[i][i] = 1.0f;
            }
        }
        return result;
    }
};

template <int STATE_DIM = 2, int MEASURE_DIM = 1>
class SubESKF {
private:
    Matrix<STATE_DIM, 1> x_nom; // [Hiz; Bias]
    Matrix<STATE_DIM, STATE_DIM> P; 
    Matrix<STATE_DIM, STATE_DIM> Q; 
    Matrix<STATE_DIM, STATE_DIM> F; 
    Matrix<STATE_DIM, STATE_DIM> I; 

    Matrix<MEASURE_DIM, STATE_DIM> H; 
    Matrix<MEASURE_DIM, MEASURE_DIM> R; 
    Matrix<MEASURE_DIM, MEASURE_DIM> S; 

    Matrix<STATE_DIM, MEASURE_DIM> K; 
    
    float x_est; // Konum tutmak için
    float hiz_model_eski; // Konum entegrasyonunda kullanılacak
    uint8_t counter;      // Update adımı frekansı için
    uint8_t zupt_counter; // ZUPT kontrolü için

public:
    SubESKF() : x_est(0.0f), hiz_model_eski(0.0f), counter(0), zupt_counter(0) {}

    void Init() {
        for(int i = 0; i < STATE_DIM; i++) {
            x_nom(i, 0) = 0.0f;
            for(int j = 0; j < STATE_DIM; j++) {
                I(i, j) = (i == j) ? 1.0f : 0.0f;
            }
        }
        H(0, 0) = 1.0f; H(0, 1) = 0.0f; 
    }

    void Step(float hiz_model, float ax, float dt) {
        // 1. KONUM ENTEGRASYONU 
        x_est += ((hiz_model_eski + hiz_model) / 2.0f) * dt;
        hiz_model_eski = hiz_model;

        // 2. PREDICT ADIMI
        float bias_est = x_nom(1, 0);
        float ivme_temiz = ax - bias_est;
        
        x_nom(0, 0) += ivme_temiz * dt; // Hız tahmini

        F = I;
        F(0, 1) = -dt; 
        P = F * P * F.transpose() + Q;

        // 3. UPDATE ADIMI (Frekans bölücü ile)
        counter++;
        if (counter >= 2) {
            Matrix<STATE_DIM, 1> x_err;
            x_err(0, 0) = 0.0f;
            x_err(1, 0) = 0.0f;

            int max_iter = 3;
            for (int i = 0; i < max_iter; i++) {
                float z = hiz_model - (x_nom(0, 0) + x_err(0, 0));
                
                S = H * P * H.transpose() + R;
                K = P * H.transpose() * S.inverse();
                
                Matrix<MEASURE_DIM, 1> z_mat;
                z_mat(0, 0) = z;
                
                x_err = x_err + K * z_mat;

                if (fabsf(z) < 1e-4f) {
                    break;
                }
            }

            P = (I - K * H) * P;

            x_nom(0, 0) += x_err(0, 0); 
            x_nom(1, 0) += x_err(1, 0); 
            
            counter = 0; 
        }

        // 4. GÜVENLİ ZUPT (DURAĞANLIK) KONTROLÜ
        bool is_model_zero = (hiz_model == 0.0f);
        bool is_acc_quiet  = (fabsf(ivme_temiz) < 0.04f);

        if (is_model_zero && is_acc_quiet) {
            zupt_counter++;
        } else {
            zupt_counter = 0; 
        }

        if (zupt_counter >= 50) {
            x_nom(0, 0) = 0.0f; 
            zupt_counter = 50;  
        }
    }

    float GetPosition() { return x_est; }
    float GetVelocity() { return x_nom(0, 0); }
    float GetBias()     { return x_nom(1, 0); }

    void setP(const Matrix<STATE_DIM, STATE_DIM>& newP) { P = newP; }
    void setQ(const Matrix<STATE_DIM, STATE_DIM>& newQ) { Q = newQ; }
    void setR(const Matrix<MEASURE_DIM, MEASURE_DIM>& newR) { R = newR; }
    void reset_pos() {x_est = 0.0f; }

};

#endif /* EKF_EKF_HPP_ */