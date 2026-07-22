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

const float DRY_MASS = 11.295f;        // kg (Aracın kuru kütlesi)
const float ADDED_MASS = 1.13f;        // kg (Eklenmiş kütle, ~%10 kuru kütle varsayımı)
const float TOTAL_MASS = DRY_MASS + ADDED_MASS; // İvme hesabında kullanılacak toplam kütle

const float RHO = 1000.0f;             // kg/m^3 (Tatlı su yoğunluğu)
const float CD = 0.22f;                // Sürüklenme Katsayısı (Drag Coefficient)
const float AREA = 0.05f;              // m^2 (İleri yön ön kesit alanı)
const float CD_A = CD * AREA;          // Toplam aerodinamik/hidrodinamik çarpan

template <int STATE_DIM = 4, int MEASURE_DIM = 1>
class SubESKF {
private:
    // State Vector
    // x_nom(0,0): position (m)
    // x_nom(1,0): velocity (m/s)
    // x_nom(2,0): accelerometer bias (m/s^2)
    // x_nom(3,0): thrust scale factor (dimensionless)
    Matrix<STATE_DIM, 1> x_nom;
    Matrix<STATE_DIM, 1> dx;
    Matrix<STATE_DIM, STATE_DIM> P;
    Matrix<STATE_DIM, STATE_DIM> Q;
    Matrix<STATE_DIM, STATE_DIM> F;
    Matrix<STATE_DIM, STATE_DIM> I;

    // Measurement Covariances
    float r_imu = 0.1f;
    float r_mdvl = 0.5f;
    float r_zupt = 0.01f;
    float r_decel = 0.2f;

    // Thrust polynomial coefficients (T = c2*u^2 + c1*u + c0)
    float thrust_c2 = 0.00015f;
    float thrust_c1 = 0.02f;
    float thrust_c0 = -0.5f;

    // Ham itki hesabı, Thrust scale factor (k_T) haricinde
    float CalculateRawThrust(float pwm) {
        float p = pwm;
        if (p < 1000.0f) p = 1000.0f;
        if (p > 2000.0f) p = 2000.0f;
        
        float u = (p - 1000.0f) / 1000.0f;
        float raw_thrust = thrust_c2 * u * u + thrust_c1 * u + thrust_c0;
        
        if (raw_thrust < 0.0f) raw_thrust = 0.0f;
        
        return raw_thrust;
    }

    // İtki hesabı (PWM -> Newton), k_T dahil edilmiş hali
    float CalculateThrust(float pwm) {
        float raw_thrust = CalculateRawThrust(pwm);
        return x_nom(3, 0) * raw_thrust; // k_T * T_raw
    }

    // Sürüklenme Kuvveti (Drag)
    float CalculateDrag(float velocity) {
        // Pozitif hız (ileriye) pozitif drag (harekete zıt), negatif hız negatif drag oluşturur.
        // Formül: 0.5 * RHO * CD_A * v * |v|
        return 0.5f * RHO * CD_A * velocity * fabsf(velocity);
    }

    // Sürüklenme Türevi (Drag Derivative)
    float CalculateDragDerivative(float velocity) {
        return RHO * CD_A * fabsf(velocity);
    }

    // Joseph Form Kalmani Güncellemesi
    void GenericUpdate(float z_measured, float h_predicted, 
                       const Matrix<MEASURE_DIM, STATE_DIM>& H, 
                       const Matrix<MEASURE_DIM, MEASURE_DIM>& R_meas) {
        // Innovation
        Matrix<MEASURE_DIM, 1> y;
        y(0, 0) = z_measured - h_predicted;
        
        // Innovation covariance: S = H*P*H^T + R
        Matrix<MEASURE_DIM, MEASURE_DIM> S = H * P * H.transpose() + R_meas;
        
        // Kalman gain: K = P*H^T*S^{-1}
        Matrix<STATE_DIM, MEASURE_DIM> K = P * H.transpose() * S.inverse();
        
        // Error state
        dx = K * y;
        
        // Joseph form covariance update: P = (I-KH)*P*(I-KH)^T + K*R*K^T
        Matrix<STATE_DIM, STATE_DIM> IKH = I - K * H;
        P = IKH * P * IKH.transpose() + K * R_meas * K.transpose();
        
        // Inject and reset
        x_nom = x_nom + dx;
        for(int i = 0; i < STATE_DIM; i++) dx(i, 0) = 0.0f;
    }

public:
    void Init() {
        for(int i = 0; i < STATE_DIM; i++) {
            x_nom(i, 0) = 0.0f;
            dx(i, 0) = 0.0f;
            for(int j = 0; j < STATE_DIM; j++) {
                I(i, j) = (i == j) ? 1.0f : 0.0f;
                P(i, j) = 0.0f;
                Q(i, j) = 0.0f;
                F(i, j) = (i == j) ? 1.0f : 0.0f;
            }
        }
        // Nominal thrust scale factor starts at 1.0
        x_nom(3, 0) = 1.0f;
    }

    void Predict(float current_pwm, float dt) {
        float current_v = x_nom(1, 0);
        
        float thrust = CalculateThrust(current_pwm);
        float drag = CalculateDrag(current_v);
        
        float a_model = (thrust - drag) / TOTAL_MASS;
        
        // State update
        x_nom(0, 0) += current_v * dt + 0.5f * a_model * dt * dt;
        x_nom(1, 0) += a_model * dt;
        // Bias unchanged: x_nom(2,0)
        // k_T unchanged: x_nom(3,0)
        
        // Build F Jacobian
        F = I;
        float dDdv = CalculateDragDerivative(current_v);
        float T_raw = CalculateRawThrust(current_pwm);
        
        F(0, 1) = dt - 0.5f * (dDdv / TOTAL_MASS) * dt * dt; // dp/dv
        F(0, 3) = 0.5f * (T_raw / TOTAL_MASS) * dt * dt;     // dp/dk_T
        
        F(1, 1) = 1.0f - (dDdv / TOTAL_MASS) * dt;           // dv/dv
        F(1, 3) = (T_raw / TOTAL_MASS) * dt;                 // dv/dk_T
        
        // Covariance predict
        P = F * P * F.transpose() + Q;
    }

    void UpdateIMU(float measured_accel, float current_pwm) {
        float thrust = CalculateThrust(current_pwm);
        float drag = CalculateDrag(x_nom(1, 0));
        float a_model = (thrust - drag) / TOTAL_MASS;
        
        float h_x = a_model + x_nom(2, 0); // a_model + bias
        
        float dDdv = CalculateDragDerivative(x_nom(1, 0));
        float T_raw = CalculateRawThrust(current_pwm);
        
        Matrix<MEASURE_DIM, STATE_DIM> H;
        H(0, 0) = 0.0f;
        H(0, 1) = -dDdv / TOTAL_MASS;
        H(0, 2) = 1.0f;
        H(0, 3) = T_raw / TOTAL_MASS;
        
        Matrix<MEASURE_DIM, MEASURE_DIM> R_meas;
        R_meas(0, 0) = r_imu;
        
        GenericUpdate(measured_accel, h_x, H, R_meas);
    }

    void UpdateModelDVL(float current_pwm) {
        float T_raw_clamped = CalculateRawThrust(current_pwm);
        float thrust_total = x_nom(3, 0) * T_raw_clamped;
        
        if (thrust_total <= 0.0f) return; // İtki yoksa güncelleme yok
        
        float v_ss = sqrtf(2.0f * thrust_total / (RHO * CD_A));
        float h_x = x_nom(1, 0);
        
        Matrix<MEASURE_DIM, STATE_DIM> H;
        H(0, 0) = 0.0f; H(0, 1) = 1.0f; H(0, 2) = 0.0f; H(0, 3) = 0.0f;
        
        Matrix<MEASURE_DIM, MEASURE_DIM> R_meas;
        R_meas(0, 0) = r_mdvl;
        
        GenericUpdate(v_ss, h_x, H, R_meas);
    }

    void UpdateZUPT() {
        float h_x = x_nom(1, 0);
        
        Matrix<MEASURE_DIM, STATE_DIM> H;
        H(0, 0) = 0.0f; H(0, 1) = 1.0f; H(0, 2) = 0.0f; H(0, 3) = 0.0f;
        
        Matrix<MEASURE_DIM, MEASURE_DIM> R_meas;
        R_meas(0, 0) = r_zupt;
        
        GenericUpdate(0.0f, h_x, H, R_meas);
    }

    void UpdateDecelProfile(float v0, float time_since_cutoff) {
        float v_predicted = v0 / (1.0f + (RHO * CD_A / (2.0f * TOTAL_MASS)) * fabsf(v0) * time_since_cutoff);
        
        float h_x = x_nom(1, 0);
        
        Matrix<MEASURE_DIM, STATE_DIM> H;
        H(0, 0) = 0.0f; H(0, 1) = 1.0f; H(0, 2) = 0.0f; H(0, 3) = 0.0f;
        
        Matrix<MEASURE_DIM, MEASURE_DIM> R_meas;
        R_meas(0, 0) = r_decel;
        
        GenericUpdate(v_predicted, h_x, H, R_meas);
    }

    // Setters
    void setQ(const Matrix<STATE_DIM, STATE_DIM>& q_mat) { Q = q_mat; }
    void setP(const Matrix<STATE_DIM, STATE_DIM>& p_mat) { P = p_mat; }
    void setR_IMU(float r) { r_imu = r; }
    void setR_ModelDVL(float r) { r_mdvl = r; }
    void setThrustCoeffs(float c2, float c1, float c0) {
        thrust_c2 = c2; thrust_c1 = c1; thrust_c0 = c0;
    }

    // Getters
    void getState(float* state_array) {
        for(int i = 0; i < STATE_DIM; i++) {
            state_array[i] = x_nom(i, 0);
        }
    }

    float getPosition() { return x_nom(0, 0); }
    float getVelocity() { return x_nom(1, 0); }
};

#endif // EKF_EKF_HPP_
