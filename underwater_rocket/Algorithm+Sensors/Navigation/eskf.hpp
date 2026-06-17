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

const float DRY_MASS = 11.295f;        // kg (Aracın kuru kütlesi)
const float ADDED_MASS = 1.13f;        // kg (Eklenmiş kütle, ~%10 kuru kütle varsayımı)
const float TOTAL_MASS = DRY_MASS + ADDED_MASS; // İvme hesabında kullanılacak toplam kütle

const float RHO = 1000.0f;             // kg/m^3 (Tatlı su yoğunluğu)
const float CD = 0.22f;                // Sürüklenme Katsayısı (Drag Coefficient)
const float AREA = 0.05f;              // m^2 (İleri yön ön kesit alanı - BURAYI KENDİ ARACINA GÖRE GÜNCELLE)
const float CD_A = CD * AREA;          // Toplam aerodinamik/hidrodinamik çarpan

// PWM'den İtki (Thrust) elde etmek için kullanılan polinom katsayıları
// T = c2*u^2 + c1*u + c0
const float C2 = 0.00015f;
const float C1 = 0.02f;
const float C0 = -0.5f;


template <int STATE_DIM = 3, int MEASURE_DIM = 1>
class SubESKF {
private:
    Matrix<STATE_DIM, 1> x_nom;
    Matrix<STATE_DIM, 1> dx;


    Matrix<STATE_DIM, STATE_DIM> P; // Hata Kovaryans Matrisi
    Matrix<STATE_DIM, STATE_DIM> Q; // Sistem Gürültü Matrisi
    Matrix<STATE_DIM, STATE_DIM> F; // Durum Geçiş Jacobian'ı
    Matrix<STATE_DIM, STATE_DIM> I; // Birim Matris

    Matrix<MEASURE_DIM, STATE_DIM> H; // Gözlem Jacobian'ı
    Matrix<MEASURE_DIM, MEASURE_DIM> R; // Ölçüm Gürültü Matrisi
    Matrix<MEASURE_DIM, MEASURE_DIM> S; // İnovasyon Kovaryansı

    Matrix<STATE_DIM, MEASURE_DIM> K; // Kalman Kazancı
    Matrix<MEASURE_DIM, 1> y;         // İnovasyon



    // PWM sinyalini Newton cinsinden itki kuvvetine çevirir
    float CalculateThrust(float pwm) {
        if (pwm < 1000.0f) return 0.0f;
        float thrust = (C2 * pwm * pwm) + (C1 * pwm) + C0;
        return (thrust > 0.0f) ? thrust : 0.0f;
    }

    // Mevcut hıza göre Newton cinsinden sürüklenme (Drag) kuvvetini hesaplar
    float CalculateDrag(float velocity) {
        float speed_sq = velocity * velocity;
        float drag = 0.5f * RHO * CD_A * speed_sq;
        // Hız negatifse drag pozitif (ileriye doğru iter), hız pozitifse drag negatif olmalıdır.
        // Büyüklüğü hesaplayıp yönü koruyoruz.
        return (velocity < 0) ? -drag : drag;
    }

public:
    SubESKF() {}

    // Filtreyi başlangıç değerleriyle kurar
    void Init() {
        for(int i = 0 ; i < STATE_DIM ; i++) {
            dx(i, 0) = 0.0f;
            x_nom(i, 0) = 0.0f; // Başlangıçta araç duruyor ve sıfır noktasında kabul ediliyor
            for(int j = 0 ; j < STATE_DIM ; j++) {
                I(i, j) = (i == j) ? 1.0f : 0.0f;
            }
        }
        // Not: P, Q ve R matrisleri set fonksiyonlarıyla dışarıdan verilmelidir.
    }

    // =========================================================
    // ADIM 1: TAHMİN (PREDICTION)
    // Yüksek frekansta (örn. 100Hz) sadece motor komutları ile çalışır
    // =========================================================
    void Predict(float current_pwm, float dt) {

        // 1. Durumları Oku
        float current_v = x_nom(1, 0);

        // 2. Kuvvetleri Hesapla
        float thrust = CalculateThrust(current_pwm);
        float drag = CalculateDrag(current_v);

        // 3. İvmeyi Bul (Newton 2. Yasa - Eklenmiş kütle ile)
        float a_model = (thrust - drag) / TOTAL_MASS;

        // 4. Nominal Durumu İleri Taşı (Kinematik İntegral)
        x_nom(0, 0) += current_v * dt + 0.5f * a_model * dt * dt; // Konum
        x_nom(1, 0) += a_model * dt;                              // Hız
        // x_nom(2, 0) -> Bias rastgele yürüyüş (random walk) yapar, burada sabit bırakıyoruz.

        // 5. F (Jacobian) Matrisini Hesapla
        // d(Drag)/dv = rho * Cd_A * |v|
        float drag_derivative = RHO * CD_A * current_v;
        if (current_v < 0) drag_derivative = -drag_derivative; // Mutlak değer

        F = I;
        F(0, 1) = dt; // Konumun hıza göre değişimi
        // Hızın kendine göre değişimi: 1 - (sürtünme_türevi / toplam_kütle) * dt
        F(1, 1) = 1.0f - (drag_derivative / TOTAL_MASS) * dt;

        // 6. Kovaryans İletimi: Hata zamanla büyür (P = F*P*F^T + Q)
        P = F * P * F.transpose() + Q;
    }

    // =========================================================
    // ADIM 2: GÜNCELLEME (UPDATE)
    // IMU'dan yeni veri geldiğinde (örn. 200Hz) çalışır
    // =========================================================
    void UpdateIMU(float measured_accel, float current_pwm) {

        // 1. Dinamik modelin ne ölçmeyi BEKLEDİĞİNİ hesapla
        float current_v = x_nom(1, 0);
        float thrust = CalculateThrust(current_pwm);
        float drag = CalculateDrag(current_v);
        float a_model = (thrust - drag) / TOTAL_MASS;

        // Beklenen Ölçüm = Modelin Gerçek İvmesi + Tahmin Edilen Sensör Sapması (Bias)
        float h_x = a_model + x_nom(2, 0);

        // 2. İnovasyon (Gerçekten ölçülen - Beklenen)
        Matrix<MEASURE_DIM, 1> z;
        z(0, 0) = measured_accel;

        Matrix<MEASURE_DIM, 1> h_x_mat;
        h_x_mat(0, 0) = h_x;

        y = z - h_x_mat;

        // 3. H (Jacobian) Matrisini Hesapla
        // İvme ölçümünün durumlara (konum, hız, bias) göre türevleri
        float drag_derivative = RHO * CD_A * current_v;
        if (current_v < 0) drag_derivative = -drag_derivative;

        H(0, 0) = 0.0f; // da/dp (İvme konuma bağlı değildir)
        H(0, 1) = -(drag_derivative / TOTAL_MASS); // da/dv (Hız arttıkça sürtünme artar, ivme düşer)
        H(0, 2) = 1.0f; // da/dbias (Bias ölçümü 1'e 1 etkiler)

        // 4. Standart Kalman Matematiği (S, K ve yeni P hesaplaması)
        S = H * P * H.transpose() + R;
        K = P * H.transpose() * S.inverse();

        dx = K * y;               // Hata durumunu (dx) bul
        P = (I - K * H) * P;      // Kovaryansı güncelle (Belirsizlik azalır)

        // 5. Bulunan hatayı sisteme entegre et
        InjectAndReset();
    }

    // =========================================================
    // ADIM 3: ENJEKSİYON VE SIFIRLAMA (INJECTION & RESET)
    // =========================================================
    void InjectAndReset() {
        // Hesaplanan hata sapmalarını, nominal gerçek duruma ekleyerek düzelt
        x_nom = x_nom + dx;

        // Hata durumunu bir sonraki tahmin adımı için sıfırla
        for(int i = 0; i < STATE_DIM; i++) {
            dx(i, 0) = 0.0f;
        }
    }

    // --- Getter ve Setter Metodları (Wrapper İçin İdeal) ---
    void setQ(const Matrix<STATE_DIM, STATE_DIM>& sys_Q) { Q = sys_Q; }
    void setR(const Matrix<MEASURE_DIM, MEASURE_DIM>& sys_R) { R = sys_R; }
    void setP(const Matrix<STATE_DIM, STATE_DIM>& sys_P) { P = sys_P; }

    // Filtrenin nihai konum, hız ve bias tahminlerini dışarı aktar
    void getState(float* state_array) {
        for(int i = 0; i < STATE_DIM; i++) {
            state_array[i] = x_nom(i, 0);
        }
    }
};

#endif /* SUB_ESKF_HPP_ */

