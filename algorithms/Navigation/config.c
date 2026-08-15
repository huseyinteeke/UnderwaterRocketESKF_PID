#include "config.h"
#include <math.h>

// --- SABİT FİZİKSEL PARAMETRELER ---
const float RHO = 1000.0f;
const float CD_CFD = 0.22f;
const float A = 0.0176f;
// m = 11.209 * 1.05 = 11.76945f
const float M_TOTAL = 11.76945f; 

// MATLAB'daki sign() fonksiyonunun C karşılığı
static inline float signf(float val) {
    if (val > 0.0f) return 1.0f;
    if (val < 0.0f) return -1.0f;
    return 0.0f;
}

float pwm_to_thrust(float pwm) {
    // 1. Sinyal Sınırlandırma
    if (pwm < 1000.0f) {
        pwm = 1000.0f;
    } else if (pwm > 2000.0f) {
        pwm = 2000.0f;
    }
    
    // 2. Ölü Bölge Kontrolü
    if (pwm < 1050.0f) {
        return 0.0f;
    }
    
    float x = pwm - 1000.0f; // 0 ile 1000 arası
    float max_thrust_N = 45.0f; 
    
    // Harman Ayarları
    float x_transition = 350.0f; 
    float T_transition = max_thrust_N * 0.15f; 
    float T = 0.0f;
    
    if (x <= x_transition) {
        float ratio = x / x_transition;
        T = T_transition * powf(ratio, 2.8f);
    } else {
        // Üst Bölge (Yüksek Gaz)
        float x_rem = x - x_transition;
        float x_rem_max = 1000.0f - x_transition;
        float ratio_rem = x_rem / x_rem_max;
        
        float gamma_high = 2.55f; 
        T = T_transition + (max_thrust_N - T_transition) * powf(ratio_rem, gamma_high);
    }
    
    return T;
}

// --- İTKİDEN HIZA (FİZİK MODELİ) ---
void pwm_to_velocity(float pwm, float* velocity, float dt) {
    
    float T_in = pwm_to_thrust(pwm);
    float K_thrust = 0.82f; 
    float T = T_in * K_thrust; 
    
    float v_model_pred = *velocity;
    
    float v_sq = v_model_pred * v_model_pred;
    float v_sign = signf(v_model_pred);
    
    // Ana gövde sürtünmesi (Hull Drag)
    float Fd_hull = 0.5f * RHO * CD_CFD * A * v_sq * v_sign;
    
    // Lineer sürtünme (Düşük hızlarda süzülme için)
    float Cd_lin = 0.60f; 
    float Fd_lin = Cd_lin * v_model_pred;
    
    // Pervane frenlemesi (Sadece gaz kapalıyken)
    float Fd_prop = 0.0f;
    if (T <= 0.0f) {
        float Cp_prop = 0.01f; 
        Fd_prop = 0.5f * RHO * Cp_prop * A * v_sq * v_sign;
    }
    
    float Fd_total = Fd_hull + Fd_lin + Fd_prop;
    
    // 3. İVME VE HIZ HESABI
    float a_model = (T - Fd_total) / M_TOTAL;
    v_model_pred = v_model_pred + a_model * dt;
  
    // Çok düşük hızlarda aracı sıfırla (Kalıcı sürüklenmeyi engeller)
    if (T <= 0.0f && fabsf(v_model_pred) < 0.05f) {
        v_model_pred = 0.0f;
    }
    
    *velocity = v_model_pred;
}