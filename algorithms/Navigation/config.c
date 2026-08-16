#include "config.h"
#include <math.h>


const float RHO = 1025.0f;
const float CD_CFD = 0.22f;
const float A = 0.0176f;
const float M_TOTAL = 11.76945f; 

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
    
    // 2. Ölü Bölge Kontrolü (Grafikte 1595-1600 civarında başlıyor ama sen 1050 tutabilirsin)
    if (pwm < 1050.0f) {
        return 0.0f;
    }
    
    // Grafikteki 24V (6S) verisine göre 2000 PWM'de max itki = 8800 gram = ~88.0 Newton
    // Ancak 4S pil kullandığımız için voltaj oranını (V_4s / V_24s)^2 buraya yansıtıyoruz.
    // 14.8V / 24.0V = 0.616 -> Karesi = ~0.38
    const float VOLTAGE_SCALE_4S = 0.38f; 
    

    
    float throttle = (pwm - 1000.0f) / 1000.0f; // 0.0 ile 1.0 arası

    float max_thrust_24v_N = 88.0f; 
    float thrust_24v = max_thrust_24v_N * (throttle * throttle);    
    float thrust_4s = thrust_24v * VOLTAGE_SCALE_4S;
    
    return thrust_4s;
}

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