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
    if (pwm < 1000.0f) {
        pwm = 1000.0f;
    } else if (pwm > 2000.0f) {
        pwm = 2000.0f;
    }
    
    if (pwm < 1050.0f) {
        return 0.0f;
    }

    const float VOLTAGE_SCALE_4S = 0.38f; 
    

    
    float throttle = (pwm - 1000.0f) / 1000.0f; // 0.0 ile 1.0 arası

    float max_thrust_24v_N = 88.0f; 
    float thrust_24v = max_thrust_24v_N * (throttle * throttle);    
    float thrust_4s = thrust_24v * VOLTAGE_SCALE_4S;
    
    return thrust_4s;
}
void pwm_to_velocity(float pwm, float* velocity, float dt) {
    
    float T_in = pwm_to_thrust(pwm);

    float K_thrust = 0.85f; 
    float T = T_in * K_thrust; 
    
    float v_model_pred = *velocity;
    
    float v_sq = v_model_pred * v_model_pred;
    float v_sign = signf(v_model_pred);

    float Fd_hull = 0.5f * RHO * CD_CFD * A * v_sq * v_sign;
    
    float Cd_lin = 0.06f; 
    float Fd_lin = Cd_lin * v_model_pred;
    
    float Fd_prop = 0.0f;
    if (T <= 0.0f) {
        float Cp_prop = 0.02f; 
        Fd_prop = 0.5f * RHO * Cp_prop * A * v_sq * v_sign;
    }
    
    float Fd_total = Fd_hull + Fd_lin + Fd_prop;
    
    float a_model = (T - Fd_total) / M_TOTAL;
    v_model_pred = v_model_pred + a_model * dt;
  
    if (T <= 0.0f && fabsf(v_model_pred) < 0.02f) {
        v_model_pred = 0.0f;
    }
    
    *velocity = v_model_pred;
}