#include "config.h"

const float RHO = 1000.0f;
const float CD  = 0.22f;
const float A   = 0.0176f;
const float m   = 12.209f;

void pwm_to_velocity(float pwm , float* velocity , float dt)
{

    float thrust = 0.0f;
    if(pwm < 1000.0f) {
        *velocity =  0.0f; // PWM değeri 1000'den küçükse, hız sıfır kabul edilir
    }else if(pwm > 2000.0f) {
        pwm = 2000.0f; // PWM değeri 2000'den büyükse, maksimum hız kabul edilir
    }
    if (pwm < 1050.0f) {
        *velocity = 0.0f;
    } else {
        float x = pwm - 1000.0f;
        float max_thrust_N = 45.0; 
        float k_thrust = max_thrust_N / (1000.0f * 1000.0f * 1000.0f); 
        thrust = k_thrust * (x * x * x);
    }


    float drag = 0.5 * RHO * CD * A * (*velocity) * (*velocity) * ((*velocity) > 0 ? 1 : -1); 
    float a_model = (thrust - drag) / m;
    *velocity += a_model * dt; // dt = 0.
}
