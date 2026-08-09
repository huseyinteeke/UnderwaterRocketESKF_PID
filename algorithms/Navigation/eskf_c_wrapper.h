#ifndef ESKF_C_WRAPPER_H_
#define ESKF_C_WRAPPER_H_

#ifdef __cplusplus
extern "C" {
#endif

void SubESKF_Init(void);

// Yeni eklenen Step fonksiyonu
void SubESKF_Step(float hiz_model, float ax, float dt);

void SubESKF_GetPosition(float* position);
void SubESKF_GetVelocity(float* velocity);
float SubESKF_GetBias(void);

#ifdef __cplusplus
}
#endif

#endif /* ESKF_C_WRAPPER_H_ */