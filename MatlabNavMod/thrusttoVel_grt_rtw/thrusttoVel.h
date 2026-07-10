/*
 * thrusttoVel.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "thrusttoVel".
 *
 * Model version              : 1.29
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C source code generated on : Sun Jun 14 19:16:42 2026
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_thrusttoVel_h_
#define RTW_HEADER_thrusttoVel_h_
#ifndef thrusttoVel_COMMON_INCLUDES_
#define thrusttoVel_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#endif                                 /* thrusttoVel_COMMON_INCLUDES_ */

#include "thrusttoVel_types.h"
#include <math.h>
#include "rt_nonfinite.h"
#include <float.h>
#include <string.h>
#include <stddef.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWLogInfo
#define rtmGetRTWLogInfo(rtm)          ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T IMUlmleri;                    /* '<Root>/&#x130;vme ölçümü' */
  real_T UnitDelay;                    /* '<Root>/Unit Delay' */
  real_T Biaskarlm;                    /* '<Root>/Subtract' */
  real_T v_model;              /* '<Root>/&#x130;tki -> H&#x131;z fonksiyonu' */
  real_T konumciktisi;
                /* '<Root>/H&#x131;z - Konum - Hata tahmin algoritmas&#x131;' */
  real_T hizciktisi;
                /* '<Root>/H&#x131;z - Konum - Hata tahmin algoritmas&#x131;' */
  real_T hatatahmin;
                /* '<Root>/H&#x131;z - Konum - Hata tahmin algoritmas&#x131;' */
} B_thrusttoVel_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T SafIMUintegrasyonu_DSTATE;    /* '<Root>/Saf IMU integrasyonu' */
  real_T UnitDelay_DSTATE;             /* '<Root>/Unit Delay' */
  real_T v_model_pred;         /* '<Root>/&#x130;tki -> H&#x131;z fonksiyonu' */
  real_T P[4];  /* '<Root>/H&#x131;z - Konum - Hata tahmin algoritmas&#x131;' */
  real_T v_est; /* '<Root>/H&#x131;z - Konum - Hata tahmin algoritmas&#x131;' */
  real_T bias_est;
                /* '<Root>/H&#x131;z - Konum - Hata tahmin algoritmas&#x131;' */
  real_T x_est; /* '<Root>/H&#x131;z - Konum - Hata tahmin algoritmas&#x131;' */
  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } PWMDeeri_PWORK;                    /* '<Root>/PWM De&#x11F;eri ' */

  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } vmelm_PWORK;                       /* '<Root>/&#x130;vme ölçümü' */

  struct {
    int_T PrevIndex;
  } PWMDeeri_IWORK;                    /* '<Root>/PWM De&#x11F;eri ' */

  struct {
    int_T PrevIndex;
  } vmelm_IWORK;                       /* '<Root>/&#x130;vme ölçümü' */
} DW_thrusttoVel_T;

/* Parameters (default storage) */
struct P_thrusttoVel_T_ {
  real_T SafIMUintegrasyonu_gainval;
                               /* Computed Parameter: SafIMUintegrasyonu_gainval
                                * Referenced by: '<Root>/Saf IMU integrasyonu'
                                */
  real_T SafIMUintegrasyonu_IC;        /* Expression: 0
                                        * Referenced by: '<Root>/Saf IMU integrasyonu'
                                        */
  real_T UnitDelay_InitialCondition;   /* Expression: 0
                                        * Referenced by: '<Root>/Unit Delay'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_thrusttoVel_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;
  RTWSolverInfo solverInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    time_T tFinal;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block parameters (default storage) */
extern P_thrusttoVel_T thrusttoVel_P;

/* Block signals (default storage) */
extern B_thrusttoVel_T thrusttoVel_B;

/* Block states (default storage) */
extern DW_thrusttoVel_T thrusttoVel_DW;

/* Model entry point functions */
extern void thrusttoVel_initialize(void);
extern void thrusttoVel_step(void);
extern void thrusttoVel_terminate(void);

/* Real-time Model object */
extern RT_MODEL_thrusttoVel_T *const thrusttoVel_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'thrusttoVel'
 * '<S1>'   : 'thrusttoVel/H&#x131;z - Konum - Hata tahmin algoritmas&#x131;'
 * '<S2>'   : 'thrusttoVel/PWM -> &#x130;tki'
 * '<S3>'   : 'thrusttoVel/&#x130;tki -> H&#x131;z fonksiyonu'
 */
#endif                                 /* RTW_HEADER_thrusttoVel_h_ */
