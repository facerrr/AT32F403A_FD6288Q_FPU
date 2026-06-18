#ifndef __PI_CALC_H__
#define __PI_CALC_H__

#include "base_types.h"
#include "arm_math.h"

typedef struct{
    float32_t f32_Kp;
    float32_t f32_Ki;
    float32_t f32_IOut;
    float32_t f32_Output;
    float32_t f32_OutMax;
    float32_t f32_OutMin;
    float32_t f32_Errl;
    float32_t f32_AbsErrMax;
}PIControl_t, *pPIControl;

float32_t PID_IncPid(float32_t f32_Ref, float32_t f32_Fbk, pPIControl pv);
float32_t PID_PosPid(float32_t f32_Ref, float32_t f32_Fbk, pPIControl pv);
void PID_Clear(pPIControl pv);

#endif // __PI_CALC_H__