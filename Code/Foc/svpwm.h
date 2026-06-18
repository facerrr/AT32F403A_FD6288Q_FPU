#ifndef __SVPWM_H__
#define __SVPWM_H__

#include "mc_type.h"

void SVPWM_Init(int32_t i32PeakCnt, float32_t f32MaxDuty, pSVPWM pv);
void SVPWM_Calc(float32_t f32Va, float32_t f32Vb, float32_t f32VBus, pSVPWM pv);
int16_t SVPWM_CalcDuty(float32_t f32VBus, pSVPWM pv);

#endif // __SVPWM_H__