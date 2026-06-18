#ifndef __FLUX_H__
#define __FLUX_H__

#include "mc_type.h"

typedef struct{
    float32_t f32_X1;
    float32_t f32_X1Dot;
    float32_t f32_X2;
    float32_t f32_X2Dot;

    float32_t f32_Ls;
    float32_t f32_Rs;
    float32_t f32_Flux;
    float32_t f32_Gain;
    float32_t f32_Ts;
    
    float32_t f32_PllKp;
    float32_t f32_PllKi;
    float32_t f32_PllIsum;
    float32_t f32_PllOut;

    float32_t f32_Theta;
    float32_t f32_ThetaEst;
    float32_t f32_Sin;
    float32_t f32_Cos;

    float32_t f32_RealTimeWr;
    float32_t f32_WrF;

    float32_t f32_LambdaEst;
    float32_t f32_IalphaLast;
    float32_t f32_IbetaLast;
}FluxObserver_t, *pFluxObserver;

void FluxObserver_Init(pFluxObserver pv);
void FluxObserver_Calc(pFluxObserver pv, AB_t Uab, AB_t Iab);
void FluxSpeed_Calc(pFluxObserver pv);
void VescFluxObserver_Update(pFluxObserver pv, AB_t Uab, AB_t Iab);

#endif // __FLUX_H__
