#ifndef __SMO_H__
#define __SMO_H__

#include "mc_type.h"

typedef struct
{
    float32_t f32_Valpha;
    float32_t f32_Ealpha;
    float32_t f32_Zalpha;
    float32_t f32_Gsmopos;
    float32_t f32_EstIalpha;
    float32_t f32_Fsmopos;
    float32_t f32_Vbeta;
    float32_t f32_Ebeta;
    float32_t f32_Zbeta;
    float32_t f32_EstIbeta;
    float32_t f32_Ialpha;
    float32_t f32_IalphaErr;
    float32_t f32_Kslide;
    float32_t f32_Ibeta;
    float32_t f32_IbetaErr;
    float32_t f32_E0;
    float32_t f32_Kslf;
    float32_t f32_Theta;
    float32_t f32_ThetaEst;
    float32_t f32_Ts;

    float32_t f32_PllKp;
    float32_t f32_PllKi;
    float32_t f32_PllIsum;
    float32_t f32_PllOut;
    
    float32_t f32_CosTheta;
    float32_t f32_SinTheta;

    float32_t f32_RealTimeWr;
    float32_t f32_WrF;
    
    float32_t f32_ErrSQ;

    uint16_t u16_ConvergeCnt;
    uint8_t b_Converged;
}SMObserver_t, *pSMObserver;


void SMObserver_Init(pSMObserver psm);
void SMObserver_Calc(pSMObserver psm, AB_t Uab, AB_t Iab);

#endif // __SMO_H__