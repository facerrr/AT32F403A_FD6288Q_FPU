#ifndef __HFI_H__
#define __HFI_H__

#include "mc_type.h"


typedef struct
{
    float32_t f32_Ts;
    
    uint8_t u8_Enable;
    uint8_t u8_NSDOut;
    uint8_t u8_NSDFlag;

    uint8_t u8_Dir;

    uint16_t u16_NSDCount;
    float32_t f32_NSDSum1;
    float32_t f32_NSDSum2;

    float32_t f32_IdRef;
    float32_t f32_IdHigh;

    float32_t f32_Uin;
    float32_t f32_InjectMagMax;
    float32_t f32_InjectMagMin;

    float32_t f32_Id;
    float32_t f32_IdLast;
    float32_t f32_IdBase;

    float32_t f32_Iq;
    float32_t f32_IqLast;
    float32_t f32_IqBase;

    float32_t f32_Ialpha;
    float32_t f32_IalphaLast;
    float32_t f32_IalphaHigh;
    float32_t f32_IalphaHighLast;

    float32_t f32_Ibeta;
    float32_t f32_IbetaLast;
    float32_t f32_IbetaHigh;
    float32_t f32_IbetaHighLast;

    float32_t f32_IalphaOut;
    float32_t f32_IbetaOut;

    float32_t f32_ThetaEst;

    float32_t f32_PllKp;
    float32_t f32_PllKi;
    float32_t f32_PllIsum;
    float32_t f32_PllOut;
    
    float32_t f32_CosTheta;
    float32_t f32_SinTheta;

    float32_t f32_RealTimeWr;
    float32_t f32_WrF;
}HFI_t, *pHFI;


typedef struct
{
    float32_t f32_HFIWr;
    float32_t f32_ObsWr;
    float32_t f32_TrgtWr;
    
    float32_t f32_HFIThetaEst;
    float32_t f32_ObsThetaEst;
    float32_t f32_ThetaErr;
    
    float32_t f32_WrOut;
    float32_t f32_ThetaOut;

    float32_t f32_WrMin;
    float32_t f32_WrMax;
    float32_t f32_WrMid;

    float32_t f32_WrLast;

    int16_t i16_CheckCnt;
    uint8_t u8_CurrentMode;

}HFI2Observer_t, *pHFI2Observer;

void HFI_Init(pHFI pHfi, pHFI2Observer pHfi2Obs);
void HFI_Calculate(pHFI pv, QD_t Iqd, AB_t Iab);
void HFI2Observer_Calculate(pHFI2Observer pHfi2Obs, pHFI pHfi);




#endif // __HFI_H__