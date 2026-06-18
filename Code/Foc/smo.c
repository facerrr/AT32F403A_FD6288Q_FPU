#include "config.h"
#include "math.h"

LPFilter_t SMOFirstLpf;

void SMObserver_Init(pSMObserver pv){
    memset((void*)pv, 0, sizeof(SMObserver_t));
    pv->f32_Ts = 1.0f / MotorCfg.f32CarrierFreqHz;
    float32_t Rs = MotorCfg.f32Rs;
    float32_t Ls = MotorCfg.f32Ls;
    float32_t Fsmopos = expf((-Rs / Ls) * pv->f32_Ts);
    float32_t Gsmopos = (1.0f / Rs) * (1.0f - Fsmopos);
    pv->f32_Fsmopos = Fsmopos;
    pv->f32_Gsmopos = Gsmopos;
    pv->f32_Kslide = 0.13733f;
    pv->f32_E0 = 0.5f;

    pv->f32_PllKp = MotorCfg.f32SMOPllKp;
    pv->f32_PllKi = MotorCfg.f32SMOPllKi;

    pv->f32_Kslf = 0.24f;

    SMOFirstLpf.f32_LpfK = M_2PI * 20.0f / MotorCfg.f32CarrierFreqHz;
    SMOFirstLpf.f32_Output = 0.0f;
    
    pv->f32_ErrSQ = 0.0f;
}

void SMObserver_Calc(pSMObserver pv, AB_t Uab, AB_t Iab){
    pv->f32_Valpha = Uab.f32_Alpha;
    pv->f32_Vbeta = Uab.f32_Beta;

    pv->f32_Ialpha = Iab.f32_Alpha;
    pv->f32_Ibeta = Iab.f32_Beta;

    pv->f32_EstIalpha = (pv->f32_Fsmopos * pv->f32_EstIalpha) + (pv->f32_Gsmopos * (pv->f32_Valpha - pv->f32_Ealpha - pv->f32_Zalpha));
    pv->f32_EstIbeta  = (pv->f32_Fsmopos * pv->f32_EstIbeta)  + (pv->f32_Gsmopos * (pv->f32_Vbeta  - pv->f32_Ebeta  - pv->f32_Zbeta));

    pv->f32_IalphaErr = pv->f32_EstIalpha - pv->f32_Ialpha;
    pv->f32_IbetaErr = pv->f32_EstIbeta - pv->f32_Ibeta;

    float32_t f32_Alpha = CLAMP(pv->f32_IalphaErr, -pv->f32_E0, pv->f32_E0);
    float32_t f32_Beta = CLAMP(pv->f32_IbetaErr, -pv->f32_E0, pv->f32_E0);

    float32_t f32_AbsWr = fabsf(pv->f32_WrF); 
    //    pv->f32_Kslide = 0.5f + (0.005f * f32_AbsWr);
    pv->f32_Kslide = 2.0f + (0.001f * f32_AbsWr);

    if(pv->f32_Kslide > 15.0f) pv->f32_Kslide = 15.0f;

    pv->f32_Zalpha = f32_Alpha * pv->f32_Kslide;
    pv->f32_Zbeta = f32_Beta *  pv->f32_Kslide;

    pv->f32_Ealpha = pv->f32_Ealpha + (pv->f32_Kslf * (pv->f32_Zalpha - pv->f32_Ealpha));
    pv->f32_Ebeta = pv->f32_Ebeta + (pv->f32_Kslf * (pv->f32_Zbeta - pv->f32_Ebeta));
    
    float32_t f32_PllErr = -((pv->f32_Ealpha * pv->f32_CosTheta) + (pv->f32_Ebeta * pv->f32_SinTheta));
    float32_t f32_KpPart = pv->f32_PllKp * f32_PllErr;
    pv->f32_PllIsum += (pv->f32_PllKi * f32_PllErr);
    float32_t f32_WrMidVar = f32_KpPart + pv->f32_PllIsum;
    pv->f32_PllOut = f32_WrMidVar;

    pv->f32_ThetaEst += f32_WrMidVar;

    utils_norm_angle_rad(&pv->f32_ThetaEst);

    pv->f32_CosTheta = cos_f32(pv->f32_ThetaEst);
    pv->f32_SinTheta = sin_f32(pv->f32_ThetaEst);
        
    pv->f32_RealTimeWr = f32_WrMidVar * MotorCfg.f32CarrierFreqHz;
    pv->f32_WrF = Filter_FirstLpf(pv->f32_RealTimeWr, &SMOFirstLpf);
}
