#include "config.h"

MoveFilter_t FluxMoveFilter;
LPFilter_t FluxFirstLpf;

float32_t f32_SpeedBuf[MOVE_FILTER_BUF_MAXSIZE] = {0};
    
void FluxObserver_Init(pFluxObserver pv){
    memset((void*)pv, 0, sizeof(FluxObserver_t));

    pv->f32_Ls = MotorCfg.f32Ls;
    pv->f32_Rs = MotorCfg.f32Rs;
    pv->f32_Flux = MotorCfg.f32LambdaF;
    pv->f32_Ts = 1.0f / MotorCfg.f32CarrierFreqHz;
    pv->f32_Gain = MotorCfg.f32FluxObsGamma;

    pv->f32_PllKp = MotorCfg.f32FluxObsPllKp;
    pv->f32_PllKi = MotorCfg.f32FluxObsPllKi;

    memset(&FluxMoveFilter, 0, sizeof(MoveFilter_t));
    FluxMoveFilter.pBuffer = f32_SpeedBuf;
    MoveFilter_Init(&FluxMoveFilter, 256, 8);

    FluxFirstLpf.f32_LpfK = M_2PI * 20.0f / MotorCfg.f32CarrierFreqHz;
    FluxFirstLpf.f32_Output = 0.0f;
}

void FluxSpeed_Calc(pFluxObserver pv) {
    pv->f32_WrF = MoveFilter_Calc(&FluxMoveFilter);
}

// void FluxObserver_Calc(pFluxObserver pv, AB_t Uab, AB_t Iab){
//     float32_t L_ia = pv->f32_Ls * Iab.f32_Alpha;
//     float32_t L_ib = pv->f32_Ls * Iab.f32_Beta;
//     float32_t R_ia = pv->f32_Rs * Iab.f32_Alpha;
//     float32_t R_ib = pv->f32_Rs * Iab.f32_Beta;


//     float32_t LammbdaF2 = pv->f32_Flux * pv->f32_Flux;
//     float32_t Eta_a = pv->f32_X1 - L_ia;
//     float32_t Eta_b = pv->f32_X2 - L_ib;

//     float32_t Err = LammbdaF2 - (Eta_a * Eta_a + Eta_b * Eta_b);
//     if(Err >= 0.0f){
//         Err = 0.0f;
//     }

//     float32_t Cmp_X1 = pv->f32_Gain * Eta_a * Err;
//     float32_t Cmp_X2 = pv->f32_Gain * Eta_b * Err;

//     pv->f32_X1Dot = Uab.f32_Alpha + Cmp_X1 - R_ia;
//     pv->f32_X2Dot = Uab.f32_Beta + Cmp_X2 - R_ib;

//     pv->f32_X1 += pv->f32_X1Dot * pv->f32_Ts;
//     pv->f32_X2 += pv->f32_X2Dot * pv->f32_Ts;
    
//     float32_t f32_KpDelta, f32_KiDelta;

//     float32_t FluxAlpha = pv->f32_X1 - L_ia;
//     float32_t FluxBeta = pv->f32_X2 - L_ib;

//     float32_t SinEst = sin_f32(pv->f32_ThetaEst);
//     float32_t CosEst = cos_f32(pv->f32_ThetaEst);

//     float32_t obsErr = (FluxBeta * CosEst - FluxAlpha * SinEst) / pv->f32_Flux;
//     f32_KpDelta = pv->f32_PllKp * obsErr;
//     f32_KiDelta = pv->f32_PllKi * obsErr;
//     pv->f32_PllIsum += f32_KiDelta * pv->f32_Ts;
//     pv->f32_PllOut = f32_KpDelta + pv->f32_PllIsum;

//     pv->f32_ThetaEst += pv->f32_PllOut * pv->f32_Ts;
//     while (pv->f32_ThetaEst > M_2PI) pv->f32_ThetaEst -= M_2PI;
//     while (pv->f32_ThetaEst < 0.0f) pv->f32_ThetaEst += M_2PI;

//     pv->f32_RealTimeWr = pv->f32_PllOut; 
//     MoveFilter_Fill(&FluxMoveFilter, pv->f32_RealTimeWr);

// }


void FluxObserver_Calc(pFluxObserver pv, AB_t Uab, AB_t Iab){
    float32_t v_alpha = Uab.f32_Alpha;
    float32_t v_beta = Uab.f32_Beta;
    float32_t i_alpha = Iab.f32_Alpha;
    float32_t i_beta = Iab.f32_Beta;
    float32_t dt = pv->f32_Ts;
    float32_t R = pv->f32_Rs;
    float32_t L = pv->f32_Ls;
    float32_t lambda_nom = pv->f32_Flux;
    float32_t L_ia = L * i_alpha;
    float32_t L_ib = L * i_beta;


    float32_t gamma_half = pv->f32_Gain * 0.5f;

    float32_t err = SQ(pv->f32_LambdaEst) - (SQ(pv->f32_X1) + SQ(pv->f32_X2));
    pv->f32_LambdaEst += 0.2 * gamma_half * pv->f32_LambdaEst * (-err) * dt;

    pv->f32_LambdaEst = CLAMP(pv->f32_LambdaEst, lambda_nom * 0.3f, lambda_nom * 2.5f);
    if (err > 0.0) {
        err = 0.0;
    }

    float x1_dot = v_alpha - R * i_alpha + gamma_half * (pv->f32_X1 - L * i_alpha) * err;
    float x2_dot = v_beta - R * i_beta + gamma_half * (pv->f32_X2 - L * i_beta) * err;

    pv->f32_X1 += x1_dot * dt;
    pv->f32_X2 += x2_dot * dt;

    pv->f32_IalphaLast = i_alpha;
    pv->f32_IbetaLast = i_beta;

    UTILS_NAN_ZERO(pv->f32_X1);
    UTILS_NAN_ZERO(pv->f32_X2);

    float32_t mag = sqrtf(SQ(pv->f32_X1) + SQ(pv->f32_X2));
    if (mag < (lambda_nom * 0.5f)) {
        pv->f32_X1 *= 1.1f;
        pv->f32_X2 *= 1.1f;
    }

    float32_t f32_KpDelta, f32_KiDelta;

    float32_t delta_alpha = pv->f32_X1 - L_ia;
    float32_t delta_beta = pv->f32_X2 - L_ib;

    float32_t sin_est = sin_f32(pv->f32_ThetaEst);
    float32_t cos_es = cos_f32(pv->f32_ThetaEst);

    float32_t obsErr = (delta_beta * cos_es - delta_alpha * sin_est) / pv->f32_LambdaEst;

    f32_KpDelta = 800.0f * obsErr;
    f32_KiDelta = 20000.0f * obsErr;

    pv->f32_PllIsum += f32_KiDelta * pv->f32_Ts;
    pv->f32_PllOut = f32_KpDelta + pv->f32_PllIsum;

    pv->f32_ThetaEst += pv->f32_PllOut * pv->f32_Ts;
    while (pv->f32_ThetaEst > M_PI) pv->f32_ThetaEst -= M_2PI;
    while (pv->f32_ThetaEst < -M_PI) pv->f32_ThetaEst += M_2PI;

    pv->f32_RealTimeWr = pv->f32_PllOut; 
    MoveFilter_Fill(&FluxMoveFilter, pv->f32_RealTimeWr);
}