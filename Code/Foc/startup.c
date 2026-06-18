#include "config.h"

void Startup_Init(Startup_t *pv){
    memset((void*)pv, 0, sizeof(Startup_t));

    float32_t f32Temp;

    pv->status = Startup_Initialize;
    pv->u8Status_StateDetect = 0;
    pv->u8Status_Align = 0;
    pv->i32TimerCnt = 0;

    pv->f32_Ts = 1.0f / MotorCfg.f32CarrierFreqHz;

    pv->f32_MaxStartIq = MotorCfg.f32MaxStartIs;
    pv->f32_MaxNormaIq = MotorCfg.f32MaxNormaIs;

    pv->f32_LowSpdLower = 0.60f * (float32_t)MotorCfg.i32PolePairs * (float32_t)MotorCfg.i32MinSpeedRpm / 60.0f;
    pv->f32_LowSpdUpper = 0.90f * (float32_t)MotorCfg.i32PolePairs * (float32_t)MotorCfg.i32MinSpeedRpm / 60.0f;

    f32Temp = MotorCfg.f32ObsStableTimeMs * MotorCfg.f32CarrierFreqHz / 1000.0f;
    pv->i32ObsStableTime = (int32_t)f32Temp;
    pv->f32_ObserableWr = MotorCfg.f32MinObserableWr;

    if (pv->f32_ObserableWr >= pv->f32_LowSpdLower){
        pv->f32_ObserableWr = pv->f32_LowSpdLower;
    }

    pv->f32_ForceIq = 0.0f;
    pv->f32_ForceWr = 0.0f;

    f32Temp = MotorCfg.f32_1stAlignTimeMs * MotorCfg.f32CarrierFreqHz / 1000.0f;
    pv->i32Align1stTime = (int32_t)f32Temp;

    f32Temp = MotorCfg.f32_2ndAlignTimeMs * MotorCfg.f32CarrierFreqHz / 1000.0f;
    pv->i32Align2ndTime = (int32_t)f32Temp;

    f32Temp = MotorCfg.f32_3rdAlignTimeMs * MotorCfg.f32CarrierFreqHz / 1000.0f;
    pv->i32Align3rdTime = (int32_t)f32Temp;

    pv->f32_AlignIq = MotorCfg.f32AlignCurrent;
    if (pv->i32Align1stTime <= 0){
        pv->i32Align1stTime = 0;
        pv->f32_AlignDIq = pv->f32_AlignIq;
    }else{
        pv->f32_AlignDIq = pv->f32_AlignIq / pv->i32Align1stTime;
    }

    if (pv->i32Align2ndTime <= 0){
        pv->i32Align2ndTime = 0;
        pv->f32_AlignDTheta = 0.0f;
    }else{
        // DEG_TO_RAD
        pv->f32_AlignDTheta = (MotorCfg.f32_2ndAlignFwdTheta * (M_PI / 180.0f)) / pv->i32Align2ndTime;
    }

    /***************************************************************************
     ** initialize force-drive parameters
     **************************************************************************/
    pv->f32_ForceMaxIq = pv->f32_MaxStartIq;

    pv->f32_ForceDIq = MotorCfg.f32ForceCurrentSlop / MotorCfg.f32CarrierFreqHz;
    pv->f32_ForceAccRate = MotorCfg.f32ForceAccRate / MotorCfg.f32CarrierFreqHz;
    pv->f32_ForceMaxWr = MotorCfg.f32MaxForceSpd;

    if(MotorCfg.f32TransTimeMaxMs < 1000.0f){
        MotorCfg.f32TransTimeMaxMs = 1000.0f;
    }
    pv->u16_TransTimeMax = (uint16_t)(MotorCfg.f32TransTimeMaxMs * MotorCfg.f32CarrierFreqHz / 1000.0f);

    if(MotorCfg.f32ObsConvergeMaxTimeMs < 2000.0f){
        MotorCfg.f32ObsConvergeMaxTimeMs = 2000.0f;
    }
    pv->u16_ConvergeTimeMax = (uint16_t)(MotorCfg.f32ObsConvergeMaxTimeMs * MotorCfg.f32CarrierFreqHz / 1000.0f);
}

void Startup_Control(Startup_t *pv){
    float32_t f32_dTheta;
    float32_t theta_est;
    float32_t wr_est;
    if (MotorCfg.i32ObserverType == USE_ENCODER) {
        theta_est = Encoder.f32_Phase;
        wr_est = Encoder.f32_Vel;
    } else if (MotorCfg.i32ObserverType == SMO_OBSERVER) {
        theta_est = MotorSMObs.f32_ThetaEst;
        wr_est = MotorSMObs.f32_WrF;
    } else if (MotorCfg.i32ObserverType == FLUX_OBSERVER) {
        theta_est = MotorFluxObs.f32_ThetaEst;
        wr_est = MotorFluxObs.f32_WrF;
    } else if (MotorCfg.i32ObserverType == HFI_OBSERVER) {
        theta_est = MotorHFI.f32_ThetaEst;
        wr_est = MotorHFI.f32_WrF;
    } else {
        theta_est = MotorHfi2Obs.f32_ThetaOut;
        wr_est = MotorHfi2Obs.f32_WrOut;
    }
    
    switch (pv->status){
    case Startup_Finish:
        MotorRunPara.f32_DrvAngle = theta_est;
        break;
    case Startup_CLStart:
        MotorTimer.i32SpdRegCnt = 0;
        MotorRunPara.bEnSpdPid = TRUE;
        MotorRunPara.bEnSpdTraj = TRUE;
            
        MotorRunPara.f32_DrvAngle = theta_est;
        if(MotorCfg.i32ObserverType > FLUX_OBSERVER){
            
        }else{
            MotorSpdCtrl.f32_Output = MotorRunPara.IqdRef.f32_Q * 0.5f;
            MotorSpdCtrl.f32_WrISum = MotorSpdCtrl.f32_Output;
            // MotorSpdCtrl.f32_TgtWr = MotorSpdCtrl.f32_MinWr / 2;
            MotorSpdCtrl.f32_TgtWr = wr_est;
        }
        pv->status = Startup_Finish;
        break;
    case Startup_Initialize:
        MotorRunPara.bEnSpdPid = FALSE;
        MotorRunPara.bEnSpdTraj = FALSE;
        MotorRunPara.bEnBreak = FALSE;
        MotorRunPara.bEnCurrentPid  = FALSE;
        pv->i32TimerCnt = 0;
        pv->status = Startup_BscCharge;
        break;
    case Startup_BscCharge:
        InitMcu_EnMotorPwm(TRUE);
        pv->status = Startup_StateDetect;
        
        break;
    case Startup_StateDetect:
        MotorRunPara.f32_DrvAngle = 0.0f;
        MotorRunPara.IqdRef.f32_D = 0.0f;
        MotorRunPara.IqdRef.f32_Q = 0.0f;
        MotorRunPara.bEnCurrentPid = TRUE;
        pv->i32TimerCnt = 0;
        if(MotorCfg.i32ObserverType > FLUX_OBSERVER){
            pv->status = Startup_CLStart;
        }else{
            pv->status = Startup_Align;
        }
    
//        if(pv->u8Status_StateDetect == 0){
//            MotorRunPara.IqdRef.f32_D = 0.0f;
//            MotorRunPara.IqdRef.f32_Q = 0.0f;
//            MotorRunPara.bEnCurrentPid = TRUE;
//            pv->i32TimerCnt = 0;
//            pv->u8Status_StateDetect = 1;
//        }else if(pv->u8Status_StateDetect == 1){
//            pv->i32TimerCnt++;
//            if(pv->i32TimerCnt > pv->i32ObsStableTime){
//                pv->i32TimerCnt = 0;
//                pv->u8Status_StateDetect = 2;
//            }
//        }else if(pv->u8Status_StateDetect == 2){
//            pv->u8Status_StateDetect = 3;
//        }else if(pv->u8Status_StateDetect == 3){
//            if(wr_est >= pv->f32_ObserableWr){
//                MotorTimer.i32SpdRegCnt = 0;
//                MotorRunPara.bEnSpdPid = TRUE;
//                MotorRunPara.bEnSpdTraj = TRUE;
//                if(wr_est < pv->f32_LowSpdUpper){
//                    MotorSpdCtrl.f32_Output = pv->f32_MaxNormaIq;
//                    MotorSpdCtrl.f32_WrISum = MotorSpdCtrl.f32_Output;
//                    MotorSpdCtrl.f32_TgtWr = pv->f32_ForceMaxWr;
//                }else{
//                    MotorSpdCtrl.f32_Output = 0.0f;
//                    MotorSpdCtrl.f32_WrISum = 0.0f;
//                    MotorSpdCtrl.f32_TgtWr = wr_est;
//                }
//                pv->status = Startup_Finish;
//            }else if(wr_est < (-pv->f32_ObserableWr)){
//                pv->status = Startup_HeadWindShortBrake;
//            }else{
//                pv->status = Startup_Align;
//                MotorRunPara.f32_DrvAngle = 0.0f;
//            }
//        }else{
//            pv->i32TimerCnt = 0;
//        }
        break;
    case Startup_IPD:
        break;
    case Startup_Align:
        if(pv->f32_ForceIq < pv->f32_AlignIq){
            pv->f32_ForceIq += pv->f32_AlignDIq;
        }else{
            pv->f32_ForceIq = pv->f32_AlignIq;
        }
        if(pv->u8Status_Align == 0){
            if((--pv->i32Align1stTime) <= 0){
                pv->u8Status_Align = 1;
                Encoder.f32_Offset = 0;
            }
        }else if(pv->u8Status_Align == 1){
            if(MotorCfg.i32ObserverType == USE_ENCODER){
                MotorRunPara.f32_DrvAngle = 0;
            }else{
                MotorRunPara.f32_DrvAngle += pv->f32_AlignDTheta;
                while(MotorRunPara.f32_DrvAngle >= (2.0f * M_PI)) { MotorRunPara.f32_DrvAngle -= (2.0f * M_PI); }
                while(MotorRunPara.f32_DrvAngle < 0.0f) { MotorRunPara.f32_DrvAngle += (2.0f * M_PI); }
            }
            if((--pv->i32Align2ndTime) <= 0){
                pv->u8Status_Align = 2;
            }
        }else if(pv->u8Status_Align == 2){
            if((--pv->i32Align3rdTime) <= 0){
                if(MotorCfg.i32ObserverType == USE_ENCODER){
                    Encoder.f32_Offset = Encoder.f32_Phase;
                    pv->status = Startup_CLStart;
                }else{
                    pv->status = Startup_ForceDrive;
                }
            }
        }
        if(MotorCfg.i32ObserverType == USE_ENCODER){
            MotorRunPara.IqdRef.f32_Q = 0.0f;
            MotorRunPara.IqdRef.f32_D = pv->f32_ForceIq;
        }else{
            MotorRunPara.IqdRef.f32_D = 0.0f;
            MotorRunPara.IqdRef.f32_Q = pv->f32_ForceIq;
        }
        
        break;
    case Startup_ForceDrive:
        if (pv->f32_ForceIq < pv->f32_ForceMaxIq){
            pv->f32_ForceIq += pv->f32_ForceDIq;
        }else{
            pv->f32_ForceIq = pv->f32_ForceMaxIq;
        }
        MotorRunPara.IqdRef.f32_D = 0.0f;
        MotorRunPara.IqdRef.f32_Q = pv->f32_ForceIq;
        pv->f32_ForceWr += pv->f32_ForceAccRate;
        if (pv->f32_ForceWr > pv->f32_ForceMaxWr){
            pv->status = Startup_CLStart;
//            if(MotorCfg.i32ObserverType == FLUX_OBSERVER){
//                if(MotorFluxObs.b_Converged == TRUE){
//                    pv->status = Startup_SoftSwitch;
//                    pv->f32_AngleDiffInit = MotorRunPara.f32_DrvAngle - MotorFluxObs.f32_ThetaEst;
//                    pv->u16_TransCnt = 0;
//                }else{
//                    pv->u16_TransCnt++;
//                    if(pv->u16_TransCnt >= pv->u16_ConvergeTimeMax){
//                        pv->u16_TransCnt = pv->u16_ConvergeTimeMax;
//                        MotorRunPara.u32FaultCode = ERR_CONVERGE; 
//                    }
//                }
//            }else{
//                pv->u16_TransCnt++;
//                if(MotorSMObs.b_Converged == TRUE){
//                    pv->status = Startup_SoftSwitch;
//                    pv->f32_AngleDiffInit = MotorRunPara.f32_DrvAngle - theta_est;
//                    pv->u16_TransCnt = 0;
//                }else{
//                    pv->u16_TransCnt++;
//                    if(pv->u16_TransCnt >= pv->u16_ConvergeTimeMax){
//                        pv->u16_TransCnt = pv->u16_ConvergeTimeMax;
////                        MotorRunPara.u32FaultCode = ERR_CONVERGE; 
//                        pv->status = Startup_SoftSwitch;
//                    }
//                }
//            }
        }
        f32_dTheta = pv->f32_ForceWr * pv->f32_Ts;
        MotorRunPara.f32_DrvAngle += f32_dTheta;
        while(MotorRunPara.f32_DrvAngle >= (2.0f * M_PI)) { MotorRunPara.f32_DrvAngle -= (2.0f * M_PI); }
        while(MotorRunPara.f32_DrvAngle < 0.0f) { MotorRunPara.f32_DrvAngle += (2.0f * M_PI); }
        break;
    case Startup_VfDrive:
        MotorRunPara.UqdRef.f32_Q = MotorRunPara.f32_MaxVs * 0.05f; 
        MotorRunPara.UqdRef.f32_D = 0.0f;
        f32_dTheta = 10.0f * pv->f32_Ts * (2.0f * M_PI); // 10Hz loosely
        MotorRunPara.f32_DrvAngle += f32_dTheta;
        while(MotorRunPara.f32_DrvAngle >= (2.0f * M_PI)) { MotorRunPara.f32_DrvAngle -= (2.0f * M_PI); }
        while(MotorRunPara.f32_DrvAngle < 0.0f) { MotorRunPara.f32_DrvAngle += (2.0f * M_PI); }
        break;

    case Startup_SoftSwitch:
        pv->u16_TransCnt++;
        float f32_Weight = (float)pv->u16_TransCnt / (float)pv->u16_TransTimeMax;
        if (f32_Weight >= 1.0f) {
            f32_Weight = 1.0f;
        }
        float f32_DecayDiff = pv->f32_AngleDiffInit * (1.0f - f32_Weight);
        MotorRunPara.f32_DrvAngle = theta_est + f32_DecayDiff;
        while(MotorRunPara.f32_DrvAngle >= (2.0f * M_PI)) { MotorRunPara.f32_DrvAngle -= (2.0f * M_PI); }
        while(MotorRunPara.f32_DrvAngle < 0.0f)           { MotorRunPara.f32_DrvAngle += (2.0f * M_PI); }

        MotorRunPara.IqdRef.f32_D = 0.0f;
        MotorRunPara.IqdRef.f32_Q = pv->f32_ForceMaxIq;

        pv->f32_ForceWr = pv->f32_ForceMaxWr;

        if (pv->u16_TransCnt >= pv->u16_TransTimeMax) {
            pv->status = Startup_CLStart;
        }

        break;
    case Startup_HeadWindFocBrake:
        break;
    case Startup_HeadWindShortBrake:
//        MotorRunPara.f32_DrvAngle = theta_est;
//        if (0U == pv->u8Status_HeadWindShortBrake){
//            MotorRunPara.UqdRef.f32_D = 0;
//            MotorRunPara.UqdRef.f32_Q = 0;
//            MotorRunPara.Uab.f32_Alpha = 0;
//            MotorRunPara.Uab.f32_Beta = 0;
//            MotorSvpwm.u16Uon = 0xFFFF;
//            MotorSvpwm.u16Uoff = 0xFFFF;
//            MotorSvpwm.u16Von = 0xFFFF;
//            MotorSvpwm.u16Voff = 0xFFFF;
//            MotorSvpwm.u16Won = 0xFFFF;
//            MotorSvpwm.u16Woff = 0xFFFF;
//            MotorRunPara.bEnCurrentPid = FALSE;
//            pv->i32TimerCnt = 0;
//            pv->u8Status_HeadWindShortBrake = 1;
//        }else if (1U == pv->u8Status_HeadWindShortBrake){
//            if (++pv->i32TimerCnt > pv->i32MinShortBrakeTime){
//                pv->i32TimerCnt = 0;
//                pv->u8Status_HeadWindShortBrake = 2;
//            }
//        }else if (2U == pv->u8Status_HeadWindShortBrake){
//            pv->i32TimerCnt++;
//            
//            float32_t temp = MotorRunPara.Iab.f32_Alpha * MotorRunPara.Iab.f32_Alpha + MotorRunPara.Iab.f32_Beta * MotorRunPara.Iab.f32_Beta;

//            if ((temp <= pv->f32_ShortBrakeEndSqrIs)|| (pv->i32TimerCnt >= pv->i32MaxShortBrakeTime)){
//                MotorRunPara.bEnCurrentPid = TRUE;
//                MotorIdPid.f32_Output = 0;
//                MotorIqPid.f32_Output = 0;
//                MotorIdPid.f32_Errl = 0;
//                MotorIqPid.f32_Errl = 0;
//                MotorRunPara.IqdRef.f32_Q = pv->f32_HeadWindIs;
//                MotorRunPara.IqdRef.f32_D = 0;
//                pv->f32_ForceWr = wr_est;
//                pv->status = Startup_HeadWindForceDrive;
//            }
//        }else{
//            
//        }
        break;
    case Startup_HeadWindForceDrive:
        pv->f32_ForceWr += pv->f32_HeadWindAccRate;
        f32_dTheta = pv->f32_ForceWr * pv->f32_Ts;
        MotorRunPara.f32_DrvAngle += f32_dTheta;
        while(MotorRunPara.f32_DrvAngle >= (2.0f * M_PI)) { MotorRunPara.f32_DrvAngle -= (2.0f * M_PI); }
        while(MotorRunPara.f32_DrvAngle < 0.0f)           { MotorRunPara.f32_DrvAngle += (2.0f * M_PI); }

        if (pv->f32_ForceWr > pv->f32_HeadWindForceWr){
            pv->f32_ForceIq = pv->f32_HeadWindIs;
            pv->status = Startup_CLStart;
        }
        break;
    default:
        break;
    }
}
