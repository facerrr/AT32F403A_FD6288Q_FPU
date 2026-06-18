#include "config.h"

void Stop_Control(pStopCtrl_t pv){
    float32_t f32_Temp;
    if (MotorCfg.i32ObserverType == USE_ENCODER /* ENCODER */) {
        MotorRunPara.f32_DrvAngle = Encoder.f32_Phase;
    } else if (MotorCfg.i32ObserverType == SMO_OBSERVER /* SMO */) {
        MotorRunPara.f32_DrvAngle = MotorSMObs.f32_Theta;
    } else {
        MotorRunPara.f32_DrvAngle = MotorFluxObs.f32_ThetaEst;
    }
    if(MotorStartup.status != Startup_Finish){
        MotorRunPara.bEnSpdPid = FALSE;
        MotorRunPara.bEnSpdTraj = FALSE;
        MotorRunPara.bEnBreak = FALSE;
        MotorRunPara.bEnCurrentPid = FALSE;

        Motor_FastStop();
        pv->Status = Stop_Finish;
        return;
    }

    if(pv->Status == Stop_Initialize){
        pv->i32TimerCnt = 0;
        if(pv->bEnBrakeStop == TRUE){
            if(MotorFluxObs.f32_WrF < MotorStartup.f32_ObserableWr){
                MotorRunPara.bEnSpdPid = FALSE;
                MotorRunPara.bEnSpdTraj = FALSE;
                MotorRunPara.bEnBreak = FALSE;
                MotorRunPara.bEnCurrentPid = FALSE;

                Motor_FastStop();
                pv->Status = Stop_Finish;
            }else{
                pv->Status = Stop_ShortBrake;
                pv->u8ShortBrakeStatus = 0;
            }
        }else{
            pv->Status = Stop_Finish;
            Motor_FastStop();
        }
    }else if (pv->Status == Stop_DecCurrent){

    }else if (pv->Status == Stop_ElecBrake){

    }else if (pv->Status == Stop_ShortBrake){
        if(pv->u8ShortBrakeStatus == 0){
            MotorRunPara.bEnSpdPid = FALSE;
            MotorRunPara.bEnSpdTraj = FALSE;
            MotorRunPara.bEnBreak = FALSE;
            MotorRunPara.bEnCurrentPid = FALSE;

            MotorRunPara.UqdRef.f32_D = 0.0f;
            MotorRunPara.UqdRef.f32_Q = 0.0f;
            MotorRunPara.Uab.f32_Alpha = 0.0f;
            MotorRunPara.Uab.f32_Beta = 0.0f;
            
            MotorSvpwm.u16Uon = 0x0;
            MotorSvpwm.u16Von = 0x0;
            MotorSvpwm.u16Won = 0x0;
            pv->i32TimerCnt = 0;
            pv->u8ShortBrakeStatus = 1;
        }else if(pv->u8ShortBrakeStatus == 1){
            if(++pv->i32TimerCnt > MotorStartup.i32MinShortBrakeTime){
                pv->i32TimerCnt = 0;
                pv->u8ShortBrakeStatus = 2;
            }
        }else{
            pv->i32TimerCnt++;
            f32_Temp = MotorRunPara.Iab.f32_Alpha * MotorRunPara.Iab.f32_Alpha +
                       MotorRunPara.Iab.f32_Beta * MotorRunPara.Iab.f32_Beta;
            if((f32_Temp <= MotorStartup.f32_ShortBrakeEndSqrIs) || (pv->i32TimerCnt >= MotorStartup.i32MaxShortBrakeTime)){
                MotorRunPara.bEnCurrentPid = TRUE;
                MotorIdPid.f32_Output = 0.0f;
                MotorIqPid.f32_Output = 0.0f;
                MotorIdPid.f32_Errl = 0.0f;
                MotorIqPid.f32_Errl = 0.0f;
                MotorRunPara.IqdRef.f32_D = 0.0f;
                MotorRunPara.IqdRef.f32_Q = 0.0f;
                pv->Status = Stop_Finish;
            }
        }
    }else if(pv->Status == Stop_Finish){
        MotorRunPara.bEnSpdPid = FALSE;
        MotorRunPara.bEnSpdTraj = FALSE;
        MotorRunPara.bEnBreak = FALSE;
        MotorRunPara.bEnCurrentPid = FALSE;

        Motor_FastStop();
    }else{
        MotorRunPara.bEnSpdPid = FALSE;
        MotorRunPara.bEnSpdTraj = FALSE;
        MotorRunPara.bEnBreak = FALSE;
        MotorRunPara.bEnCurrentPid = FALSE;

        Motor_FastStop();
    }
}
