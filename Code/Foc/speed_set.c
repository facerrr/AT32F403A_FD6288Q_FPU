#include "config.h"

SpeedFCtrl_t SpeedFCtrl;

void Speed_InitCmdSpdSet(pCmdSpeedSet pv){
    float32_t f32Temp;
    f32Temp = (60.0f) / (M_2PI * (float32_t)MotorCfg.i32PolePairs);
    pv->f32_Wr2Rpm = f32Temp;

    f32Temp = (float32_t)MotorCfg.i32PolePairs * M_2PI / 60.0f;
    pv->f32_Rpm2Wr = f32Temp;
    pv->i32MinRpm = MotorCfg.i32MinSpeedRpm;
    pv->i32MaxRpm = MotorCfg.i32MaxSpeedRpm;

    pv->i32CmdRpm = 0;
    pv->i32RealRpm = 0;
    pv->i32PreCmdRpm = 0;

    memset(&SpeedFCtrl, 0, sizeof(SpeedFCtrl));
}


void Speed_InitSpdTraj(pSpeedCtrl pv){
    memset(pv, 0, sizeof(SpeedCtrl_t));
    pv->f32_WrAccRate = MotorCfg.f32AccPerSec;
    pv->f32_WrDecRate = MotorCfg.f32DecPerSec;
    pv->f32_MaxWr = MotorCfg.i32MaxSpeedRpm * (float32_t)MotorCfg.i32PolePairs * M_2PI / 60.0f;
    pv->f32_MinWr = MotorCfg.i32MinSpeedRpm * (float32_t)MotorCfg.i32PolePairs * M_2PI / 60.0f;
    pv->f32_WrMaxStep = MotorCfg.f32AccPerSec / (1000.0f / MotorCfg.f32SpdRegPeriodMs);
    pv->f32_WrKp = MotorCfg.f32WrKp;
    pv->f32_WrKi = MotorCfg.f32WrKi / (1000.0f / MotorCfg.f32SpdRegPeriodMs);
    pv->f32_WrOutMax = MotorCfg.f32MaxNormaIs;
}


void Speed_CmdSpdSet(pCmdSpeedSet pv){

    int32_t i32Rpm;
    int32_t i32AbsRpm;

    i32Rpm = (int32_t)(MotorRunPara.f32_RealTimeWr * pv->f32_Wr2Rpm);
    pv->i32RealRpm = i32Rpm;
//    if(ROTOR_DIR_CCW == MotorRunPara.i32TgtWrDir){
//        pv->i32RealRpm = i32Rpm;
//    }else{
//        pv->i32RealRpm = -i32Rpm;
//    }

    if(MotorRunPara.u32FaultCode != ERR_NONE || MotorRunPara.u8SysRdyFlag == FALSE){
        pv->i32CmdRpm = 0;
        pv->i32PreCmdRpm = 0;
        MotorRunPara.f32_CmdWr = 0.0f;
        MotorRunPara.u32MotorStatus = MOTOR_STUS_IDLE;
        return;
    }

    if(pv->i32CmdRpm != pv->i32PreCmdRpm){
        i32Rpm = pv->i32CmdRpm;
        i32AbsRpm = ABS(i32Rpm);
        if(i32AbsRpm < pv->i32MinRpm){
            i32AbsRpm = pv->i32MinRpm;
        }

        if(i32Rpm > 0){
            i32Rpm = i32AbsRpm;
            MotorRunPara.i32CmdWrDir = ROTOR_DIR_CCW;
        }else if(i32Rpm < 0){
            i32Rpm = -i32AbsRpm;
            MotorRunPara.i32CmdWrDir = ROTOR_DIR_CW;
        }else{
            i32Rpm = 0;
            i32AbsRpm = 0;
        }

        pv->i32CmdRpm = i32Rpm;
        pv->i32PreCmdRpm = i32Rpm;
        MotorRunPara.f32_CmdWr = (float32_t)i32AbsRpm * pv->f32_Rpm2Wr;
    }

    if(0 == pv->i32PreCmdRpm){
        if(MotorRunPara.u32MotorStatus == MOTOR_STUS_DRV_RUN){
            MotorRunPara.u32MotorStatus = MOTOR_STUS_DRV_STOP;
        }
    }else{
        if(MotorRunPara.u32MotorStatus == MOTOR_STUS_IDLE){
            MotorSW_Init(); 
            MotorRunPara.i32TgtWrDir = MotorRunPara.i32CmdWrDir;
            MotorRunPara.u32MotorStatus = MOTOR_STUS_DRV_RUN;
        }else if(MotorRunPara.u32MotorStatus == MOTOR_STUS_DRV_RUN){
            if(MotorRunPara.i32CmdWrDir != MotorRunPara.i32TgtWrDir){
                if(MotorCfg.i32BreakEnable == TRUE){
                    MotorRunPara.u32MotorStatus = MOTOR_STUS_DRV_STOP;
                }else{
                    
                }
            }
        }
    }
}


float32_t Speed_TgtSpdTraj(float32_t f32_CmdWr, pSpeedCtrl pv){
    float32_t f32_DeltaWr;
    
    if(MotorRunPara.i32CmdWrDir != MotorRunPara.i32TgtWrDir && MotorCfg.i32BreakEnable == FALSE){
        if(pv->f32_TgtWr < pv->f32_WrMaxStep){
            MotorRunPara.u32MotorStatus = MOTOR_STUS_DRV_STOP;
            pv->f32_TgtWr = pv->f32_WrMaxStep;
        }else{
            pv->f32_TgtWr -= pv->f32_WrMaxStep;
        }
        pv->f32_TgtWr = CLAMP(pv->f32_TgtWr, -pv->f32_MaxWr, pv->f32_MaxWr);
        
    }else{
        f32_DeltaWr = f32_CmdWr - pv->f32_TgtWr;
        float32_t f32_Step = CLAMP(f32_DeltaWr, -pv->f32_WrMaxStep, pv->f32_WrMaxStep);
        pv->f32_TgtWr += f32_Step;
        pv->f32_TgtWr = CLAMP(pv->f32_TgtWr, -pv->f32_MaxWr, pv->f32_MaxWr);
    }
    
    
    return pv->f32_TgtWr;
}

