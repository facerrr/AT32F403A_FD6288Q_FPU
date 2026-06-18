#include "config.h"

AdcSample_t MotorAdcSample;

MotorRunPara_t MotorRunPara;
CmdSpeedSet_t MotorCmdSpdSet;
SpeedCtrl_t MotorSpdCtrl;

FluxObserver_t MotorFluxObs;
SMObserver_t MotorSMObs;
HFI_t MotorHFI;
HFI2Observer_t MotorHfi2Obs;

Startup_t MotorStartup;
StopCtrl_t MotorStopCtrl;

PIControl_t MotorIqPid;
PIControl_t MotorIdPid;
PIControl_t MotorWrPid;

MotorTimer_t MotorTimer;

ProtectVbus_t MotorProtectVbus;
ProtectOC_t MotorProtectOC;

SVPWM_t MotorSvpwm;
LPFilter_t MotorVdLpf;

CalibrationCtrl_t MotorCalibCtrl = {
    .e_CalibStep = CS_NULL
};


void MotorHW_Init(void){
    uint16_t u16PeakCnt;
    uint16_t u16DeadTimeCnt;
    int32_t i32Temp;
    int32_t i32AdcTrigCnt;
    float32_t f32Temp;
    SampleCfg_t SampleCfg;

    u16PeakCnt = HALF_PWM_PERIOD_CYCLES;
    f32Temp = MotorCfg.f32DeadTimeUs * (float32_t)APB1CLK_HZ / 1000000.0f;
    u16DeadTimeCnt = (int32_t)f32Temp;

    i32Temp = APB1CLK_HZ / 1000000;
    f32Temp = MotorCfg.f32AdcTrigTimeUs * (float32_t)i32Temp;
    i32AdcTrigCnt = (int32_t)f32Temp;

    InitMcu_MotorTimer(u16PeakCnt, u16DeadTimeCnt, (uint16_t)i32AdcTrigCnt);
    SVPWM_Init((int32_t)u16PeakCnt, MotorCfg.f32MaxDutyRatio, &MotorSvpwm);
    InitMcu_Adc();

    SampleCfg.f32Fs = MotorCfg.f32CarrierFreqHz;
    SampleCfg.f32IuvwK = 1.0f / (MotorCfg.f32IuvwSampleRs * MotorCfg.f32IuvwSampleK);
    SampleCfg.f32VbusK = MotorCfg.f32VbusSampleK;
    SampleCfg.i32RefIuvwOffset = MotorCfg.i32IuvwRefOffset;
    SampleCfg.i32MaxIuvwOffsetBias = MotorCfg.i32IuvwMaxOffsetBias;
    SampleCfg.f32DetectDelayTimeMs = MotorCfg.f32AdcCheckDelayMs;
    SampleCfg.f32OffsetCheckTimeMs = MotorCfg.f32OffsetCheckTimeMs;
    Adc_InitSample(SampleCfg, &MotorAdcSample);
    
    InitMcu_SystemTicker((int32_t)MotorCfg.f32SysTickerFreqHz);
    
    if(MotorCfg.i32ObserverType == USE_ENCODER){
        SPIx_Init();
    }
}

void MotorSW_Init(void){
    float32_t f32Temp;

    if(MotorRunPara.u8SysRdyFlag == FALSE){
        MotorTimer.i32_1msCnt = (int32_t)(MotorCfg.f32SysTickerFreqHz / 1000);
        MotorTimer.i32_50msCnt = (int32_t)(50 * MotorCfg.f32SysTickerFreqHz / 1000);
        f32Temp = MotorCfg.f32SpdRegPeriodMs * MotorCfg.f32SysTickerFreqHz / 1000.0f;
        MotorTimer.i32SpdRegTime = (int32_t)f32Temp;

        MotorCfg.f32SpdRegPeriodMs = (1000.0f * (float32_t)MotorTimer.i32SpdRegTime) / MotorCfg.f32SysTickerFreqHz;
    
        // Voltage Protect
        MotorProtectVbus.f32_AbnormalHighVbus = MotorCfg.f32AbnormalHighVbus;
        MotorProtectVbus.f32_AbnormalLowVbus = MotorCfg.f32AbnormalLowVbus;
        f32Temp = MotorCfg.f32SysTickerFreqHz * MotorCfg.i32AbnormalVbusTimeUs / 1000000.0f;
        MotorProtectVbus.i32AbnormalTime = (int32_t)f32Temp;
        MotorProtectVbus.f32_OvpThreshold = MotorCfg.f32OverVoltageThold;
        f32Temp = MotorCfg.f32SysTickerFreqHz * MotorCfg.i32PeakOverCurrTimeUs / 1000000.0f;
        MotorProtectVbus.i32OvpTime = (int32_t)f32Temp;
        MotorProtectVbus.f32_UvpThreshold = MotorCfg.f32UnderVoltageThold;
        f32Temp = MotorCfg.f32SysTickerFreqHz * MotorCfg.i32UnderVoltageTimeUs / 1000000.0f;
        MotorProtectVbus.i32UvpTime = (int32_t)f32Temp;
        f32Temp = MotorCfg.f32SysTickerFreqHz * MotorCfg.i32ErrClearTimeMs / 1000.0f;
        MotorProtectVbus.i32ErrClrTime = (int32_t)f32Temp;
        
        // Current Protect
        MotorProtectOC.f32_PeakOcpThreshold = MotorCfg.f32PeakOverCurrThold;
        MotorProtectOC.i32PeakOcpTime = MotorCfg.i32PeakOverCurrTimeUs * MotorCfg.f32CarrierFreqHz / 1000000.0f;
        MotorProtectOC.i32ErrClrTime = MotorCfg.i32ErrClearTimeMs * MotorCfg.f32CarrierFreqHz / 1000.0f;
        
        Speed_InitCmdSpdSet(&MotorCmdSpdSet);

        MotorRunPara.f32_CmdWr = 0.0f;
        MotorRunPara.i32CmdWrDir = ROTOR_DIR_CCW;
        MotorRunPara.i32TgtWrDir = ROTOR_DIR_CCW;
        MotorRunPara.u32MotorStatus = MOTOR_STUS_IDLE;
        MotorRunPara.u32FaultCode = ERR_NONE;

        MotorRunPara.f32_MaxVsk = 0.57735027f; // 1/sqrt(3)
        MotorRunPara.u8SysRdyFlag = TRUE;
    }
    
    if(MotorCfg.i32ObserverType == USE_ENCODER){
        Encoder_Init();
    }

    MotorRunPara.bEnSpdPid = FALSE;
    MotorRunPara.bEnSpdTraj = FALSE;
    MotorRunPara.bEnCurrentPid = FALSE;
    MotorRunPara.bEnBreak = FALSE;

    Calibration_Init(&MotorCalibCtrl);
    
    FluxObserver_Init(&MotorFluxObs);
    SMObserver_Init(&MotorSMObs);
    HFI_Init(&MotorHFI, &MotorHfi2Obs);
    
    Speed_InitSpdTraj(&MotorSpdCtrl);
    
    memset(&MotorWrPid, 0, sizeof(PIControl_t));
    MotorWrPid.f32_Kp = MotorCfg.f32WrKp;
    MotorWrPid.f32_Ki = MotorCfg.f32WrKi / (1000.0f / MotorCfg.f32SpdRegPeriodMs);
    MotorWrPid.f32_OutMax = MotorCfg.f32MaxNormaIs;
    MotorWrPid.f32_OutMin = -MotorWrPid.f32_OutMax;
    MotorWrPid.f32_AbsErrMax = 200;
    
    float32_t f32_UMax = MotorCfg.f32AbnormalHighVbus * 0.57735027f; // Set initial max to bus limit
    float32_t CurrentKp = MotorCfg.f32Ls * MotorCfg.f32CarrierFreqHz / 3.0f;
    float32_t CurrentKi = MotorCfg.f32Rs / 3.0f;
    memset(&MotorIdPid, 0, sizeof(PIControl_t));
    MotorIdPid.f32_Errl = 0.0f;
    MotorIdPid.f32_Kp = 0.9f * CurrentKp;
    MotorIdPid.f32_Ki = 0.9f * CurrentKi;
    MotorIdPid.f32_OutMax = f32_UMax;
    MotorIdPid.f32_OutMin = -f32_UMax;
    MotorIdPid.f32_AbsErrMax = 1.0f;

    memset(&MotorIqPid, 0, sizeof(PIControl_t));
    MotorIqPid.f32_Errl = 0.0f;
    MotorIqPid.f32_Kp = CurrentKp;
    MotorIqPid.f32_Ki = CurrentKi;
    MotorIqPid.f32_OutMax = f32_UMax;
    MotorIqPid.f32_OutMin = -f32_UMax;    
    MotorIqPid.f32_AbsErrMax = 1.0f;
    
    MotorVdLpf.f32_LpfK = (2.0f * M_PI) * 100.0f / MotorCfg.f32SysTickerFreqHz;
    MotorVdLpf.f32_Output = 0.0f;
    
    Startup_Init(&MotorStartup);

    MotorStopCtrl.bEnBrakeStop = FALSE;
    MotorStopCtrl.u8ShortBrakeStatus = 0;
    MotorStopCtrl.i32TimerCnt = 0;
    MotorStopCtrl.Status = Stop_Initialize;
}

void Motor_FastStop(void){
    InitMcu_EnMotorPwm(FALSE);
    InitMcu_UpdateMotorPwmDuty(0, 0, 0);

    MotorRunPara.u32MotorStatus = MOTOR_STUS_IDLE;
    MotorRunPara.bEnSpdPid = FALSE;
    MotorRunPara.bEnCurrentPid = FALSE;
    MotorRunPara.bEnBreak = FALSE;
    MotorRunPara.f32_RealTimeWr = 0.0f;

    MotorRunPara.Uqd.f32_D = 0.0f;
    MotorRunPara.Uqd.f32_Q = 0.0f;
    MotorRunPara.Uab.f32_Alpha = 0.0f;
    MotorRunPara.Uab.f32_Beta = 0.0f;
    MotorRunPara.Iab.f32_Alpha = 0.0f;
    MotorRunPara.Iab.f32_Beta = 0.0f;
    MotorRunPara.Iqd.f32_D = 0.0f;
    MotorRunPara.Iqd.f32_Q = 0.0f;
    MotorRunPara.IqdRef.f32_D = 0.0f;
    MotorRunPara.IqdRef.f32_Q = 0.0f;

    MotorSvpwm.u16Uon = 0;
    MotorSvpwm.u16Von = 0;
    MotorSvpwm.u16Won = 0;
}

void Motor_CVLimit(void){
    float32_t f32Temp;
    float32_t f32Val;

    MotorRunPara.f32_MaxVs = (MotorRunPara.f32_MaxVsk * MotorRunPara.f32_AvgVbus);

    MotorIdPid.f32_OutMax = MotorRunPara.f32_MaxVs;
    MotorIdPid.f32_OutMin = -MotorIdPid.f32_OutMax;
    
    MotorRunPara.f32_VdLpf = Filter_FirstLpf(MotorRunPara.UqdRef.f32_D, &MotorVdLpf);
    f32Temp = fabsf(MotorRunPara.f32_VdLpf);
    f32Val = MotorRunPara.f32_MaxVs * MotorRunPara.f32_MaxVs - f32Temp * f32Temp;
    if (f32Val > 0.0f) {
        f32Temp = sqrtf(f32Val);
    } else {
        f32Temp = 0.0f;
    }

    MotorIqPid.f32_OutMax = f32Temp;
    MotorIqPid.f32_OutMin = -MotorIqPid.f32_OutMax;
}

void Motor_SysTickerIsrHandler(void){
    MotorTimer.i32_1msCnt++;
    MotorTimer.i32_50msCnt++;
    MotorTimer.i32SpdRegCnt++;
    
    InitMcu_StartAdc1Sample();

    UTILS_LP_FAST(MotorRunPara.f32_AvgVbus, MotorRunPara.f32_Vbus, 0.8);
    Prot_MonitorVbus(MotorRunPara.f32_AvgVbus, &MotorProtectVbus);
    Motor_CVLimit();
    
    if (MotorCfg.i32ObserverType == FLUX_OBSERVER) {
        FluxSpeed_Calc(&MotorFluxObs);
    }
    
    MotorRunPara.f32_PWR = (MotorRunPara.UqdRef.f32_D * MotorRunPara.Iqd.f32_D + MotorRunPara.UqdRef.f32_Q * MotorRunPara.Iqd.f32_Q);

    if(MotorTimer.i32_1msCnt >= MotorTimer.i32_1msTime) {
        MotorTimer.i32_1msCnt = 0;
    }
    if(MotorTimer.i32_50msCnt >= MotorTimer.i32_50msTime){
        MotorTimer.i32_50msCnt = 0;
    }
    if(MotorTimer.i32SpdRegCnt >= MotorTimer.i32SpdRegTime){
        MotorTimer.i32SpdRegCnt = 0;
        if (MotorRunPara.bEnSpdTraj == TRUE) {
            MotorRunPara.f32_TgtWr = Speed_TgtSpdTraj(MotorRunPara.f32_CmdWr, &MotorSpdCtrl);
        }
        if(MotorRunPara.bEnSpdPid == TRUE){
            MotorRunPara.IqdRef.f32_Q = PID_PosPid(MotorRunPara.f32_TgtWr,  MotorRunPara.f32_RealTimeWr, &MotorWrPid);
            MotorRunPara.IqdRef.f32_D = 0.0f;
        }
    }
}

void Motor_MainIsrHandler(void){

    Prot_MotorOverCurrent(MotorRunPara.Iuvw, &MotorProtectOC);

    if(MotorRunPara.u32FaultCode != ERR_NONE){
        Motor_FastStop();
    }

    Adc_Sample(&MotorAdcSample);
    MotorRunPara.bSampleCmp = 1;
    
    if(MotorCfg.i32ObserverType == USE_ENCODER){
        Encoder_Loop();
        utils_norm_angle_rad(&Encoder.f32_Phase);
    }
    CLARKE_Calc(&MotorRunPara);

    if(MotorRunPara.u32MotorStatus == MOTOR_STUS_CALIBRATE){
        Calibration_Loop(&MotorCalibCtrl);
        if(MotorCalibCtrl.i32_OutputType == 1){
            MotorRunPara.f32_Sin = sin_f32(MotorRunPara.f32_DrvAngle);
            MotorRunPara.f32_Cos = cos_f32(MotorRunPara.f32_DrvAngle);
            IPARK_Calc(&MotorRunPara);
            SVPWM_Calc(MotorRunPara.Uab.f32_Alpha, MotorRunPara.Uab.f32_Beta, MotorRunPara.f32_AvgVbus, &MotorSvpwm);
            InitMcu_UpdateMotorPwmDuty(MotorSvpwm.u16Uon, MotorSvpwm.u16Von, MotorSvpwm.u16Won);

        }else if(MotorCalibCtrl.i32_OutputType == 2){
            SVPWM_Calc(MotorRunPara.Uab.f32_Alpha, MotorRunPara.Uab.f32_Beta, MotorRunPara.f32_AvgVbus, &MotorSvpwm);
            InitMcu_UpdateMotorPwmDuty(MotorSvpwm.u16Uon, MotorSvpwm.u16Von, MotorSvpwm.u16Won);

        }else{
            InitMcu_UpdateMotorPwmDuty(0, 0, 0);
        }

    }else if(MotorRunPara.u32MotorStatus != MOTOR_STUS_IDLE){
        
        if(MotorCfg.i32ObserverType == SMO_OBSERVER){
            SMObserver_Calc(&MotorSMObs, MotorRunPara.Uab, MotorRunPara.Iab);
            MotorRunPara.f32_RealTimeWr = MotorSMObs.f32_WrF;
        }else if(MotorCfg.i32ObserverType == FLUX_OBSERVER){
            FluxObserver_Calc(&MotorFluxObs, MotorRunPara.Uab, MotorRunPara.Iab);
            MotorRunPara.f32_RealTimeWr = MotorFluxObs.f32_WrF;
        }else if(MotorCfg.i32ObserverType == USE_ENCODER){
            MotorRunPara.f32_RealTimeWr = Encoder.f32_PhaseVel;
            MotorRunPara.f32_RealTimePos = Encoder.f32_Pos;
        }else if(MotorCfg.i32ObserverType > FLUX_OBSERVER){
            HFI_Calculate(&MotorHFI, MotorRunPara.Iqd, MotorRunPara.Iab);
            if(MotorCfg.i32ObserverType == HFI_SMO_OBSERVER){
                SMObserver_Calc(&MotorSMObs, MotorRunPara.Uab, MotorRunPara.Iab);
                MotorHfi2Obs.f32_HFIWr = MotorHFI.f32_WrF;
                MotorHfi2Obs.f32_ObsWr = MotorSMObs.f32_WrF;
                MotorHfi2Obs.f32_HFIThetaEst = MotorHFI.f32_ThetaEst;
                MotorHfi2Obs.f32_ObsThetaEst = MotorSMObs.f32_ThetaEst;
                MotorHfi2Obs.f32_TrgtWr = MotorRunPara.f32_CmdWr;
                HFI2Observer_Calculate(&MotorHfi2Obs, &MotorHFI);
                
            }else if(MotorCfg.i32ObserverType == HFI_FLUX_OBSERVER){
                FluxObserver_Calc(&MotorFluxObs, MotorRunPara.Uab, MotorRunPara.Iab);
                MotorHfi2Obs.f32_HFIWr = MotorHFI.f32_WrF;
                MotorHfi2Obs.f32_ObsWr = MotorFluxObs.f32_WrF;
                MotorHfi2Obs.f32_HFIThetaEst = MotorHFI.f32_ThetaEst;
                MotorHfi2Obs.f32_ObsThetaEst = MotorFluxObs.f32_ThetaEst;
                MotorHfi2Obs.f32_TrgtWr = MotorRunPara.f32_CmdWr;
                HFI2Observer_Calculate(&MotorHfi2Obs, &MotorHFI);
            }
        }
        
        if(MotorRunPara.u32MotorStatus == MOTOR_STUS_DRV_RUN){
            Startup_Control(&MotorStartup);
        }else{
            Stop_Control(&MotorStopCtrl);
        }
        
        MotorRunPara.f32_Sin = sin_f32(MotorRunPara.f32_DrvAngle);
        MotorRunPara.f32_Cos = cos_f32(MotorRunPara.f32_DrvAngle);
        PARK_Calc(&MotorRunPara);
        if(MotorRunPara.bEnCurrentPid == TRUE){
            float32_t Id_fbk;
            float32_t Iq_fbk;
            
            if(MotorHFI.u8_Enable == 0){
                Id_fbk = MotorRunPara.Iqd.f32_D;
                Iq_fbk = MotorRunPara.Iqd.f32_Q;
            }else{
                Id_fbk = MotorHFI.f32_IdBase;
                Iq_fbk = MotorHFI.f32_IqBase;
            }
            
            MotorRunPara.UqdRef.f32_Q = PID_IncPid(MotorRunPara.IqdRef.f32_Q, Iq_fbk, &MotorIqPid);
            
            if(MotorHFI.u8_Enable == 1){
                if(MotorHFI.u8_Dir == 0){
                    MotorRunPara.UqdRef.f32_D = MotorHFI.f32_Uin + PID_IncPid(MotorRunPara.IqdRef.f32_D, Id_fbk, &MotorIdPid);
                }else{
                    MotorRunPara.UqdRef.f32_D = -MotorHFI.f32_Uin + PID_IncPid(MotorRunPara.IqdRef.f32_D, Id_fbk, &MotorIdPid);
                }
            }else{
                MotorRunPara.UqdRef.f32_D = PID_IncPid(MotorRunPara.IqdRef.f32_D, Id_fbk, &MotorIdPid);
                
                if(MotorCfg.i32FeedforwardEnable == 1){
                    float32_t Vd_decouple = -1.0f * MotorRunPara.f32_RealTimeWr * MotorCfg.f32Ls * MotorRunPara.Iqd.f32_Q;
                    float32_t Vq_decouple = MotorRunPara.f32_RealTimeWr * MotorCfg.f32Ls * MotorRunPara.Iqd.f32_D + MotorRunPara.f32_RealTimeWr * MotorCfg.f32LambdaF;
                    MotorRunPara.UqdRef.f32_D += Vd_decouple;
                    MotorRunPara.UqdRef.f32_Q += Vq_decouple;
                }
            }
            
            IPARK_Calc(&MotorRunPara);
            SVPWM_Calc(MotorRunPara.Uab.f32_Alpha, MotorRunPara.Uab.f32_Beta, MotorRunPara.f32_AvgVbus, &MotorSvpwm);
        }else if(MotorStartup.status == Startup_VfDrive){
            IPARK_Calc(&MotorRunPara);
            SVPWM_Calc(MotorRunPara.Uab.f32_Alpha, MotorRunPara.Uab.f32_Beta, MotorRunPara.f32_AvgVbus, &MotorSvpwm);
        }
        
        if(MotorRunPara.i32TgtWrDir == ROTOR_DIR_CCW){
            InitMcu_UpdateMotorPwmDuty(MotorSvpwm.u16Uon, MotorSvpwm.u16Von, MotorSvpwm.u16Won);
        }else if(MotorRunPara.i32TgtWrDir == ROTOR_DIR_CW){
            InitMcu_UpdateMotorPwmDuty(MotorSvpwm.u16Von, MotorSvpwm.u16Uon, MotorSvpwm.u16Won);
        }else{
            InitMcu_UpdateMotorPwmDuty(0, 0, 0);
        }
//        InitMcu_UpdateMotorPwmDuty(MotorSvpwm.u16Uon, MotorSvpwm.u16Von, MotorSvpwm.u16Won);
    }else{
        Motor_FastStop();
    }
}
