
#include "config.h"

void Calibration_Init(pCalibrationCtrl pv)
{
    memset(pv, 0, sizeof(CalibrationCtrl_t));
    pv->f32_FluxEstRaw = 0.00245f;
    pv->f32_CalibFlux = 0.00245f;
    pv->f32_Ts = 1.0f / (float32_t)CARRIER_FREQUENCY;
}


void Calibration_Start(pCalibrationCtrl pv)
{
    pv->f32_CalibCurrent = MotorCfg.f32CalibCurrent;
    pv->f32_CalibVoltage = MotorCfg.f32CalibVoltage;
    pv->f32_Ts = 1.0f / (float32_t)CARRIER_FREQUENCY;

    pv->f32_Resistance = 0.0f;
    pv->f32_Inductance = 0.0f;
    pv->i32_PolePairs = 0;
    pv->u32_LoopCount = 0;
    pv->f32_ActualCurrent = 0.0f;
    pv->f32_LastIalpha = 0.0f;
    pv->f32_DeltaI = 0.0f;
    pv->f32_TestVoltage = 0.0f;
    pv->f32_PhaseSet = 0.0f;
    pv->f32_StartCount = 0.0f;
    pv->i16_SampleCount = 0;
    pv->f32_NextSampleTime = 0.0f;
    
    InitMcu_EnMotorPwm(TRUE);
    pv->e_CalibStep = CS_MOTOR_R_START;
    MotorRunPara.u32MotorStatus = MOTOR_STUS_CALIBRATE; 

}

void Calibration_Loop(pCalibrationCtrl pv)
{
    const float32_t kI = 1.0f;
    const uint32_t num_R_cycles = CARRIER_FREQUENCY * 2;
    const float32_t resistance_calib_max_voltage = 2.0f;
    const uint32_t num_L_cycles = CARRIER_FREQUENCY / 2;
    const float32_t calib_phase_vel = M_PI;

    float32_t time = (float32_t)pv->u32_LoopCount * pv->f32_Ts;
    float32_t voltage = pv->f32_CalibCurrent * pv->f32_Resistance * 3.0f / 2.0f;

    switch (pv->e_CalibStep) {
        case CS_NULL:
            break;
        
        case CS_MOTOR_R_START:
            pv->u32_LoopCount  = 0;
            pv->f32_TestVoltage = 0.0f;
            pv->e_CalibStep  = CS_MOTOR_R_LOOP;
            break;
        
        case CS_MOTOR_R_LOOP:
            pv->f32_ActualCurrent = MotorRunPara.Iuvw.f32_U;
            pv->f32_TestVoltage += (kI * pv->f32_Ts) * (pv->f32_CalibCurrent - pv->f32_ActualCurrent);

            if (ABS(pv->f32_TestVoltage) > resistance_calib_max_voltage) {
                pv->i32_OutputType = 0;
                pv->e_CalibStep = CS_MOTOR_R_END;
            } else {
                pv->i32_OutputType = 2;
                MotorRunPara.Uab.f32_Alpha = pv->f32_TestVoltage;
                MotorRunPara.Uab.f32_Beta = 0.0f;
            }

            if (pv->u32_LoopCount >= num_R_cycles) {
                MotorRunPara.Uab.f32_Alpha = 0.0f;
                pv->e_CalibStep = CS_MOTOR_R_END;
            }
            break;

        case CS_MOTOR_R_END:
            pv->f32_Resistance = (pv->f32_TestVoltage / pv->f32_CalibCurrent);
            pv->e_CalibStep = CS_MOTOR_L_START;
            break;

        case CS_MOTOR_L_START:
            pv->u32_LoopCount = 0;
            pv->f32_DeltaI = 0.0f;
            pv->f32_TestVoltage = pv->f32_CalibVoltage;
            pv->e_CalibStep = CS_MOTOR_L_LOOP;
            break;

        case CS_MOTOR_L_LOOP: {
            float32_t Ialpha = MotorRunPara.Iab.f32_Alpha;
            float32_t sign = pv->f32_TestVoltage >= 0.0f ? 1.0f : -1.0f;
            pv->f32_DeltaI += -sign * (Ialpha - pv->f32_LastIalpha);
            pv->f32_LastIalpha = Ialpha;
      
            pv->f32_TestVoltage *= -1.0f;
            MotorRunPara.Uab.f32_Alpha = pv->f32_TestVoltage;
            MotorRunPara.Uab.f32_Beta = 0.0f;
            pv->i32_OutputType = 2;

            if (pv->u32_LoopCount >= num_L_cycles) {
                pv->i32_OutputType = 0;
                pv->e_CalibStep = CS_MOTOR_L_END;
            }
        } break;
        
        case CS_MOTOR_L_END: {
            float32_t dI_by_dt = pv->f32_DeltaI / ((float32_t)num_L_cycles * pv->f32_Ts);
            float32_t L = pv->f32_TestVoltage / dI_by_dt;
            pv->f32_Inductance = L;

            pv->f32_PhaseSet = 0.0f;
            pv->u32_LoopCount = 0;
            if(MotorCfg.i32ObserverType == USE_ENCODER) {
                pv->e_CalibStep = CS_DIR_PP_START;
            }else{
                pv->e_CalibStep = CS_NULL;
                MotorRunPara.u32MotorStatus = MOTOR_STUS_IDLE;
            }
            
        } break;

        case CS_DIR_PP_START:
            MotorRunPara.UqdRef.f32_D = voltage;
            MotorRunPara.UqdRef.f32_Q = 0.0f;
            MotorRunPara.f32_DrvAngle = pv->f32_PhaseSet;
            pv->i32_OutputType = 1;
            if (time >= 2.0f) {
                pv->f32_StartCount = (float32_t)Encoder.i64_ShadowCnt;
                pv->e_CalibStep = CS_DIR_PP_LOOP;
                break;
            }
            break;

        case CS_DIR_PP_LOOP:
            pv->f32_PhaseSet += calib_phase_vel * pv->f32_Ts;
            MotorRunPara.UqdRef.f32_D = voltage;
            MotorRunPara.UqdRef.f32_Q = 0.0f;
            MotorRunPara.f32_DrvAngle = pv->f32_PhaseSet;
            pv->i32_OutputType = 1;
            if (pv->f32_PhaseSet >= 4.0f * M_2PI) {
                pv->e_CalibStep = CS_DIR_PP_END;
            }
            break;

        case CS_DIR_PP_END: {
            int32_t diff = Encoder.i64_ShadowCnt - (int32_t)pv->f32_StartCount;
            if (diff > 0) {
                Encoder.i32_Dir = 1;
            } else {
                Encoder.i32_Dir = -1;
            }
            pv->i32_OutputType = 0;
            pv->i32_PolePairs = round(4.0f / ABS((float32_t)diff / ENCODER_CPR_F));
            pv->e_CalibStep = CS_NULL;
            MotorRunPara.u32MotorStatus = MOTOR_STUS_IDLE;
        } break;
        
        default:
            break;
    }
    
    MotorCalibCtrl.u32_LoopCount++;
}



void MotorFlux_OnlineCalib(pCalibrationCtrl pv, AB_t Uab, AB_t Iab)
{   
    pv->f32_X1 += (Uab.f32_Alpha - MotorCfg.f32Rs * Iab.f32_Alpha) * pv->f32_Ts - MotorCfg.f32Ls * (Iab.f32_Alpha - pv->f32_IalphaLast);
    pv->f32_X2 += (Uab.f32_Beta - MotorCfg.f32Rs * Iab.f32_Beta) * pv->f32_Ts - MotorCfg.f32Ls * (Iab.f32_Beta - pv->f32_IbetaLast);
    pv->f32_IalphaLast = Iab.f32_Alpha;
    pv->f32_IbetaLast = Iab.f32_Beta;

    const float32_t f32_SMOFluxAdaptK = 0.1f;
    const float32_t f32_SMOFluxObsGain = 600.0f / MotorCfg.f32Ls;

    float32_t flux_min = MotorCfg.f32LambdaF * 0.3f;
    float32_t flux_max = MotorCfg.f32LambdaF * 2.5f;

    float32_t err = SQ(pv->f32_FluxEstRaw) - (SQ(pv->f32_X1) + SQ(pv->f32_X2));
    float32_t gamma_half = 0.5f * f32_SMOFluxObsGain;

    pv->f32_FluxEstRaw += f32_SMOFluxAdaptK * gamma_half * pv->f32_FluxEstRaw * (-err) * pv->f32_Ts;
    pv->f32_FluxEstRaw = CLAMP(pv->f32_FluxEstRaw, flux_min, flux_max);

    pv->f32_X1 = CLAMP(pv->f32_X1, -pv->f32_FluxEstRaw, pv->f32_FluxEstRaw);
    pv->f32_X2 = CLAMP(pv->f32_X2, -pv->f32_FluxEstRaw, pv->f32_FluxEstRaw);

    UTILS_LP_FAST(pv->f32_CalibFlux, pv->f32_FluxEstRaw, 0.005f);
}