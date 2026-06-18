#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__

#include "mc_type.h"

typedef enum {
    CS_NULL = 0,

    CS_MOTOR_R_START,
    CS_MOTOR_R_LOOP,
    CS_MOTOR_R_END,

    CS_MOTOR_L_START,
    CS_MOTOR_L_LOOP,
    CS_MOTOR_L_END,

    CS_DIR_PP_START,
    CS_DIR_PP_LOOP,
    CS_DIR_PP_END
} CalibStep_e;

typedef struct{
    CalibStep_e e_CalibStep;

    float32_t f32_Ts;
    float32_t f32_CalibVoltage;
    float32_t f32_CalibCurrent;

    float32_t f32_Resistance;
    float32_t f32_Inductance;
    int32_t i32_PolePairs;

    int32_t i32_OutputType;

    uint32_t u32_LoopCount;
    float32_t f32_ActualCurrent;
    float32_t f32_LastIalpha;
    float32_t f32_DeltaI;
    float32_t f32_TestVoltage;
    float32_t f32_PhaseSet;
    float32_t f32_StartCount;
    
    int16_t i16_SampleCount;
    float32_t f32_NextSampleTime;

    // Flux Online Calib
    float32_t f32_CalibFlux;
    float32_t f32_FluxEstRaw;
    float32_t f32_X1;
    float32_t f32_X2;
    float32_t f32_IalphaLast;
    float32_t f32_IbetaLast;

}CalibrationCtrl_t, *pCalibrationCtrl;

void Calibration_Init(pCalibrationCtrl pv);
void Calibration_Start(pCalibrationCtrl pv);
void Calibration_Loop(pCalibrationCtrl pv);
void MotorFlux_OnlineCalib(pCalibrationCtrl pv, AB_t Uab, AB_t Iab);

#endif // __CALIBRATION_H__