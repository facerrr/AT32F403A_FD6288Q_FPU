#ifndef __STARTUP_H__
#define __STARTUP_H__

#include "mc_type.h"

typedef enum{
    Startup_Initialize          = 0,
    Startup_BscCharge           = 1,
    Startup_IPD                 = 2,
    Startup_Align               = 3,
    Startup_StateDetect         = 4,
    Startup_HeadWindFocBrake    = 5,
    Startup_HeadWindShortBrake  = 6,
    Startup_HeadWindForceDrive  = 7,
    Startup_CLStart             = 10,
    Startup_ForceDrive          = 11,
    Startup_VfDrive             = 12,
    Startup_SoftSwitch          = 13,
    Startup_Finish              = 14,
}StartupStatus_t;


typedef struct {
    StartupStatus_t status;
    int8_t u8Status_StateDetect;
    int8_t u8Status_Align;
    int8_t u8Status_HeadWindShortBrake;
    int32_t i32TimerCnt;
    float32_t f32_Ts;
    float32_t f32_MaxStartIq;
    float32_t f32_MaxNormaIq;
    float32_t f32_LowSpdUpper;
    float32_t f32_LowSpdLower;
    int32_t i32ObsStableTime;
    float32_t f32_ObserableWr;
    float32_t f32_ForceIq;
    float32_t f32_ForceWr;

    float32_t f32_AlignIq;
    float32_t f32_AlignDIq;
    float32_t f32_AlignDTheta;
    int32_t i32Align1stTime;
    int32_t i32Align2ndTime;
    int32_t i32Align3rdTime;

    float32_t f32_ForceMaxIq;
    float32_t f32_ForceDIq;
    float32_t f32_ForceMaxWr;
    float32_t f32_ForceAccRate;

    uint16_t u16_TransCnt; 
    uint16_t u16_TransTimeMax;
    uint16_t u16_ConvergeTimeMax;

    float32_t f32_AngleDiffInit;

    /** short-brake parameters ************************************************/
    int32_t i32MinShortBrakeTime;
    int32_t i32MaxShortBrakeTime;
    float32_t f32_ShortBrakeEndSqrIs;

    float32_t f32_HeadWindAccRate;
    float32_t f32_HeadWindForceWr;
    float32_t f32_HeadWindIs;
} Startup_t;

void Startup_Init(Startup_t *pv);
void Startup_Control(Startup_t *pv);

#endif // __STARTUP_H__