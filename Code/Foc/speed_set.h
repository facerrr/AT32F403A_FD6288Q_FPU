#ifndef __SPEED_SET__
#define __SPEED_SET__

#include "mc_type.h"

typedef struct{
    int32_t i32CmdRpm;
    int32_t i32RealRpm;
    int32_t i32PreCmdRpm;

    int32_t i32MinRpm;
    int32_t i32MaxRpm;
    float32_t f32_Rpm2Wr;
    float32_t f32_Wr2Rpm;
}CmdSpeedSet_t, *pCmdSpeedSet;


typedef struct
{
    float32_t f32_TgtWr;
    float32_t f32_WrAccRate;
    float32_t f32_WrDecRate;
    float32_t f32_WrMaxStep;

    float32_t f32_TgtTorque;

    float32_t f32_MinWr;
    float32_t f32_MaxWr;
    float32_t f32_Output;

    float32_t f32_WrErr;
    float32_t f32_WrKp;
    float32_t f32_WrKi;
    float32_t f32_WrISum;
    float32_t f32_WrOutMax;

}SpeedCtrl_t, *pSpeedCtrl;


typedef struct
{
    float32_t f32VelSet;
    float32_t f32VelKp;
    float32_t f32VelKi;
    float32_t f32VelIsum;
    float32_t f32VelOutput;
    float32_t f32TorqueSet;
}SpeedFCtrl_t, *pSpeedFCtrl;

void Speed_InitCmdSpdSet(pCmdSpeedSet pv);
void Speed_InitSpdTraj(pSpeedCtrl pv);
void Speed_CmdSpdSet(pCmdSpeedSet pv);
float32_t Speed_TgtSpdTraj(float32_t f32_CmdWr, pSpeedCtrl pv);
float32_t SpeedSet_Update(pSpeedCtrl pv, float32_t f32_CmdWr);
float32_t SpeedCtrl_Update(int32_t i32Rpm);

#endif // __SPEED_SET__
