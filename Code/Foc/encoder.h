#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <stdbool.h>
#include "mc_type.h"

#define __AS5047         0
#define __MT6816         1
#define __MT6701         2

#define ENCODER_CPR         (int32_t)16384
#define ENCODER_CPR_F       (16384.0f)
#define ENCODER_CPR_DIV2    (ENCODER_CPR >> 1)

#define NCS_SET()           GPIOA->scr = GPIO_PINS_4
#define NCS_RESET()         GPIOA->clr = GPIO_PINS_4


typedef struct{

    bool b_NoMag;
    bool b_OverSpeed;
    uint32_t u32_Angle;

    uint8_t u8_RxErrCnt;
    uint8_t u8_CheckErrCnt;

}MT6816_t;



typedef struct{
    int32_t i32_Raw;
    int32_t i32_CntInCpr;
    int32_t i32_CntInCprPrev;

    int64_t i64_ShadowCnt;

    float32_t f32_EAngle;

    //pll
    float32_t f32_PosCprCnts;
    float32_t f32_VelEstCnts;

    float32_t f32_Pos;
    float32_t f32_Vel;

    float32_t f32_Phase;
    float32_t f32_PhaseVel;

    float32_t f32_PllKp;
    float32_t f32_PllKi;
    float32_t f32_Interpolation;
    float32_t f32_SnapThreshold;
    
    float32_t f32_Offset;

    int32_t i32_Dir;
}Encoder_t;

extern MT6816_t MT6816;
extern Encoder_t Encoder;

void Encoder_Init(void);
bool Encoder_Sample(void);
void Encoder_Loop(void);

#endif // __ENCODER_H__