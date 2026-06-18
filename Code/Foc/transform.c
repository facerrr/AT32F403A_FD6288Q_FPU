#include "config.h"

void CLARKE_Calc(pMotorRunPara pv) {
    pv->Iab.f32_Alpha = pv->Iuvw.f32_U;
    pv->Iab.f32_Beta = (INV_SQRT3 * pv->Iuvw.f32_U) + (TWO_INV_SQRT3 * pv->Iuvw.f32_V);
}

void PARK_Calc(pMotorRunPara pv) {
    pv->Iqd.f32_D = (pv->Iab.f32_Alpha * pv->f32_Cos) + (pv->Iab.f32_Beta * pv->f32_Sin);
    pv->Iqd.f32_Q = (pv->Iab.f32_Beta * pv->f32_Cos) - (pv->Iab.f32_Alpha * pv->f32_Sin);
}

void IPARK_Calc(pMotorRunPara pv) {
    pv->Uab.f32_Alpha = (pv->UqdRef.f32_D * pv->f32_Cos) - (pv->UqdRef.f32_Q * pv->f32_Sin);
    pv->Uab.f32_Beta = (pv->UqdRef.f32_Q * pv->f32_Cos) + (pv->UqdRef.f32_D * pv->f32_Sin);
}