#include "config.h"

void VFControl_Init(pVFControl pv){
    memset((void*)pv, 0, sizeof(VFControl_t));
    pv->Uqd.f32_Q = 1.5f; // 1.5V as arbitrary safe initial Vf voltage
    pv->Uqd.f32_D = 0.0f;
    pv->f32_K = 0.099312f;
}

/**
 * @brief VF angle calculation
 * @param pv Pointer to the VFControl_t structure
 */
void VFAngle_Calc(pVFControl pv){
    pv->f32_Step = pv->f32_K * (pv->Uqd.f32_Q + pv->Uqd.f32_D);
    pv->f32_EAngle += pv->f32_Step;
    while(pv->f32_EAngle >= (2.0f * M_PI)) { pv->f32_EAngle -= (2.0f * M_PI); }
    while(pv->f32_EAngle < 0.0f) { pv->f32_EAngle += (2.0f * M_PI); }
}


/**
 * @brief VF speed calculation
 * @param pv Pointer to the VFControl_t structure
 */
void VFSpeed_Calc(pVFControl pv){
    pv->f32_Speed = pv->f32_Step / (2.0f * M_PI) * MotorCfg.f32CarrierFreqHz * 60.0f / MotorCfg.i32PolePairs;
}
