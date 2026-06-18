#include "config.h"

PIControl_t PI_Id;
PIControl_t PI_Iq;
PIControl_t PI_Spd;

float32_t f32_VK = 0.9f * 0.866025403784f; // 0.9 * SQRT3_BY_2

float32_t PID_IncPid(float32_t f32_Ref, float32_t f32_Fbk, pPIControl pv) {
    float32_t f32_Err = f32_Ref - f32_Fbk;
    
    f32_Err = CLAMP(f32_Err, -pv->f32_AbsErrMax, pv->f32_AbsErrMax);
    pv->f32_Output += pv->f32_Kp * (f32_Err - pv->f32_Errl);
    pv->f32_Output += pv->f32_Ki * f32_Err;

    if(pv->f32_Output > pv->f32_OutMax){
        pv->f32_Output = pv->f32_OutMax;
    }else if(pv->f32_Output < pv->f32_OutMin){
        pv->f32_Output = pv->f32_OutMin;
    }

    pv->f32_Errl = f32_Err;
    return pv->f32_Output;
}

float32_t PID_PosPid(float32_t f32_Ref, float32_t f32_Fbk, pPIControl pv) {
    float32_t f32_Err = f32_Ref - f32_Fbk;
    
    pv->f32_Output = pv->f32_Kp * f32_Err;
    pv->f32_Output += pv->f32_IOut;
    
    uint8_t limited = 0;
    
    if(pv->f32_Output > pv->f32_OutMax){
        pv->f32_Output = pv->f32_OutMax;
        limited = 1;
    }else if(pv->f32_Output < pv->f32_OutMin){
        pv->f32_Output = pv->f32_OutMin;
        limited = 1;
    }
    
    if(limited){
        pv->f32_IOut *= 0.98;
    }else{
        pv->f32_IOut += pv->f32_Ki * f32_Err;
    }
    
//    f32_Err = CLAMP(f32_Err, -pv->f32_AbsErrMax, pv->f32_AbsErrMax);
//    
//    pv->f32_IOut += pv->f32_Ki * f32_Err;
//    if(pv->f32_IOut > pv->f32_OutMax){
//        pv->f32_IOut = pv->f32_OutMax;
//        pv->f32_IOut *= 0.984375f;
//    }else if(pv->f32_IOut < pv->f32_OutMin){
//        pv->f32_IOut = pv->f32_OutMin;
//    }

//    pv->f32_Output = pv->f32_Kp * f32_Err + pv->f32_IOut;
//    if(pv->f32_Output > pv->f32_OutMax){
//        pv->f32_Output = pv->f32_OutMax;
//    }else if(pv->f32_Output < pv->f32_OutMin){
//        pv->f32_Output = pv->f32_OutMin;
//    }

//    pv->f32_Errl = f32_Err;
    return pv->f32_Output;
}

/**
 * @brief Clear PI control parameters
 * @param pv Pointer to the PIControl structure
 */
void PID_Clear(pPIControl pv) {
    pv->f32_Output = 0.0f;
    pv->f32_IOut = 0.0f;
    pv->f32_Errl = 0.0f;
}