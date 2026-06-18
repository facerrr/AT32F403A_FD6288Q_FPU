#include "config.h"
#include "protection.h"

void Prot_MonitorVbus(float32_t f32_Vbus, pProtectVbus pv) {
    // Over Voltage Protection
    if (f32_Vbus >= pv->f32_OvpThreshold) {
        if (pv->i32OvpCnt < pv->i32OvpTime) {
            pv->i32OvpCnt++;
        } else {
            MotorRunPara.u32FaultCode |= ERR_VDC_OV;
        }
    } else {
        if (pv->i32OvpCnt > 0) {
            pv->i32OvpCnt--;
        } else {
            MotorRunPara.u32FaultCode &= ~ERR_VDC_OV;
        }
    }

    // Under Voltage Protection
    if (f32_Vbus <= pv->f32_UvpThreshold) {
        if (pv->i32UvpCnt < pv->i32UvpTime) {
            pv->i32UvpCnt++;
        } else {
            MotorRunPara.u32FaultCode |= ERR_VDC_UV;
        }
    } else {
        if (pv->i32UvpCnt > 0) {
            pv->i32UvpCnt--;
        } else {
            MotorRunPara.u32FaultCode &= ~ERR_VDC_UV;
        }
    }

    // Abnormal Voltage Detection
    if ((f32_Vbus >= pv->f32_AbnormalHighVbus) || (f32_Vbus <= pv->f32_AbnormalLowVbus)) {
        if (pv->i32AbnormalCnt < pv->i32AbnormalTime) {
            pv->i32AbnormalCnt++;
        } else {
            MotorRunPara.u32FaultCode |= ERR_VDC_ABNORM;
        }
    } else {
        if (pv->i32AbnormalCnt > 0) {
            pv->i32AbnormalCnt--;
        } else {
            MotorRunPara.u32FaultCode &= ~ERR_VDC_ABNORM;
        }
    }
}

void Prot_MotorOverCurrent(UVW_t Iuvw, pProtectOC pv) {
    // Currently empty in QMath implementation, keeping signature for alignment
    float32_t f32_DeltaU = Iuvw.f32_U - pv->f32_PeakOcpThreshold;
    float32_t f32_DeltaV = Iuvw.f32_V - pv->f32_PeakOcpThreshold;
    float32_t f32_DeltaW = Iuvw.f32_W - pv->f32_PeakOcpThreshold;

    if((MotorRunPara.u32FaultCode & ERR_OC_PEAK) == 0U){
        if(f32_DeltaU > 0.0f || f32_DeltaV > 0.0f || f32_DeltaW > 0.0f){
            pv->i32PeakOcpCnt++;
            if(pv->i32PeakOcpCnt >= pv->i32PeakOcpTime){
                MotorRunPara.u32FaultCode |= ERR_OC_PEAK;
                pv->i32PeakOcpCnt = pv->i32ErrClrTime;
            }
        } else {
            pv->i32PeakOcpCnt = 0;
        }
    }else{
        if(f32_DeltaU <= 0.0f && f32_DeltaV <= 0.0f && f32_DeltaW <= 0.0f){
            pv->i32PeakOcpCnt--;
            if(pv->i32PeakOcpCnt <= 0){ 
                MotorRunPara.u32FaultCode &= ~ERR_OC_PEAK;
            }
        }else{
            pv->i32PeakOcpCnt = pv->i32ErrClrTime;
        }
    }
}
