#ifndef __PROTECTION_H__
#define __PROTECTION_H__

#include "mc_type.h"

typedef struct {
    int32_t i32AbnormalCnt;
    int32_t i32OvpCnt;
    int32_t i32UvpCnt;
    int32_t i32AbnormalTime;
    int32_t i32OvpTime;
    int32_t i32UvpTime;
    int32_t i32ErrClrTime;
    float32_t f32_OvpThreshold;
    float32_t f32_UvpThreshold;
    float32_t f32_AbnormalHighVbus;
    float32_t f32_AbnormalLowVbus;
} ProtectVbus_t, *pProtectVbus;

typedef struct {
    int32_t i32PeakOcpCnt;
    int32_t i32PeakOcpTime;
    float32_t f32_PeakOcpThreshold;
    int32_t i32ErrClrTime;
} ProtectOC_t, *pProtectOC;

void Prot_MonitorVbus(float32_t f32_Vbus, pProtectVbus pv);
void Prot_MotorOverCurrent(UVW_t Iuvw, pProtectOC pv);

#endif // __PROTECTION_H__
