#ifndef __ADC_SAMPLE_H__
#define __ADC_SAMPLE_H__

#include "mc_type.h"

typedef struct{
    float32_t f32Fs;
    float32_t f32IuvwK;
    float32_t f32VbusK;
    float32_t f32DetectDelayTimeMs;
    float32_t f32OffsetCheckTimeMs;
    int32_t i32RefIuvwOffset;
    int32_t i32MaxIuvwOffsetBias;
}SampleCfg_t, *pSampleCfg;

typedef struct{
    uint8_t bAdcRdy;
    int32_t i32ShuntNum;
    int32_t i32AdcValue1;
    int32_t i32AdcValue2;
    int32_t i32OffsetDetectCnt;
    int32_t i32OffsetDetectTime;
    int32_t i32DelayTime;
    float32_t f32IuOffset;
    float32_t f32IvOffset;
    float32_t f32IwOffset;
    int32_t i32RefOffset;
    int32_t i32MaxOffsetBias;
    float32_t f32_VbusK;
    float32_t f32_IuvwK;
} AdcSample_t, *pAdcSample;

extern uint16_t Adc_u16AdcResult[16];

void Adc_InitSample(SampleCfg_t Cfg, pAdcSample pv);
void Adc_Sample(pAdcSample pv);
void Adc_CheckOffset(pAdcSample pv);

#endif // __ADC_SAMPLE_H__
