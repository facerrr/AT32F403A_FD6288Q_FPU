#include "config.h"
#include "hardware_config.h"
#include "adc_sample.h"
#include "math.h"

uint16_t Adc_u16AdcResult[16];

void Adc_InitSample(SampleCfg_t Cfg, pAdcSample pv) {
    float32_t f32Temp;
    
    // Calculate simple float multipliers instead of using Q-Format
    pv->f32_IuvwK = ADC_VOLT_REF * Cfg.f32IuvwK / 4096.0f;
    pv->f32_VbusK = ADC_VOLT_REF * Cfg.f32VbusK / 4096.0f;
    
    pv->i32RefOffset = Cfg.i32RefIuvwOffset;
    pv->i32MaxOffsetBias = Cfg.i32MaxIuvwOffsetBias;
    
    f32Temp = (Cfg.f32DetectDelayTimeMs * Cfg.f32Fs / 1000.0f);
    pv->i32DelayTime = (int32_t)f32Temp;
    
    f32Temp = (Cfg.f32OffsetCheckTimeMs * Cfg.f32Fs / 1000.0f);
    pv->i32OffsetDetectTime = (int32_t)f32Temp;
    
    pv->f32IuOffset = 0.0f;
    pv->f32IvOffset = 0.0f;
    pv->f32IwOffset = 0.0f;
    pv->i32OffsetDetectCnt = 0;
    pv->bAdcRdy = FALSE;
}

void Adc_Sample(pAdcSample pv){
    MotorRunPara.f32_Vbus = (pv->f32_VbusK * (float32_t)Adc_u16AdcResult[ADC_CH_VDC]);
    
//    MotorRunPara.Iuvw.f32_U = (pv->f32_IuvwK * ((float32_t)Adc_u16AdcResult[ADC_CH_IU] - pv->f32IuOffset));
//    MotorRunPara.Iuvw.f32_V = (pv->f32_IuvwK * ((float32_t)Adc_u16AdcResult[ADC_CH_IV] - pv->f32IvOffset));
//    MotorRunPara.Iuvw.f32_W = (pv->f32_IuvwK * ((float32_t)Adc_u16AdcResult[ADC_CH_IW] - pv->f32IwOffset));
    
    if(MotorRunPara.i32TgtWrDir == ROTOR_DIR_CCW){
        MotorRunPara.Iuvw.f32_U = (pv->f32_IuvwK * ((float32_t)Adc_u16AdcResult[ADC_CH_IU] - pv->f32IuOffset));
        MotorRunPara.Iuvw.f32_V = (pv->f32_IuvwK * ((float32_t)Adc_u16AdcResult[ADC_CH_IV] - pv->f32IvOffset));
        MotorRunPara.Iuvw.f32_W = (pv->f32_IuvwK * ((float32_t)Adc_u16AdcResult[ADC_CH_IW] - pv->f32IwOffset));
    }else{
        MotorRunPara.Iuvw.f32_V = (pv->f32_IuvwK * ((float32_t)Adc_u16AdcResult[ADC_CH_IU] - pv->f32IuOffset));
        MotorRunPara.Iuvw.f32_U = (pv->f32_IuvwK * ((float32_t)Adc_u16AdcResult[ADC_CH_IV] - pv->f32IvOffset));
        MotorRunPara.Iuvw.f32_W = (pv->f32_IuvwK * ((float32_t)Adc_u16AdcResult[ADC_CH_IW] - pv->f32IwOffset));
    }
}

void Adc_CheckOffset(pAdcSample pv){
    MotorRunPara.f32_Vbus = (pv->f32_VbusK * (float32_t)Adc_u16AdcResult[ADC_CH_VDC]);

    if(pv->i32DelayTime > 0){
        pv->i32DelayTime--;
        return;
    }

    if(pv->i32OffsetDetectCnt < pv->i32OffsetDetectTime){
        pv->f32IuOffset += (float32_t)Adc_u16AdcResult[ADC_CH_IU];
        pv->f32IvOffset += (float32_t)Adc_u16AdcResult[ADC_CH_IV];
        pv->f32IwOffset += (float32_t)Adc_u16AdcResult[ADC_CH_IW];
        pv->i32OffsetDetectCnt++;
    }else{
        pv->f32IuOffset = pv->f32IuOffset / (float32_t)pv->i32OffsetDetectCnt;
        pv->f32IvOffset = pv->f32IvOffset / (float32_t)pv->i32OffsetDetectCnt;
        pv->f32IwOffset = pv->f32IwOffset / (float32_t)pv->i32OffsetDetectCnt;

        float32_t f32_TempU = fabsf(pv->f32IuOffset - (float32_t)pv->i32RefOffset);
        float32_t f32_TempV = fabsf(pv->f32IvOffset - (float32_t)pv->i32RefOffset);
        float32_t f32_TempW = fabsf(pv->f32IwOffset - (float32_t)pv->i32RefOffset);

        if ((f32_TempU > (float32_t)pv->i32MaxOffsetBias) || (f32_TempV > (float32_t)pv->i32MaxOffsetBias)){
            MotorRunPara.u32FaultCode |= ERR_AD_OFFSET;
        } else {
            pv->bAdcRdy = TRUE;
        }
    }
}
