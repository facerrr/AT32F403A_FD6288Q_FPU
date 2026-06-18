#ifndef __USR_CONFIG_H__
#define __USR_CONFIG_H__

#include "base_types.h"

#define USE_ENCODER         0x00
#define SMO_OBSERVER        0x01
#define FLUX_OBSERVER       0x02
#define HFI_OBSERVER        0x03
#define HFI_SMO_OBSERVER    0x04
#define HFI_FLUX_OBSERVER   0x05

#define ENCODER_TYPE    1



#define CARRIER_FREQUENCY       20000
#define CURRENT_MEASURE_HZ      CARRIER_FREQUENCY
#define CURRENT_MEASURE_PERIOD  (float) (1.0f / (float) CURRENT_MEASURE_HZ)

#define APB1CLK_HZ              240000000
#define TIMER_CLK_MHz           240
#define PWM_PERIOD_CYCLES       (uint16_t)((TIMER_CLK_MHz * (uint32_t) 1000000u / ((uint32_t) (CARRIER_FREQUENCY))) & 0xFFFE)
#define HALF_PWM_PERIOD_CYCLES  (uint16_t)(PWM_PERIOD_CYCLES / 2U)

#define MAX_DUTY_RATIO          (0.75f)

#define SPEED_SCALE             (2857)

#define MAX_START_IS            (1.5f)
#define MAX_NORMAL_IS           (2.0f)

typedef struct
{
    uint8_t PolePairs;
    float Rs;
    float Ls;
    float Flux;
    float Lq;
    float Ld;
}MotorPM_t;

typedef struct{
    float f32CarrierFreqHz;
    float f32BaseFreqHz;
    float f32SysTickerFreqHz;
    float f32MaxDutyRatio;
    float f32DeadTimeUs;

    // ADC Sampling
    float f32AdcTrigTimeUs;

    float f32IuvwSampleRs;
    float f32IuvwSampleK;
    float f32AdcCheckDelayMs;
    float f32OffsetCheckTimeMs;
    int32_t i32IuvwRefOffset;
    int32_t i32IuvwMaxOffsetBias;

    float f32VbusSampleK;

    // Motor Parameters
    float f32Rs;
    float f32Ls;
    float f32LambdaF;
    int32_t i32PolePairs;

    int32_t i32MinSpeedRpm;
    int32_t i32MaxSpeedRpm;
    
    // Feed-forward Compensation
    int32_t i32FeedforwardEnable;
    
    // Currents (A)
    float f32MaxNormaIs;
    float f32MaxStartIs;
    float f32MaxBrakeIs;

    // Protection
    float f32AbnormalHighVbus;
    float f32AbnormalLowVbus;
    int32_t i32AbnormalVbusTimeUs;

    float f32OverVoltageThold;
    int32_t i32OverVoltageTimeUs;

    float f32UnderVoltageThold;
    int32_t i32UnderVoltageTimeUs;

    // Over Current
    float f32PeakOverCurrThold;
    int32_t i32PeakOverCurrTimeUs;
    int32_t i32ErrClearTimeMs;

    // Speed Pid
    float f32WrKp;
    float f32WrKi;

    // Acceleration/Deceleration
    float f32WrRampRate;
    float f32AccPerSec;
    float f32DecPerSec;

    int32_t i32ObserverType; // 0: SMO, 1: FluxObs

    // Observer Parameters
    float f32FluxObsGamma;
    float f32FluxObsPllKp;
    float f32FluxObsPllKi;
    int32_t i32FluxUseAtan2;

    // SMO Parameters
    float f32SMOPllKp;
    float f32SMOPllKi;
    int32_t i32SMOFluxOnlineEnable;
    
    // HFI Parameters
    float f32HfiPllKp;
    float f32HfiPllKi;
    float f32HfiInjectMagMax;
    float f32HfiInjectMagMin;
    float f32Hfi2ObsWrMax;
    float f32Hfi2ObsWrMin;
    float f32Hfi2ObsWrMid;
    
    // Control Period Setting
    float f32SpdRegPeriodMs;

    // Startup Control Parameters
    float f32ObsStableTimeMs;
    float f32MinObserableWr;

    float f32ObsConvergeMaxTimeMs;
    float f32TransTimeMaxMs;
    
    float f32AlignCurrent;
    float f32_1stAlignTimeMs;
    float f32_2ndAlignTimeMs;
    float f32_3rdAlignTimeMs;
    float f32_2ndAlignFwdTheta;

    float f32ForceCurrentSlop;
    float f32ForceAccRate;
    float f32MaxForceSpd;
    
    // Stop Control
    int32_t i32BreakEnable;

    // Calibration
    float f32CalibVoltage;
    float f32CalibCurrent;


}MotorCfg_t;

extern const MotorPM_t MotorPM;
extern MotorCfg_t MotorCfg;

void MotorCfg_Init(void);

#endif // __USR_CONFIG_H__