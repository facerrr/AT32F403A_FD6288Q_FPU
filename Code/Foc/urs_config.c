#include "config.h"

#define M55                     0
#define LODA                    1
#define M2312A                  2
#define W48_22                  3  
#define ATK                     4
#define E2804                   5
#define GIM6010                 6

#define SELECTED_MOTOR          W48_22

const MotorPM_t MotorPM;
MotorCfg_t MotorCfg;

#if SELECTED_MOTOR == M55
const MotorPM_t MotorPM = {
    .PolePairs = 1,
    .Rs = 0.0247350615f,
    .Flux = 0.00104845f,
    .Lq = 0.00001204f,
    .Ld = 0.00001148f,
    .Ls = 0.0000104953f,
};
#elif SELECTED_MOTOR == LODA
const MotorPM_t MotorPM = {
    .PolePairs = 7,
    .Rs = 0.315297991f,
    .Ls = 0.0000695712f,
    .Flux = 0.00410514f,
};
#elif SELECTED_MOTOR == M2312A
const MotorPM_t MotorPM = {
    .PolePairs = 7,
    .Rs = 0.106162012f,
    .Ls = 0.0000224214f,
    .Flux = 0.00099018f,
};
#elif SELECTED_MOTOR == W48_22
const MotorPM_t MotorPM = {
    .PolePairs = 11,
    .Rs = 0.0971951857f,
    .Ls = 0.0000317365f,
    .Flux = 0.00123515f,
    // .Flux = 0.00245f
};
#elif SELECTED_MOTOR == ATK
const MotorPM_t MotorPM = {
    .PolePairs = 2,
    .Rs = 0.4f,
    .Ls = 0.000772f,
    .Flux = 0.03151268f,
};
#elif SELECTED_MOTOR == E2804
const MotorPM_t MotorPM = {
    .PolePairs = 2,
    .Rs = 4.8488555f,
    .Ls = 0.000772f,
    .Flux = 0.00560226f,
};
#elif SELECTED_MOTOR == GIM6010
const MotorPM_t MotorPM = {
    .PolePairs = 14,
    .Rs = 0.229342341f,
    .Ls = 0.000113911272f,
    .Flux = 0.00386174046f,
};
#endif


void MotorCfg_Init(void){
    
    MotorCfg.i32PolePairs = MotorPM.PolePairs;

    // Use real physical units directly (Ohms, Henries, Webers)
    MotorCfg.f32Rs = MotorPM.Rs;                                    // (Ohm) Motor phase resistance
    MotorCfg.f32Ls = MotorPM.Ls;                                    // (H) Motor phase inductance           
    MotorCfg.f32LambdaF = MotorPM.Flux;                             // (Wb) Motor flux linkage

    MotorCfg.f32CarrierFreqHz = (float)CARRIER_FREQUENCY;           // (Hz) PWM carrier frequency
    MotorCfg.f32BaseFreqHz = 500.0f;                                // (Hz) Base frequency
    MotorCfg.f32SysTickerFreqHz = 4000.0f;                          // (Hz) System ticker frequency
    MotorCfg.f32MaxDutyRatio = 0.75f;                               // SVPWM Max duty ratio (0.0 ~ 1.0)
    MotorCfg.f32DeadTimeUs = 1.0f;                                  // (us)

    // Adc Sample
    MotorCfg.f32AdcTrigTimeUs = 1.0f;                               // (us)
    MotorCfg.f32IuvwSampleRs = 0.001f;                              // (Ohm) Current sampling resistor
    MotorCfg.f32IuvwSampleK = 30.3f;                                // (A/V) Current sampling gain (1 / (Rs * ADC_Vref))
    MotorCfg.f32AdcCheckDelayMs = 50.0f;                            // (ms) Delay time after ADC trigger to check ADC value
    MotorCfg.f32OffsetCheckTimeMs = 50.0f;                          // (ms) Delay time to check offset values
    MotorCfg.i32IuvwRefOffset = 2048;                               // ADC reference offset for current sampling
    MotorCfg.i32IuvwMaxOffsetBias = 80;                             // ADC max offset bias for current sampling

    MotorCfg.f32VbusSampleK = 19.0f;                                // (V/V) Vbus sampling gain (Vbus / ADC voltage)

    // Speed Ranges (RPM)
    MotorCfg.i32MinSpeedRpm = 800;                                  // (RPM) Minimum speed 
    MotorCfg.i32MaxSpeedRpm = 5000;                                 // (RPM) Maximum speed

    // Currents (A)
    MotorCfg.f32MaxNormaIs = 5.0f;                                 // (A) Maximum normal operating current
    MotorCfg.f32MaxStartIs = 1.5f;                                  // (A) Maximum starting current
    MotorCfg.f32MaxBrakeIs = 0.8f;                                  // (A) Maximum braking current

    // Accel/Decel
    MotorCfg.f32WrRampRate = 100.0f;                                // (/s) Speed ramp rate
    MotorCfg.f32AccPerSec = 1000.0f;
    MotorCfg.f32DecPerSec = 1000.0f;

    MotorCfg.i32ObserverType = SMO_OBSERVER;
    // Observer
    MotorCfg.f32FluxObsGamma = 20000000.0f;
    MotorCfg.f32FluxObsPllKp = 1036.0f;
    MotorCfg.f32FluxObsPllKi = 1000000.0f;
    MotorCfg.i32FluxUseAtan2 = FALSE;
    
    MotorCfg.f32SMOPllKp = 0.122f;
    MotorCfg.f32SMOPllKi = 0.031f;
    MotorCfg.i32SMOFluxOnlineEnable = TRUE;

    MotorCfg.f32ObsStableTimeMs = 100.0f;
    MotorCfg.f32MinObserableWr = 420.0f;

    // Align Current (A)
    MotorCfg.f32AlignCurrent = 1.0f;
    MotorCfg.f32_1stAlignTimeMs = 50.0f;
    MotorCfg.f32_2ndAlignTimeMs = 50.0f;
    MotorCfg.f32_3rdAlignTimeMs = 50.0f;
    MotorCfg.f32_2ndAlignFwdTheta = 0.0f;

    MotorCfg.f32ForceCurrentSlop = 10.0f; // A/s
    MotorCfg.f32ForceAccRate = 200.0f;
    MotorCfg.f32MaxForceSpd = 400.0f;
    
    MotorCfg.i32BreakEnable = FALSE;

    MotorCfg.f32SpdRegPeriodMs = 1.0f;

    // Abnormal Vbus (V)
    MotorCfg.f32AbnormalHighVbus = 28.0f;
    MotorCfg.f32AbnormalLowVbus = 6.0f;
    MotorCfg.i32AbnormalVbusTimeUs = 1000;
    // Over voltage (V)
    MotorCfg.f32OverVoltageThold = 26.0f;
    MotorCfg.i32OverVoltageTimeUs = 1000;
    // Under voltage (V)
    MotorCfg.f32UnderVoltageThold = 8.0f;
    MotorCfg.i32UnderVoltageTimeUs = 1000;
    // Over current (A)
    MotorCfg.f32PeakOverCurrThold = 10.0f;
    MotorCfg.i32PeakOverCurrTimeUs = 500;
    MotorCfg.i32ErrClearTimeMs = 5000;

    // Speed PID
    MotorCfg.f32WrKp = 0.01f;
    MotorCfg.f32WrKi = 0.058f;
    
    MotorCfg.i32FeedforwardEnable = 0;

    MotorCfg.f32CalibCurrent = 3.0f;
    MotorCfg.f32CalibVoltage = 2.0f;
}
