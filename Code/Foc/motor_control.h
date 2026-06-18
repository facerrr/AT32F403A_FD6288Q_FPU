#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include "mc_type.h"
#include "startup.h"
#include "pi_calc.h"
#include "adc_sample.h"
#include "speed_set.h"
#include "transform.h"
#include "smo.h"
#include "hfi.h"
#include "flux.h"
#include "svpwm.h"
#include "stop_control.h"
#include "protection.h"
#include "filter.h"
#include "calibration.h"

typedef struct{
    volatile int32_t i32_1msCnt;
    int32_t i32_1msTime;
    volatile int32_t i32_50msCnt;
    int32_t i32_50msTime;
    volatile int32_t i32SpdRegCnt;
    int32_t i32SpdRegTime;
}MotorTimer_t, *pMotorTimer;


extern AdcSample_t MotorAdcSample;
extern MotorRunPara_t MotorRunPara;
extern SpeedCtrl_t MotorSpdCtrl;
extern CmdSpeedSet_t MotorCmdSpdSet;
extern FluxObserver_t MotorFluxObs;
extern SMObserver_t MotorSMObs;
extern HFI_t MotorHFI;
extern HFI2Observer_t MotorHfi2Obs;
extern Startup_t MotorStartup;
extern PIControl_t MotorIqPid;
extern PIControl_t MotorIdPid;
extern ProtectVbus_t MotorProtectVbus;
extern ProtectOC_t MotorProtectOC;
extern StopCtrl_t MotorStopCtrl;
extern CalibrationCtrl_t MotorCalibCtrl;

extern MotorTimer_t MotorTimer;
extern SVPWM_t MotorSvpwm;

void MotorHW_Init(void);
void MotorSW_Init(void);
void Motor_FastStop(void);
void Motor_CVLimit(void);
void Motor_SysTickerIsrHandler(void);
void Motor_MainIsrHandler(void);

#endif // __MOTOR_CONTROL_H__
