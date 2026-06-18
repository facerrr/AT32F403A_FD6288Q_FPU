#ifndef __STOP_CONTROL_H__
#define __STOP_CONTROL_H__

#include "mc_type.h"

typedef enum {
    Stop_Initialize = 1,
    Stop_DecCurrent = 2,
    Stop_ElecBrake  = 3,
    Stop_ShortBrake = 4,
    Stop_Finish     = 5,
}StopStatus_t;

typedef struct{
    StopStatus_t Status;
    uint8_t bEnBrakeStop;
    uint8_t u8ShortBrakeStatus;
    int32_t i32TimerCnt;
}StopCtrl_t, *pStopCtrl_t;

void Stop_Control(pStopCtrl_t pv);


#endif // __STOP_CONTROL_H__
