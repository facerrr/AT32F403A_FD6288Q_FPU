#ifndef __INIT_MCU_H__
#define __INIT_MCU_H__

#include "hardware_config.h"

void InitMcu_Clock(void);
void InitMcu_SystemTicker(int32_t i32FreqHz);
void InitMcu_BaseTimer(s32 i32FreqHz);
void InitMcu_MotorTimer(u16 u16PeakCnt, u16 u16DeadTimeCnt, u16 u16AdcTrigCnt);
void InitMcu_Adc(void);
void InitMcu_UpdateMotorPwmDuty(u16 u16_Occr0, u16 u16_Occr1, u16 u16_Occr2);
void InitMcu_EnMotorPwm(boolean_t bEn);
void InitMcu_Gpio(void);
void InitMcu_StartAdc1Sample(void);

#endif // __INIT_MCU_H__