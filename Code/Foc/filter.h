#ifndef __FILTER_H__
#define __FILTER_H__

#include "base_types.h"

#define MOVE_FILTER_BUF_MAXSIZE     512
#define MOVE_FILTER_BUF_BITSIZE     9

typedef struct
{
    float32_t f32_LpfK;
    float32_t f32_Output;
    float32_t f32_Input;
}LPFilter_t, *pLPFilter;

typedef struct
{
   float32_t f32_Avg;
   float32_t f32_Sum;
   float32_t f32_Cnt;
   float32_t f32_InvTotalCnt;
}PeriodAvg_t, *pPeriodAvg;

typedef struct{
    int32_t BufferSize;
    int32_t BufferIndex;
    int32_t BufferFilled;
    float32_t f32_BufSum;
    float32_t f32_Val;
    int8_t BufferBitSize;
    float32_t* pBuffer;
}MoveFilter_t, *pMoveFilter;


float32_t Filter_FirstLpf(float32_t f32_Input, pLPFilter pv);
float32_t Filter_PeriodAvg(float32_t f32_Input, pPeriodAvg pv);

void MoveFilter_Init(pMoveFilter pv, int32_t bufSize, int8_t BitSize);
void MoveFilter_Reset(pMoveFilter pv);
void MoveFilter_Fill(pMoveFilter pv, float32_t val);
float32_t MoveFilter_Calc(pMoveFilter pv);

#endif // __FILTER_H__