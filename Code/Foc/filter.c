#include "filter.h"

float32_t Filter_FirstLpf(float32_t f32_Input, pLPFilter pv) {
    float32_t f32_temp = (f32_Input + pv->f32_Input) * 0.5f - pv->f32_Output;
    pv->f32_Output += pv->f32_LpfK * f32_temp;
    pv->f32_Input = f32_Input;

    return pv->f32_Output;
}

float32_t Filter_PeriodAvg(float32_t f32_Input, pPeriodAvg pv) {
    pv->f32_Sum += f32_Input;
    pv->f32_Cnt += pv->f32_InvTotalCnt;
    if(pv->f32_Cnt >= 1.0f){
        pv->f32_Avg = pv->f32_Sum * pv->f32_InvTotalCnt;
        pv->f32_Sum = 0.0f;
        pv->f32_Cnt = 0.0f; // Alternatively f32_Cnt -= 1.0f to reduce jitter
    }
    return pv->f32_Avg;
}

void MoveFilter_Init(pMoveFilter pv, int32_t bufSize, int8_t BitSize){
    int32_t i;
    if(bufSize > MOVE_FILTER_BUF_MAXSIZE){
        bufSize = MOVE_FILTER_BUF_MAXSIZE;
        pv->BufferBitSize = MOVE_FILTER_BUF_BITSIZE;
    } else {
        pv->BufferBitSize = BitSize;
    }
    pv->BufferSize = bufSize;
    for(i = 0; i < pv->BufferSize; i++){
        pv->pBuffer[i] = 0.0f;
    }
}

void MoveFilter_Reset(pMoveFilter pv){
    int32_t i = 0;
    pv->BufferFilled = 0;
    pv->BufferIndex = 0;
    pv->f32_BufSum = 0.0f;
    pv->f32_Val = 0.0f;

    for(i = 0; i < pv->BufferSize; i++){
        pv->pBuffer[i] = 0.0f;
    }
}

void MoveFilter_Fill(pMoveFilter pv, float32_t val){
    pv->f32_BufSum -= pv->pBuffer[pv->BufferIndex];
    pv->pBuffer[pv->BufferIndex] = val;
    pv->f32_BufSum += pv->pBuffer[pv->BufferIndex];
    pv->BufferIndex++;
    if(pv->BufferIndex >= pv->BufferSize){
        pv->BufferIndex = 0;
    }
    if(pv->BufferFilled < pv->BufferSize){
        pv->BufferFilled++;
    }
}

float32_t MoveFilter_Calc(pMoveFilter pv){
    float32_t f32_Val;
    if(pv->BufferFilled == 0) return 0.0f;
    if(pv->BufferFilled < pv->BufferSize){
        f32_Val = pv->f32_BufSum / (float32_t)pv->BufferFilled;
    } else {
        f32_Val = pv->f32_BufSum / (float32_t)pv->BufferSize;
    }
    pv->f32_Val = f32_Val;
    return f32_Val;
}
