#include "timer.h"
#include "at32f403a_407.h"

uint8_t u8BaseTimUevFlag;
uint32_t u32TimeTickCnt;
uint8_t u8Time1ms;
uint8_t u8Time100ms;
uint32_t u32Time1s;
uint32_t u32SysTick1ms;

uint32_t _NOP_MS;

void tick1ms(void);
void tick100ms(void);
void tick1s(void);

/**
 * @brief  1ms Function
 * @param  None
 * @retval None
 */
void tick1ms(void){
    
    
}

/**
 * @brief  100ms Function
 * @param  None
 * @retval None
 */
void tick100ms(void){
}

/**
 * @brief  1s Function
 * @param  None
 * @retval None
 */
void tick1s(void){
}



/**
 * @brief  1MS Define
 * @param  None
 * @retval None
 */
void OneMs_Define(void){
    while(!u8BaseTimUevFlag){
    }
    u8BaseTimUevFlag = 0;
    _NOP_MS = 0;
    while(!u8BaseTimUevFlag){
        __NOP();
        _NOP_MS++;
    }
    u8BaseTimUevFlag = 0;
    _NOP_MS = _NOP_MS * 2 * 0.95;
}


/**
 * @brief  1ms延时函数
 * @param  ms: 延时时间
 * @retval None
 */
void delay_ms(const unsigned int ms)
{
    volatile uint32_t i = ms * _NOP_MS;
    while (i-- > 0) {
        __NOP();
    }
}

/**
 * @brief  1us延时函数
 * @param  us: 延时时间
 * @retval None
 */
void delay_us(const unsigned int us){
    volatile uint32_t i = us * _NOP_MS / 1000;
    while (i-- > 0) {
        __NOP();
    }
}


void BTimer_IRQHandler(void){
    
    u32TimeTickCnt++;
    if(u32TimeTickCnt >= 4){
        u32TimeTickCnt = 0;
        u8BaseTimUevFlag = 1;
        u8Time1ms++;
        tick1ms();
        if(u8Time1ms >= 100){
            u8Time1ms = 0;
            u8Time100ms++;
            tick100ms();
            if(u8Time100ms >= 10){
                u8Time100ms = 0;
                u32Time1s++;
                tick1s();
            }
        }
    }
}

