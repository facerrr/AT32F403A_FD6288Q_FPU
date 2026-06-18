#ifndef __TIMER_H__
#define __TIMER_H__

void OneMs_Define(void);
void delay_ms(const unsigned int ms);
void delay_us(const unsigned int us);


void BTimer_IRQHandler(void);
#endif // __TIMER_H__