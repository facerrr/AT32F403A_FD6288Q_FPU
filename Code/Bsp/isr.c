#include "config.h"
#include "isr.h"


void ADC1_2_IRQHandler(void){
    static uint8_t i;
    if (adc_flag_get(ADC2, ADC_PCCE_FLAG) != RESET){
        adc_flag_clear(ADC2, ADC_PCCE_FLAG);
        Adc_u16AdcResult[ADC_CH_IW] = (uint16_t)(ADC2->pdt1_bit.pdt1);
        Adc_u16AdcResult[ADC_CH_IV] = (uint16_t)(ADC2->pdt2_bit.pdt2);
        Adc_u16AdcResult[ADC_CH_IU] = (uint16_t)(ADC2->pdt3_bit.pdt3);
        if(i == 0){
            i = 1;
            gpio_bits_reset(TEST_PORT, TEST_PIN);
        }else{
            i = 0;
            gpio_bits_set(TEST_PORT, TEST_PIN);
        }

        if(MotorAdcSample.bAdcRdy == FALSE){
            Adc_CheckOffset(&MotorAdcSample);
        }else{
            Motor_MainIsrHandler();
        }
    }else{
        Motor_FastStop();
        MotorRunPara.u32FaultCode |= ERR_UNDEF_INT;
    }
}


void TMR3_GLOBAL_IRQHandler(void){
    if(tmr_interrupt_flag_get(TMR3, TMR_OVF_FLAG) != RESET){
        tmr_flag_clear(TMR3, TMR_OVF_FLAG);
        BTimer_IRQHandler();
        if (TRUE == MotorAdcSample.bAdcRdy) {
            Motor_SysTickerIsrHandler();
        }
    }
}


void TMR2_GLOBAL_IRQHandler(void){
    if(tmr_interrupt_flag_get(TMR2, TMR_OVF_FLAG) != RESET){
        tmr_flag_clear(TMR2, TMR_OVF_FLAG);
        
    }
}