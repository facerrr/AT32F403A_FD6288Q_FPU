#include "config.h"


void SysClk_Init(void) {
    crm_reset();

    crm_clock_source_enable(CRM_CLOCK_SOURCE_LICK, TRUE);
    while(crm_flag_get(CRM_LICK_STABLE_FLAG) != SET);

    #if USE_HEXT == TRUE
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
    while(crm_hext_stable_wait() == ERROR)
    #endif
    
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HICK, TRUE);
    while(crm_flag_get(CRM_HICK_STABLE_FLAG) != SET);
    
    #if USE_HEXT == TRUE
    crm_pll_config(CRM_PLL_SOURCE_HEXT, CRM_PLL_MULT_15, CRM_PLL_OUTPUT_RANGE_GT72MHZ);
//    crm_hext_clock_div_set(CRM_HEXT_DIV_2);
    #else
    crm_pll_config(CRM_PLL_SOURCE_HICK, CRM_PLL_MULT_60, CRM_PLL_OUTPUT_RANGE_GT72MHZ);
    #endif

    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
    while(crm_flag_get(CRM_PLL_STABLE_FLAG) != SET);

    crm_ahb_div_set(CRM_AHB_DIV_1);
    crm_apb2_div_set(CRM_APB2_DIV_2);
    crm_apb1_div_set(CRM_APB1_DIV_2);

    crm_auto_step_mode_enable(TRUE);

    crm_sysclk_switch(CRM_SCLK_PLL);
    while(crm_sysclk_switch_status_get() != CRM_SCLK_PLL);

    crm_auto_step_mode_enable(FALSE);;
    system_core_clock_update();
}



void Clock_Config(void){
    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_UART7_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_ADC2_PERIPH_CLOCK, TRUE);
    crm_adc_clock_div_set(CRM_ADC_DIV_6);
    crm_periph_clock_enable(CRM_SPI1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_ACC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_USB_PERIPH_CLOCK, TRUE);
}