#include "config.h"
#include "init_mcu.h"

void InitMcu_Clock(void){
    crm_reset();

    crm_clock_source_enable(CRM_CLOCK_SOURCE_LICK, TRUE);
    while(crm_flag_get(CRM_LICK_STABLE_FLAG) != SET);

    #if CLK_SOURCE == CLK_USE_XTH
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
    while(crm_hext_stable_wait() == ERROR)
    #endif
    
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HICK, TRUE);
    while(crm_flag_get(CRM_HICK_STABLE_FLAG) != SET);

    #if CLK_SOURCE == CLK_USE_XTH
    crm_pll_config(CRM_PLL_SOURCE_HEXT, CRM_PLL_MULT_15, CRM_PLL_OUTPUT_RANGE_GT72MHZ);
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


void InitMcu_SystemTicker(int32_t i32FreqHz){
    s32 period = (s32)(4000000 / i32FreqHz) - 1U;

    tmr_base_init(TMR3, period, 60U - 1U);
    tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);
    tmr_clock_source_div_set(TMR3, TMR_CLOCK_DIV1);
    tmr_repetition_counter_set(TMR3, 0);
    
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(TMR3_GLOBAL_IRQn, 13, 0);

    tmr_interrupt_enable(TMR3, TMR_OVF_INT, TRUE);
    tmr_counter_enable(TMR3, TRUE);
}


void InitMcu_BaseTimer(s32 i32FreqHz){
    s32 div = 240;
    s32 period = (s32)(240000000 / div / i32FreqHz) - 1U;

    tmr_base_init(TMR2, period, div - 1U);
    tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
    tmr_clock_source_div_set(TMR2, TMR_CLOCK_DIV1);
    tmr_repetition_counter_set(TMR2, 0);
    
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(TMR2_GLOBAL_IRQn, 15, 0);

    tmr_interrupt_enable(TMR2, TMR_OVF_INT, TRUE);
    tmr_counter_enable(TMR2, TRUE);
}


void InitMcu_Gpio(void){
    gpio_pin_remap_config(SWJTAG_GMUX_010, TRUE);
    // LED
    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);

    /* configure the LED pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_pins = LEDRUN_PIN;
    gpio_init(LEDRUN_GPIO_PORT, &gpio_init_struct);
    
    gpio_init_struct.gpio_pins = LEDERR_PIN;
    gpio_init(LEDERR_GPIO_PORT, &gpio_init_struct);
    
    gpio_init_struct.gpio_pins = TEST_ADC_PIN;
    gpio_init(TEST_ADC_PORT, &gpio_init_struct);
    
    gpio_init_struct.gpio_pins = TEST_PIN;
    gpio_init(TEST_PORT, &gpio_init_struct);
    
    gpio_bits_set(LEDERR_GPIO_PORT, LEDERR_PIN);
    gpio_bits_reset(LEDRUN_GPIO_PORT, LEDRUN_PIN);
    
    // Motor
    /* configure the CH1C pin */
    gpio_init_struct.gpio_pins = L3_PIN;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init(L3_GPIO_PORT, &gpio_init_struct);

    /* configure the CH2C pin */
    gpio_init_struct.gpio_pins = L2_PIN;
    gpio_init(L2_GPIO_PORT, &gpio_init_struct);
    /* configure the CH3C pin */
    gpio_init_struct.gpio_pins = L1_PIN;
    gpio_init(L1_GPIO_PORT, &gpio_init_struct);
    /* configure the CH1 pin */
    gpio_init_struct.gpio_pins = H3_PIN;
    gpio_init(H3_GPIO_PORT, &gpio_init_struct);
    /* configure the CH2 pin */
    gpio_init_struct.gpio_pins = H2_PIN;
    gpio_init(H2_GPIO_PORT, &gpio_init_struct);
    /* configure the CH3 pin */
    gpio_init_struct.gpio_pins = H1_PIN;
    gpio_init(H1_GPIO_PORT, &gpio_init_struct);


    // ADC
    gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
    /* configure the VBUS_AD pin */
    gpio_init_struct.gpio_pins = VBUS_AD_PIN;
    gpio_init(VBUS_AD_GPIO_PORT, &gpio_init_struct);
    /* configure the TEMP_AD pin */
    gpio_init_struct.gpio_pins = TEMP_AD_PIN;
    gpio_init(TEMP_AD_GPIO_PORT, &gpio_init_struct);
    /* configure the IA pin */
    gpio_init_struct.gpio_pins = IA_PIN;
    gpio_init(IA_GPIO_PORT, &gpio_init_struct);
    /* configure the IB pin */
    gpio_init_struct.gpio_pins = IB_PIN;
    gpio_init(IB_GPIO_PORT, &gpio_init_struct);
    /* configure the IC pin */
    gpio_init_struct.gpio_pins = IC_PIN;
    gpio_init(IC_GPIO_PORT, &gpio_init_struct);
    
    
    // SPI
    gpio_default_para_init(&gpio_init_struct);

    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = GPIO_PINS_5;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOA, &gpio_init_struct);

    /* configure the MISO pin */
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = GPIO_PINS_6;
    gpio_init(GPIOA, &gpio_init_struct);

    /* configure the MOSI pin */
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = GPIO_PINS_7;
    gpio_init(GPIOA, &gpio_init_struct);

    /* configure the CS pin */
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pins = GPIO_PINS_4;
    gpio_init(GPIOA, &gpio_init_struct);
}


void InitMcu_MotorTimer(u16 u16PeakCnt, u16 u16DeadTimeCnt, u16 u16AdcTrigCnt){
    tmr_output_config_type tmr_output_struct;
    tmr_brkdt_config_type tmr_brkdt_struct;

    tmr_base_init(TMR1, u16PeakCnt - 1U, 1U - 1U);
    tmr_cnt_dir_set(TMR1, TMR_COUNT_TWO_WAY_1);
    tmr_clock_source_div_set(TMR1, TMR_CLOCK_DIV1);
    tmr_repetition_counter_set(TMR1, 1);
    
    /* configure channel 1 output settings */
    tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    tmr_output_struct.oc_output_state = TRUE;
    tmr_output_struct.occ_output_state = TRUE;
    tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.oc_idle_state = FALSE;
    tmr_output_struct.occ_idle_state = FALSE;
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &tmr_output_struct);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, 0);
    tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_1, TRUE);
    
    /* configure channel 2 output settings */
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_2, &tmr_output_struct);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, 0);
    tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_2, TRUE);
    tmr_output_channel_immediately_set(TMR1, TMR_SELECT_CHANNEL_2, FALSE);
    
    /* configure channel 3 output settings */
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_3, &tmr_output_struct);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, 0);
    tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_3, TRUE);
    tmr_output_channel_immediately_set(TMR1, TMR_SELECT_CHANNEL_3, FALSE);

    /* configure channel 4 output settings */
    tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
    tmr_output_struct.oc_output_state = TRUE;
    tmr_output_struct.occ_output_state = FALSE;
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_4, &tmr_output_struct);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, (u16PeakCnt - u16AdcTrigCnt));
    tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_4, TRUE);
    tmr_output_channel_immediately_set(TMR1, TMR_SELECT_CHANNEL_4, FALSE);

    tmr_brkdt_default_para_init(&tmr_brkdt_struct);
    tmr_brkdt_struct.deadtime = 50;
    tmr_brkdt_struct.brk_enable = FALSE;
    tmr_brkdt_struct.brk_polarity = TMR_BRK_INPUT_ACTIVE_LOW;

    tmr_brkdt_config(TMR1, &tmr_brkdt_struct);


    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_1, FALSE);
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2, FALSE);
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_3, FALSE);
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_1C, FALSE);
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2C, FALSE);
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_3C, FALSE);

    tmr_output_enable(TMR1, TRUE);
    tmr_counter_enable(TMR1, TRUE);
}


void InitMcu_Adc(void){
    dma_init_type dma_init_struct;

    dma_reset(DMA1_CHANNEL1);
    dma_flexible_config(DMA1, FLEX_CHANNEL1, DMA_FLEXIBLE_ADC1);

    dma_default_para_init(&dma_init_struct);
    dma_init_struct.buffer_size = 2;
    dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_base_addr = (uint32_t)(&Adc_u16AdcResult[3]);

    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = (uint32_t)&(ADC1->odt);
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_HIGH;
    dma_init_struct.loop_mode_enable = TRUE;
    dma_init(DMA1_CHANNEL1, &dma_init_struct);
    dma_channel_enable(DMA1_CHANNEL1, TRUE);

    // ADC1 Init
    adc_base_config_type adc_base_struct;

    adc_base_default_para_init(&adc_base_struct);
    adc_base_struct.sequence_mode = TRUE;
    adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
    adc_base_struct.ordinary_channel_length = 2;
    adc_base_config(ADC1, &adc_base_struct);

    adc_ordinary_channel_set(ADC1, ADC_CH_VDC, 1, ADC_SAMPLETIME_7_5);
    adc_ordinary_channel_set(ADC1, ADC_CH_TEMP, 2, ADC_SAMPLETIME_7_5);
    adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_SOFTWARE, TRUE);
    
    adc_dma_mode_enable(ADC1, TRUE);

    adc_enable(ADC1, TRUE);
    adc_calibration_init(ADC1);
    while(adc_calibration_init_status_get(ADC1));
    adc_calibration_start(ADC1);
    while(adc_calibration_status_get(ADC1));

    // ADC2 Init
    adc_base_default_para_init(&adc_base_struct);
    adc_base_struct.sequence_mode = TRUE;
    adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
    adc_base_config(ADC2, &adc_base_struct);

    adc_preempt_channel_length_set(ADC2, 3);
    adc_preempt_channel_set(ADC2, ADC_CH_IW, 1, ADC_SAMPLETIME_7_5);
    adc_preempt_channel_set(ADC2, ADC_CH_IV, 2, ADC_SAMPLETIME_7_5);
    adc_preempt_channel_set(ADC2, ADC_CH_IU, 3, ADC_SAMPLETIME_7_5);

    adc_preempt_conversion_trigger_set(ADC2, ADC12_PREEMPT_TRIG_TMR1CH4, TRUE);

    nvic_irq_enable(ADC1_2_IRQn, 0, 0);
    adc_interrupt_enable(ADC2, ADC_PCCE_INT, FALSE);
    adc_flag_clear(ADC2, ADC_PCCE_FLAG);

    adc_enable(ADC2, TRUE);
    adc_calibration_init(ADC2);
    while(adc_calibration_init_status_get(ADC2));
    adc_calibration_start(ADC2);
    while(adc_calibration_status_get(ADC2));

    adc_flag_clear(ADC2, ADC_PCCE_FLAG);
    adc_interrupt_enable(ADC2, ADC_PCCE_INT, TRUE);
}


void InitMcu_UpdateMotorPwmDuty(u16 u16_Occr0, u16 u16_Occr1, u16 u16_Occr2){
    TMR1->c1dt = u16_Occr2;
    TMR1->c2dt = u16_Occr1;
    TMR1->c3dt = u16_Occr0;
}

void InitMcu_EnMotorPwm(boolean_t bEnable){
    if (TRUE == bEnable){
        tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_1, TRUE);
        tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2, TRUE);
        tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_3, TRUE);
        tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_1C, TRUE);
        tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2C, TRUE);
        tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_3C, TRUE);
        
//        tmr_output_enable(TMR1, TRUE);

    }else{
        tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_1, FALSE);
        tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2, FALSE);
        tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_3, FALSE);
        tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_1C, FALSE);
        tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2C, FALSE);
        tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_3C, FALSE);

//        tmr_output_enable(TMR1, FALSE);
    }
}


void InitMcu_StartAdc1Sample(void){
    adc_ordinary_software_trigger_enable(ADC1, TRUE);
    while (dma_flag_get(DMA1_FDT1_FLAG) == RESET);
    dma_flag_clear(DMA1_FDT1_FLAG);
}