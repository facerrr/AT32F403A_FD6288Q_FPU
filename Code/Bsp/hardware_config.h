#ifndef __HARDWARE_CONFIG_H__
#define __HARDWARE_CONFIG_H__


#include "base_types.h"

/******************************************************************************
 * system clock configuration
 *****************************************************************************/
#define CLK_HSRC_FREQ               240      // frequency of HSRC (MHz)
#define CLK_HSRC_DIV                0       // set HSRC clock divider, {0,1,2,3,4,5}
#define APB1_CLK_DIV                0       // set system clock divider

#define CLK_USE_RCH  0
#define CLK_USE_XTH  1

#define CLK_SOURCE   CLK_USE_XTH
#define SYSTEM_XTH          (8u*1000u*1000u)    /* 8MHz */

#define SWAP_ROTOR_DIR      FALSE   /** TRUE: swap default direction, FALSE: default direction */
#define EN_DT_VOLT_RECOVER  FALSE    /** TRUE: apply dead-time voltage recovery, FALSE: none recovery */

#define PWM_ACTIVE_LEVEL    PWM_ACTIVE_LEVEL_HH /** PWM active-level setting **/

/*******************************************************************************
** ADC sampling: default use ADC1 for sampling
*******************************************************************************/
#define ADC_VOLT_REF        (3.3f)      /** reference voltage of ADC        */
#define ADC_BIT_LENGTH      12          /** 12-bit ADC                      */
#define SHUNT_NUM           SHUNT_NUM_2 /** current sample shunt numbers    */

#define ADC_CH_IU           ADC_CHANNEL_2           /** IU  sample channel              */
#define ADC_CH_IV           ADC_CHANNEL_1           /** IV  sample channel, case single-shunt, ignore */
#define ADC_CH_IW           ADC_CHANNEL_0           /** IW  sample channel              */

#define ADC_CH_TEMP         ADC_CHANNEL_8           /** TEMP sample channel             */
#define ADC_CH_VDC          ADC_CHANNEL_3           /** VDC sample channel              */


#define LEDERR_GPIO_PORT        GPIOA
#define LEDERR_PIN              GPIO_PINS_7
#define LEDRUN_GPIO_PORT        GPIOB
#define LEDRUN_PIN              GPIO_PINS_0
#define TEST_ADC_PORT           GPIOA
#define TEST_ADC_PIN            GPIO_PINS_5
#define TEST_PORT               GPIOA
#define TEST_PIN                GPIO_PINS_6

#define IA_PIN                  GPIO_PINS_2
#define IA_GPIO_PORT            GPIOA
#define IB_PIN                  GPIO_PINS_1
#define IB_GPIO_PORT            GPIOA
#define IC_PIN                  GPIO_PINS_0
#define IC_GPIO_PORT            GPIOA

#define H1_PIN                  GPIO_PINS_10
#define H1_GPIO_PORT            GPIOA
#define H2_PIN                  GPIO_PINS_9
#define H2_GPIO_PORT            GPIOA
#define H3_PIN                  GPIO_PINS_8
#define H3_GPIO_PORT            GPIOA

#define L1_PIN                  GPIO_PINS_15
#define L1_GPIO_PORT            GPIOB
#define L2_PIN                  GPIO_PINS_14
#define L2_GPIO_PORT            GPIOB
#define L3_PIN                  GPIO_PINS_13
#define L3_GPIO_PORT            GPIOB

#define VBUS_AD_PIN             GPIO_PINS_4
#define VBUS_AD_GPIO_PORT       GPIOA

#define TEMP_AD_PIN             GPIO_PINS_3
#define TEMP_AD_GPIO_PORT       GPIOA



#endif // __HARDWARE_CONFIG_H__