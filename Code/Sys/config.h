#ifndef __CONFIG_H__
#define __CONFIG_H__


#include <string.h>
#include <stdbool.h>
#include "at32f403a_407.h"
#include "at32f403a_407_int.h"

#include "adc_sample.h"
#include "init_mcu.h"
#include "timer.h"
#include "spix.h"

#include "usr_config.h"
#include "filter.h"
#include "mc_type.h"
#include "transform.h"
#include "svpwm.h"
#include "vf_control.h" 
#include "pi_calc.h"
#include "flux.h"
#include "startup.h"
#include "speed_set.h"
#include "protection.h"
#include "motor_control.h"
#include "stop_control.h"
#include "smo.h"
#include "encoder.h"
#include "calibration.h"
#include "hfi.h"


void SysClk_Init(void);
void Clock_Config(void);

#endif // __CONFIG_H__