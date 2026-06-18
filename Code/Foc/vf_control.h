#ifndef __VF_STARTUP_H__
#define __VF_STARTUP_H__

#include "mc_type.h"

void VFControl_Init(pVFControl pv);
void VFAngle_Calc(pVFControl pv);
void VFSpeed_Calc(pVFControl pv);

#endif // __VF_STARTUP_H__
