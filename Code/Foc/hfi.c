#include "config.h"

void HFI_Init(pHFI pHfi, pHFI2Observer pHfi2Obs){
    memset((void*)pHfi, 0, sizeof(HFI_t));
    pHfi->f32_Ts = 1.0f / MotorCfg.f32CarrierFreqHz;
    pHfi->f32_IdRef = 0.0f;
    pHfi->f32_IdHigh = 0.0f;
    pHfi->f32_InjectMagMax = MotorCfg.f32HfiInjectMagMax;
    pHfi->f32_InjectMagMin = MotorCfg.f32HfiInjectMagMin;
    
    pHfi->f32_PllKp = MotorCfg.f32HfiPllKp;
    pHfi->f32_PllKi = MotorCfg.f32HfiPllKi;
    
    memset((void*)pHfi2Obs, 0, sizeof(HFI2Observer_t));
    pHfi2Obs->f32_WrMax = MotorCfg.f32Hfi2ObsWrMax;
    pHfi2Obs->f32_WrMin = MotorCfg.f32Hfi2ObsWrMin;
    pHfi2Obs->f32_WrMid = MotorCfg.f32Hfi2ObsWrMid;
    
}

void HFI_Calculate(pHFI pv, QD_t Iqd, AB_t Iab){
    pv->f32_Iq = Iqd.f32_Q;
    pv->f32_Id = Iqd.f32_D;

    pv->f32_Ialpha = Iab.f32_Alpha;
    pv->f32_Ibeta = Iab.f32_Beta;

    // --- 1. 高频电流提取与同步解调 (持续运行以保持滤波器状态) ---
    pv->f32_IalphaHighLast = pv->f32_IalphaHigh;
    pv->f32_IbetaHighLast  = pv->f32_IbetaHigh;

    pv->f32_IalphaHigh = (pv->f32_Ialpha - pv->f32_IalphaLast) * 0.5f;
    pv->f32_IbetaHigh  = (pv->f32_Ibeta  - pv->f32_IbetaLast)  * 0.5f;

    pv->f32_IalphaLast = pv->f32_Ialpha;
    pv->f32_IbetaLast  = pv->f32_Ibeta;

    // 差分/解调提取注入频率下的电流包络
    if (pv->u8_Dir == 0){
        pv->f32_IalphaOut = pv->f32_IalphaHigh - pv->f32_IalphaHighLast;
        pv->f32_IbetaOut  = pv->f32_IbetaHigh  - pv->f32_IbetaHighLast;
        pv->u8_Dir = 1; 
    }else if (pv->u8_Dir == 1){
        pv->f32_IalphaOut = pv->f32_IalphaHighLast - pv->f32_IalphaHigh;
        pv->f32_IbetaOut  = pv->f32_IbetaHighLast  - pv->f32_IbetaHigh;
        pv->u8_Dir = 0;
    }

    // --- 2. 初始极性辨识 (NSD - 消除180度模糊) ---
    if(pv->u8_NSDFlag == 0){
        pv->u16_NSDCount++;
        
        // 辨识期间采样 D轴高频响应 以判断磁饱和程度
        pv->f32_IdHigh = (pv->f32_Id - pv->f32_IdLast) * 0.5f;
        if (pv->f32_IdHigh < 0.0f){
            pv->f32_IdHigh = -pv->f32_IdHigh;
        }

        if (pv->u16_NSDCount < 10400){
            // 阶段0：等待观测器/滤波器收敛（约400次调用）
            pv->f32_IdRef = 5.0f;
        }else if (pv->u16_NSDCount >= 10400 && pv->u16_NSDCount < 10600){
            // 阶段1：施加正向电流脉冲（+5A）
            pv->f32_IdRef = 5.0f;
        }else if (pv->u16_NSDCount >= 10600 && pv->u16_NSDCount < 10610){
            // 阶段2：保持正向电流，同时采样高频电流幅值（累加10次）
            pv->f32_IdRef = 5.0f;
            pv->f32_NSDSum1 += pv->f32_IdHigh;
        }else if (pv->u16_NSDCount >= 10610 && pv->u16_NSDCount < 10810){
            // 阶段3：等待电流归零
            pv->f32_IdRef = 0.0f;
        }else if (pv->u16_NSDCount >= 10810 && pv->u16_NSDCount < 11010){
            // 阶段4：施加负向电流脉冲（-5A）
            pv->f32_IdRef = -5.0f;
        }else if (pv->u16_NSDCount >= 11010 && pv->u16_NSDCount < 11020){
            // 阶段5：保持负向电流，采样高频电流幅值（累加10次）
            pv->f32_IdRef = -5.0f;
            pv->f32_NSDSum2 += pv->f32_IdHigh;
        }else if (pv->u16_NSDCount == 11020){
            // 阶段6：电流归零
            pv->f32_IdRef = 0.0f;
        }else if (pv->u16_NSDCount == 11021){
            // 阶段7：完成NSD，比较正负脉冲下的高频电流响应幅值
            pv->u8_NSDFlag = 1;       // NSD完成标志
            pv->u16_NSDCount = 0;      // 计数器复位

            // 若负向电流下的高频幅值大于正向，则NSDOut=1，否则0
            // 该结果用于确定转子N极相对于注入轴的方向
            if (pv->f32_NSDSum2 > pv->f32_NSDSum1){
                pv->u8_NSDOut = 1;
            }else{
                pv->u8_NSDOut = 0;
            }
        }
        // 冻结PLL
        pv->f32_PllOut = 0.0f;
    } else {
        // --- 3. 极性补偿与常规角度闭环跟踪 (PLL) ---
        if(pv->u8_NSDOut == 1){
            pv->u8_NSDOut = 0;
            pv->f32_ThetaEst += M_PI;
            utils_norm_angle_rad(&pv->f32_ThetaEst);
        }

        float32_t f32_IqHigh = -(pv->f32_IalphaOut * pv->f32_SinTheta) + (pv->f32_IbetaOut * pv->f32_CosTheta);
 
        float32_t f32_PllErr = f32_IqHigh; 
        
        float32_t f32_KpPart = pv->f32_PllKp * f32_PllErr;
        pv->f32_PllIsum += (pv->f32_PllKi * f32_PllErr);
        float32_t f32_WrMidVar = f32_KpPart + pv->f32_PllIsum;
        pv->f32_PllOut = f32_WrMidVar;

        pv->f32_ThetaEst += f32_WrMidVar * pv->f32_Ts; 
    }

    // --- 4. 角度标准化与参数更新 ---
    utils_norm_angle_rad(&pv->f32_ThetaEst);

    pv->f32_CosTheta = cos_f32(pv->f32_ThetaEst);
    pv->f32_SinTheta = sin_f32(pv->f32_ThetaEst);
        
    pv->f32_RealTimeWr = pv->f32_PllOut * MotorCfg.f32CarrierFreqHz;

    pv->f32_IdBase = (pv->f32_Id + pv->f32_IdLast) * 0.5f;
    pv->f32_IqBase = (pv->f32_Iq + pv->f32_IqLast) * 0.5f;

    pv->f32_IdLast = pv->f32_Id;
    pv->f32_IqLast = pv->f32_Iq;
}


#define HFI 0
#define OBS 1

void HFI2Observer_Calculate(pHFI2Observer pv, pHFI pHfi){

    pv->f32_ThetaErr = pv->f32_HFIThetaEst - pv->f32_ObsThetaEst;
    if(pv->f32_ThetaErr > M_2PI) {
        pv->f32_ThetaErr -= M_2PI;
    }
    if(pv->f32_ThetaErr < -M_2PI) {
        pv->f32_ThetaErr += M_2PI;
    }

    pv->i16_CheckCnt = pv->f32_ThetaErr <= 0.3f ? pv->i16_CheckCnt + 1 : pv->i16_CheckCnt - 1;

    float32_t f32_HfiWrAbs = fabsf(pv->f32_HFIWr);
    float32_t f32_ObsWrAbs = fabsf(pv->f32_ObsWr);
    float32_t f32_TrgtWrAbs = fabsf(pv->f32_TrgtWr);
    if(f32_HfiWrAbs <= pv->f32_WrMin || f32_ObsWrAbs <= pv->f32_WrMin){
        pHfi->u8_Enable = 1;
        pv->f32_WrOut = pv->f32_HFIWr;
        pv->f32_ThetaOut = pv->f32_HFIThetaEst;
        pv->u8_CurrentMode = HFI;
        pHfi->f32_Uin -= 0.0002f;
        
    }else if((f32_TrgtWrAbs > pv->f32_WrMin && f32_TrgtWrAbs < pv->f32_WrMax) ||
             (f32_HfiWrAbs > pv->f32_WrMin && f32_HfiWrAbs < pv->f32_WrMax &&
              f32_ObsWrAbs > pv->f32_WrMin && f32_ObsWrAbs < pv->f32_WrMax))
    {   
        if(pv->u8_CurrentMode == HFI){
            if(pv->i16_CheckCnt > 100){
                pv->f32_WrOut = pv->f32_ObsWr;
                pv->f32_ThetaOut = pv->f32_ObsThetaEst;
                pHfi->f32_Uin -= 0.0002f;
            }else if(pv->i16_CheckCnt <= 30){
                pv->f32_WrOut = pv->f32_HFIWr;
                pv->f32_ThetaOut = pv->f32_HFIThetaEst;
            }else{
                pv->f32_WrOut = pv->f32_WrOut;
                pv->f32_ThetaOut = pv->f32_ThetaOut;
            }
        }else{
            if(pv->i16_CheckCnt > 100){
                pv->f32_WrOut = pv->f32_HFIWr;
                pv->f32_ThetaOut = pv->f32_HFIThetaEst;
                pHfi->f32_Uin -= 0.02f;
            }else if(pv->i16_CheckCnt < 30){
                pv->f32_WrOut = pv->f32_ObsWr;
                pv->f32_ThetaOut = pv->f32_ObsThetaEst;
                pHfi->f32_Uin += 0.02f;
            }else{
                pv->f32_WrOut = pv->f32_WrOut;
                pv->f32_ThetaOut = pv->f32_ThetaOut;
            }
        }
        
    }else{
        if(f32_ObsWrAbs > (pv->f32_WrMax + pv->f32_WrMid * 3.0f)){
            pHfi->u8_Enable = 0;
        }else if(f32_ObsWrAbs < (pv->f32_WrMax + pv->f32_WrMid * 2.5f)){
            pHfi->u8_Enable = 1;
            
            float32_t f32_WrChangeRate = fabsf(pv->f32_WrOut) - fabsf(pv->f32_WrLast);
            pv->f32_WrLast = pv->f32_WrOut;
            
            if(f32_WrChangeRate < 0.0f && fabsf(f32_WrChangeRate) > 56.0f){
                pHfi->f32_Uin = pHfi->f32_InjectMagMax;
            }else if(f32_WrChangeRate > 0.0f && f32_WrChangeRate < 14.0f){
                pHfi->f32_Uin -= 0.0002f;
            }
        }
        pv->f32_WrOut = pv->f32_ObsWr;
        pv->f32_ThetaOut = pv->f32_ObsThetaEst;
        pv->u8_CurrentMode = OBS;
    }

    pHfi->f32_Uin = CLAMP(pHfi->f32_Uin, pHfi->f32_InjectMagMin, pHfi->f32_InjectMagMax);

    if(pv->i16_CheckCnt > 120){
        pv->i16_CheckCnt = 120;
    }else if(pv->i16_CheckCnt <= 0){
        pv->i16_CheckCnt = 0;
    }

}