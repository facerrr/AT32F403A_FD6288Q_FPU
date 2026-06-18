#include "config.h"
#include "usb.h"

//char txOnce[8] = "connect";
#define INFINITY                                       __builtin_inff()

float data_show[4];
uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};

int main(void){
    __disable_irq();
        
    MotorCfg_Init();
   
    InitMcu_Clock();
    InitMcu_Gpio();
            
    MotorHW_Init();
    MotorSW_Init();
    
    BSP_USB_Init();
    __enable_irq();
    
    OneMs_Define();
    
    while(1){
        Speed_CmdSpdSet(&MotorCmdSpdSet);
        data_show[3] = INFINITY;
        if(printflag == 0){
            data_show[0] = MotorRunPara.Iuvw.f32_U;
            data_show[1] = MotorRunPara.Iuvw.f32_V;
            data_show[2] = MotorRunPara.Iuvw.f32_W;
        }else if(printflag == 1){
            data_show[0] = MotorRunPara.Iqd.f32_D;
            data_show[1] = MotorRunPara.Iqd.f32_Q;
            data_show[2] = MotorRunPara.IqdRef.f32_Q;
        }else if(printflag == 2){
            data_show[0] = Encoder.f32_Phase;
            data_show[1] = MotorFluxObs.f32_ThetaEst;
            data_show[2] = MotorSMObs.f32_ThetaEst;
        }else if(printflag == 3){
            data_show[0] = MotorFluxObs.f32_PllOut;
            data_show[1] = MotorSvpwm.u16Von;
            data_show[2] = MotorSvpwm.u16Won;
        }else if(printflag == 4){
            data_show[0] = MotorRunPara.f32_RealTimeWr;
            data_show[1] = MotorRunPara.f32_CmdWr;
            data_show[2] = MotorCmdSpdSet.i32RealRpm;
        }
        usb_vcp_send_data(&usb_core_dev, (uint8_t *)data_show, (uint16_t)(sizeof(float) * 4));
    }
}

void Error_Handler(void){
    __disable_irq();
    tmr_output_enable(TMR1, FALSE);
    while (1) {
    }
}


void HardFault_Handler(void){
    __disable_irq();
    while(1){
    }
}

