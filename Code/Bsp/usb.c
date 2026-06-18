#include "usb.h"
#include "config.h"


usbd_core_type usb_core_dev;

uint8_t buf[128];

uint8_t printflag = 0;
uint8_t reverseFlag = 0;

float data_to_float(uint8_t *data);

void BSP_USB_Init(void)
{
    //    crm_usb_clock_div_set(CRM_USB_DIV_4);
    crm_usb_clock_source_select(CRM_USB_CLOCK_SOURCE_HICK);
    crm_periph_clock_enable(CRM_ACC_PERIPH_CLOCK, TRUE);

    acc_write_c1(7980);
    acc_write_c2(8000);
    acc_write_c3(8020);

    acc_calibration_mode_enable(ACC_CAL_HICKTRIM, TRUE);

    crm_periph_clock_enable(CRM_USB_PERIPH_CLOCK, TRUE);

    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(USBFS_L_CAN1_RX0_IRQn, 0, 0);

    usbd_core_init(&usb_core_dev, USB, &cdc_class_handler, &cdc_desc_handler, 0);

    usbd_connect(&usb_core_dev);
}



void USBFS_L_CAN1_RX0_IRQHandler(void)
{
    usbd_irq_handler(&usb_core_dev);
    USB_RX_Handle();
}


void USB_RX_Handle(void)
{

    uint16_t usb_rx_len = 0;
    usb_rx_len = usb_vcp_get_rxdata(&usb_core_dev, buf);

    if(usb_rx_len != 0){
        parse_frame(buf, usb_rx_len);
    }
}

void parse_frame(uint8_t *buf, uint16_t rx_len)
{
    int ret;
    uint8_t *send_buf;
    
    if (buf[0] == 0x01)
    {
        MotorCmdSpdSet.i32CmdRpm = 0;
    }
    else if (buf[0] == 0x02)
    {
        MotorCmdSpdSet.i32CmdRpm = 300;
    }
    else if (buf[0] == 0x03)
    {
        Calibration_Start(&MotorCalibCtrl);
    }
    else if(buf[0] == 0x04){
    }
    else if(buf[0] == 0x09){
    }
    else if(buf[0] == 0x99){
        printflag = 0;
    }else if(buf[0] == 0x98){
        printflag = 1;
    }else if(buf[0] == 0x97){
        printflag = 2;
    }else if(buf[0] == 0x96){
        printflag = 3;
    }else if(buf[0] == 0x95){
        printflag = 4; 
    }else if(buf[0] == 0x90){
        MotorCmdSpdSet.i32CmdRpm = -MotorCmdSpdSet.i32CmdRpm;
    }else if(buf[0] == 0x15){
        if(rx_len == 5){
            MotorCmdSpdSet.i32CmdRpm = (int32_t)data_to_float(&buf[1]);
        }
    }
}


float data_to_float(uint8_t *data)
{
    float tmp_float;
    *(((uint8_t *) (&tmp_float)) + 0) = *(data);
    *(((uint8_t *) (&tmp_float)) + 1) = *(data + 1);
    *(((uint8_t *) (&tmp_float)) + 2) = *(data + 2);
    *(((uint8_t *) (&tmp_float)) + 3) = *(data + 3);
    return tmp_float;
}

int32_t data_to_int32(uint8_t *data)
{
    int32_t tmp;
    *(((uint8_t *) (&tmp)) + 0) = data[0];
    *(((uint8_t *) (&tmp)) + 1) = data[1];
    *(((uint8_t *) (&tmp)) + 2) = data[2];
    *(((uint8_t *) (&tmp)) + 3) = data[3];
    return tmp;
}


void usb_delay_ms(uint32_t ms)
{
    delay_ms(ms);
}

void usb_delay_us(uint32_t us)
{
    delay_us(us);
}


