#ifndef __USB_H__
#define __USB_H__

#include <stdint.h>
#include "usbd_core.h"
#include "cdc_class.h"
#include "cdc_desc.h"
#include "usbd_int.h"

extern usbd_core_type usb_core_dev;
extern uint8_t printflag;
extern uint8_t reverseFlag;

void BSP_USB_Init(void);
void USB_SendData(uint8_t ch, uint16_t len);
void USB_RX_Handle(void);
void parse_frame(uint8_t *buf, uint16_t rx_len);
void USBFS_L_CAN1_RX0_IRQHandler(void);
    
#endif // 


