#ifndef _STM32F4XX_HCD_H_
#define _STM32F4XX_HCD_H_

#include <rtthread.h>
#include "usb_core.h"
#include "usbh_core.h"

//#include "usb_host.h"

extern USB_OTG_CORE_HANDLE USB_OTG_Core;
extern USBH_HOST USB_Host;


int susb_control_xfer(uinst_t uinst, ureq_t setup, void* buffer,int nbytes, int timeout);

int susb_int_xfer(upipe_t pipe, void* buffer, int nbytes, int timeout);

int susb_bulk_xfer(upipe_t pipe, void* buffer, int nbytes, int timeout);

int susb_iso_xfer(upipe_t pipe, void* buffer, int nbytes, int timeout);
rt_err_t susb_alloc_pipe(upipe_t* pipe, uifinst_t ifinst, uep_desc_t ep, func_callback callback);
rt_err_t susb_free_pipe(upipe_t pipe);

rt_err_t susb_hub_control(rt_uint16_t port, rt_uint8_t cmd, void* args);



#endif

