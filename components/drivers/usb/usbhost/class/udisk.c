#include <rtthread.h>
#include <drivers/usb_host.h>
#include "udisk.h"



static struct uclass_driver udisk_driver;

struct rt_device udisk_dev;






/**
 * This function will run adk class driver when usb device is detected and identified
 *  as a mass storage class device, it will continue the enumulate process.
 *
 * @param arg the argument.
 * 
 * @return the error code, RT_EOK on successfully.
 */

extern void usbh_msc_start(uifinst_t pifinst);

static rt_err_t rt_usb_udisk_run(void* arg)
{
    int i = 0;
    udiskinst_t udisk_inst;
    uifinst_t ifinst = (uifinst_t)arg;
    udev_desc_t dev_desc;
    rt_uint16_t protocol;
    rt_err_t ret;    

	struct uhost_msg msg;
	int state;
	struct _udiskmsgcontext context;
    
    /* parameter check */
    if(ifinst == RT_NULL)
    {
        rt_kprintf("the interface is not available\n");
        return -RT_EIO;
    }

    RT_DEBUG_LOG(RT_DEBUG_USB,("rt_usb_udisk_run\n"));
        
    dev_desc = &ifinst->uinst->dev_desc;
	rt_kprintf("desc=%x:%x\r\n",dev_desc->idVendor,dev_desc->idProduct);
	rt_kprintf("ifinst->intf_desc->bNumEndpoints=%d\r\n",ifinst->intf_desc->bNumEndpoints);

	
//alloc bulk ep

udisk_inst = rt_malloc(sizeof(struct _udiskinst));
RT_ASSERT(udisk_inst != RT_NULL);

/* initilize the data structure */
rt_memset(udisk_inst, 0, sizeof(struct _udiskinst));    
ifinst->user_data = (void*)udisk_inst;


for(i=0; i<ifinst->intf_desc->bNumEndpoints; i++)
{		 
	uep_desc_t ep_desc;
	
	/* get endpoint descriptor from interface descriptor */
	rt_usb_get_endpoint_descriptor(ifinst->intf_desc, i, &ep_desc);
	if(ep_desc == RT_NULL)
	{
		rt_kprintf("rt_usb_get_endpoint_descriptor error\n");
		return -RT_ERROR;
	}
	
	/* the endpoint type of adk class should be BULK */    
	if((ep_desc->bmAttributes & USB_EP_ATTR_TYPE_MASK) != USB_EP_ATTR_BULK)
		continue;
	
	/* allocate pipes according to the endpoint type */
	if(ep_desc->bEndpointAddress & USB_DIR_IN)
	{
		/* allocate an in pipe for the adk instance */
		ret = rt_usb_hcd_alloc_pipe(ifinst->uinst->hcd, &udisk_inst->pipe_in,ifinst, ep_desc, RT_NULL);
		if(ret != RT_EOK) 
		{
			rt_kprintf("alloc pipe_in error\r\n");
			return ret;
		}	
	}
	else
	{		 
		/* allocate an output pipe for the adk instance */
		ret = rt_usb_hcd_alloc_pipe(ifinst->uinst->hcd, &udisk_inst->pipe_out,ifinst, ep_desc, RT_NULL);			  
		if(ret != RT_EOK) return ret;
	}
}

/* check pipes infomation */
if(udisk_inst->pipe_in == RT_NULL || udisk_inst->pipe_out == RT_NULL)
{
	rt_kprintf("pipe error, unsupported device\n");
	return -RT_ERROR;
}	 

/* set configuration */
ret = rt_usb_set_configure(ifinst->uinst, 1);
if(ret != RT_EOK) return ret;

	usbh_msc_start(ifinst);

/*
msg.type=USB_MSG_CALLBACK;
msg.content.cb.function=&usbh_msc_callback;
context.ifinst=(uifinst_t)arg;
state=0;
context.payload=(void*)state;
msg.content.cb.context=&context;
rt_usb_post_event(&msg, sizeof(struct uhost_msg));
*/

#if 0
    /* register adk device */
    adkinst->device.type  = RT_Device_Class_Char;                         
    adkinst->device.init = RT_NULL;         
    adkinst->device.open = RT_NULL;         
    adkinst->device.close = RT_NULL;                 
    adkinst->device.read = rt_usb_adk_read;
    adkinst->device.write = rt_usb_adk_write;
    adkinst->device.control = RT_NULL;
    adkinst->device.user_data = (void*)ifinst;
 

    rt_device_register(&adkinst->device, "adkdev", RT_DEVICE_FLAG_RDWR);
#endif   
    return RT_EOK;
}

/**
 * This function will be invoked when usb device plug out is detected and it would clean 
 * and release all hub class related resources.
 *
 * @param arg the argument.
 * 
 * @return the error code, RT_EOK on successfully.
 */
static rt_err_t rt_usb_udisk_stop(void* arg)
{
//    uadkinst_t adkinst;
    uifinst_t ifinst = (uifinst_t)arg;

    RT_ASSERT(ifinst != RT_NULL);

    RT_DEBUG_LOG(RT_DEBUG_USB, ("rt_usb_udisk_stop\n"));
#if 0
    adkinst = (uadkinst_t)ifinst->user_data;
    if(adkinst == RT_NULL) 
    {
        rt_free(ifinst);    
        return RT_EOK;
    }
    
    if(adkinst->pipe_in != RT_NULL)
        rt_usb_hcd_free_pipe(ifinst->uinst->hcd, adkinst->pipe_in);

    if(adkinst->pipe_out != RT_NULL)
        rt_usb_hcd_free_pipe(ifinst->uinst->hcd, adkinst->pipe_out);

    /* unregister adk device */
    rt_device_unregister(&adkinst->device);

    /* free adk instance */
    if(adkinst != RT_NULL) rt_free(adkinst);
    
    /* free interface instance */
    rt_free(ifinst);
#endif
    return RT_EOK;
}



ucd_t rt_usb_class_driver_storage(void)
{
    udisk_driver.class_code = USB_CLASS_MASS_STORAGE;
    
    udisk_driver.run = rt_usb_udisk_run;
    udisk_driver.stop = rt_usb_udisk_stop;

    return &udisk_driver;
}



