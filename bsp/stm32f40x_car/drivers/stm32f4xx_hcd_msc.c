/*
About MSC state machine process
*/

#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>


#include "stm32f4xx_hcd.h"

#include "udisk.h"


#include "usbh_stdreq.h"



typedef union _USBH_CBW_Block
{
  struct __CBW
  {
    uint32_t CBWSignature;
    uint32_t CBWTag;
    uint32_t CBWTransferLength;
    uint8_t CBWFlags;
    uint8_t CBWLUN; 
    uint8_t CBWLength;
    uint8_t CBWCB[16];
}field;
  uint8_t CBWArray[31];
}HostCBWPkt_TypeDef;

typedef enum
{
  USBH_MSC_BOT_INIT_STATE = 0,                
  USBH_MSC_BOT_RESET,                
  USBH_MSC_GET_MAX_LUN,              
  USBH_MSC_TEST_UNIT_READY,          
  USBH_MSC_READ_CAPACITY10,
  USBH_MSC_MODE_SENSE6,
  USBH_MSC_REQUEST_SENSE,            
  USBH_MSC_BOT_USB_TRANSFERS,        
  USBH_MSC_DEFAULT_APPLI_STATE,  
  USBH_MSC_CTRL_ERROR_STATE,
  USBH_MSC_UNRECOVERED_STATE
}
MSCState;


typedef struct _BOTXfer
{
uint8_t MSCState;
uint8_t MSCStateBkp;
uint8_t MSCStateCurrent;
uint8_t CmdStateMachine;
uint8_t BOTState;
uint8_t BOTStateBkp;
uint8_t* pRxTxBuff;
uint16_t DataLength;
uint8_t BOTXferErrorCount;
uint8_t BOTXferStatus;
} USBH_BOTXfer_TypeDef;


typedef union _USBH_CSW_Block
{
  struct __CSW
  {
    uint32_t CSWSignature;
    uint32_t CSWTag;
    uint32_t CSWDataResidue;
    uint8_t  CSWStatus;
  }field;
  uint8_t CSWArray[13];
}HostCSWPkt_TypeDef;

#define USBH_MSC_SEND_CBW                 1
#define USBH_MSC_SENT_CBW                 2
#define USBH_MSC_BOT_DATAIN_STATE         3
#define USBH_MSC_BOT_DATAOUT_STATE        4
#define USBH_MSC_RECEIVE_CSW_STATE        5
#define USBH_MSC_DECODE_CSW               6
#define USBH_MSC_BOT_ERROR_IN             7
#define USBH_MSC_BOT_ERROR_OUT            8


#define USBH_MSC_BOT_CBW_SIGNATURE        0x43425355
#define USBH_MSC_BOT_CBW_TAG              0x20304050             
#define USBH_MSC_BOT_CSW_SIGNATURE        0x53425355           
#define USBH_MSC_CSW_DATA_LENGTH          0x000D
#define USBH_MSC_BOT_CBW_PACKET_LENGTH    31
#define USBH_MSC_CSW_LENGTH               13  
#define USBH_MSC_CSW_MAX_LENGTH           63     

/* CSW Status Definitions */
#define USBH_MSC_CSW_CMD_PASSED           0x00
#define USBH_MSC_CSW_CMD_FAILED           0x01
#define USBH_MSC_CSW_PHASE_ERROR          0x02

#define USBH_MSC_SEND_CSW_DISABLE         0
#define USBH_MSC_SEND_CSW_ENABLE          1

#define USBH_MSC_DIR_IN                   0
#define USBH_MSC_DIR_OUT                  1
#define USBH_MSC_BOTH_DIR                 2

//#define USBH_MSC_PAGE_LENGTH                 0x40
#define USBH_MSC_PAGE_LENGTH              512


#define CBW_CB_LENGTH                     16
#define CBW_LENGTH                        10
#define CBW_LENGTH_TEST_UNIT_READY         6

#define USB_REQ_BOT_RESET                0xFF
#define USB_REQ_GET_MAX_LUN              0xFE

#define MAX_BULK_STALL_COUNT_LIMIT       0x04   /* If STALL is seen on Bulk 
                                         Endpoint continously, this means 
                                         that device and Host has phase error
                                         Hence a Reset is needed */





#define OPCODE_TEST_UNIT_READY            0X00
#define OPCODE_READ_CAPACITY10            0x25
#define OPCODE_MODE_SENSE6                0x1A
#define OPCODE_READ10                     0x28
#define OPCODE_WRITE10                    0x2A
#define OPCODE_REQUEST_SENSE              0x03

#define DESC_REQUEST_SENSE                0X00
#define ALLOCATION_LENGTH_REQUEST_SENSE   63 
#define XFER_LEN_READ_CAPACITY10           8
#define XFER_LEN_MODE_SENSE6              63

#define MASK_MODE_SENSE_WRITE_PROTECT     0x80
#define MODE_SENSE_PAGE_CONTROL_FIELD     0x00
#define MODE_SENSE_PAGE_CODE              0x3F
#define DISK_WRITE_PROTECTED              0x01




void usbh_msc_botreset(uifinst_t pifinst, USB_OTG_CORE_HANDLE *pdev,USBH_HOST *phost)
{
	struct ureqest setup;
	uinst_t puinst;
	int ret;
	
	setup.request_type = USB_H2D | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE;
	setup.request = USB_REQ_BOT_RESET;
	setup.value = 0;
	setup.index = 0;
	setup.length = 0;			

	puinst=pifinst->uinst;
	ret=susb_control_xfer(puinst, &setup, RT_NULL, 0, 100);
	rt_kprintf("%s>ret=0x%x\r\n",__func__,ret);
}


USBH_Status usbh_msc_getmaxlun(uifinst_t pifinst,USB_OTG_CORE_HANDLE *pdev,USBH_HOST *phost)
{
	struct ureqest setup;
	uinst_t puinst;
	uint8_t buf[0x40];
	int ret,i;

	
	setup.request_type = USB_D2H | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE;
	setup.request = USB_REQ_GET_MAX_LUN;
	setup.value = 0;
	setup.index = 0;
	setup.length = 1;			

	puinst=pifinst->uinst;
	ret=susb_control_xfer(puinst, &setup, buf, 1, 100);
	rt_kprintf("%s>ret=%x  value=0x%x \r\n",__func__,ret,buf[0]);
}





void usbh_msc_testunitready(uifinst_t pifinst,USB_OTG_CORE_HANDLE *pdev,USBH_HOST *phost)
{
	HostCBWPkt_TypeDef USBH_MSC_CBWData;
	int index,ret;
	udiskinst_t pdiskinst=(udiskinst_t)(pifinst->user_data);
	uint8_t buf[USBH_MSC_CSW_MAX_LENGTH];

    USBH_MSC_CBWData.field.CBWSignature = USBH_MSC_BOT_CBW_SIGNATURE;
    USBH_MSC_CBWData.field.CBWTag = USBH_MSC_BOT_CBW_TAG;
    USBH_MSC_CBWData.field.CBWLUN = 0;  /*Only one LUN is supported*/

	USBH_MSC_CBWData.field.CBWTransferLength = 0;
	USBH_MSC_CBWData.field.CBWFlags = USB_EP_DIR_OUT;
	USBH_MSC_CBWData.field.CBWLength = CBW_LENGTH_TEST_UNIT_READY;

	for(index=CBW_CB_LENGTH;index!=0;index--)
	{
		USBH_MSC_CBWData.field.CBWCB[index] = 0x0;
	}
	USBH_MSC_CBWData.field.CBWCB[0] = OPCODE_TEST_UNIT_READY;
	ret=susb_bulk_xfer(pdiskinst->pipe_out,(void *)&USBH_MSC_CBWData,USBH_MSC_BOT_CBW_PACKET_LENGTH,100);
	//rt_kprintf("%s ret=0x%x\r\n",__func__,ret);
	/*receive CSW */
	ret=susb_bulk_xfer(pdiskinst->pipe_in,(void *)buf,USBH_MSC_CSW_MAX_LENGTH,100);
	rt_kprintf("%s ret=0x%x\r\n",__func__,ret);
	/*decode CSW*/
#if 0	
	for(index=0;index<USBH_MSC_CSW_MAX_LENGTH;index++)
	{
		rt_kprintf("%02x ",buf[index]);
	}
	rt_kprintf("\r\n");
#endif	
}


void usbh_msc_readcapacity10(uifinst_t pifinst,USB_OTG_CORE_HANDLE *pdev,USBH_HOST *phost)
{
	HostCBWPkt_TypeDef USBH_MSC_CBWData;
	int index,ret;
	udiskinst_t pdiskinst=(udiskinst_t)(pifinst->user_data);
	uint8_t buf[USBH_MSC_CSW_MAX_LENGTH];

    USBH_MSC_CBWData.field.CBWSignature = USBH_MSC_BOT_CBW_SIGNATURE;
    USBH_MSC_CBWData.field.CBWTag = USBH_MSC_BOT_CBW_TAG;
    USBH_MSC_CBWData.field.CBWLUN = 0;  /*Only one LUN is supported*/

	USBH_MSC_CBWData.field.CBWTransferLength = XFER_LEN_READ_CAPACITY10;
    USBH_MSC_CBWData.field.CBWFlags = USB_EP_DIR_IN;
    USBH_MSC_CBWData.field.CBWLength = CBW_LENGTH;

	for(index=CBW_LENGTH;index!=0;index--)
	{
		USBH_MSC_CBWData.field.CBWCB[index] = 0x0;
	}
	USBH_MSC_CBWData.field.CBWCB[0] = OPCODE_READ_CAPACITY10;
	ret=susb_bulk_xfer(pdiskinst->pipe_out,(void *)&USBH_MSC_CBWData,USBH_MSC_BOT_CBW_PACKET_LENGTH,100);
	//rt_kprintf("%s ret=0x%x\r\n",__func__,ret);
	/*receive CSW */
	ret=susb_bulk_xfer(pdiskinst->pipe_in,(void *)buf,USBH_MSC_CSW_MAX_LENGTH,100);
	//rt_kprintf("%s ret=0x%x\r\n",__func__,ret);

	rt_kprintf("Capacity=0x%08x PageLen=%04x\r\n",\
		(buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0],
		(buf[6]<<8)|buf[7]);
	/*decode CSW*/
#if 0	
	for(index=0;index<USBH_MSC_CSW_MAX_LENGTH;index++)
	{
		rt_kprintf("%02x ",buf[index]);
	}
	rt_kprintf("\r\n");
#endif	

}


void usbh_msc_modesense6(uifinst_t pifinst,USB_OTG_CORE_HANDLE *pdev,USBH_HOST *phost)
{
	HostCBWPkt_TypeDef USBH_MSC_CBWData;
	int index,ret;
	udiskinst_t pdiskinst=(udiskinst_t)(pifinst->user_data);
	uint8_t buf[USBH_MSC_CSW_MAX_LENGTH];

	 USBH_MSC_CBWData.field.CBWSignature = USBH_MSC_BOT_CBW_SIGNATURE;
	 USBH_MSC_CBWData.field.CBWTag = USBH_MSC_BOT_CBW_TAG;
	 USBH_MSC_CBWData.field.CBWLUN = 0;  /*Only one LUN is supported*/


	 USBH_MSC_CBWData.field.CBWTransferLength = XFER_LEN_MODE_SENSE6;
     USBH_MSC_CBWData.field.CBWFlags = USB_EP_DIR_IN;
     USBH_MSC_CBWData.field.CBWLength =CBW_LENGTH;
      
      
      for(index = CBW_CB_LENGTH; index != 0; index--)
      {
        USBH_MSC_CBWData.field.CBWCB[index] = 0x00;
      }    
      
      USBH_MSC_CBWData.field.CBWCB[0]  = OPCODE_MODE_SENSE6; 
      USBH_MSC_CBWData.field.CBWCB[2]  = MODE_SENSE_PAGE_CONTROL_FIELD | MODE_SENSE_PAGE_CODE;
      USBH_MSC_CBWData.field.CBWCB[4]  = XFER_LEN_MODE_SENSE6;
	ret=susb_bulk_xfer(pdiskinst->pipe_in,(void *)&USBH_MSC_CBWData,USBH_MSC_BOT_CBW_PACKET_LENGTH,100);
	rt_kprintf("%s ret=0x%x\r\n",__func__,ret);
	/*receive CSW */
//	ret=susb_bulk_xfer(pdiskinst->pipe_in,(void *)buf,USBH_MSC_CSW_MAX_LENGTH,100);
//	rt_kprintf("%s ret=0x%x\r\n",__func__,ret);
	/*decode CSW*/
	for(index=0;index<ret;index++)
	{
		rt_kprintf("%02x ",USBH_MSC_CBWData.CBWArray[index]);
	}
	rt_kprintf("\r\n");

}


void usbh_msc_requestsense(uifinst_t pifinst,USB_OTG_CORE_HANDLE *pdev,USBH_HOST *phost)
{
	HostCBWPkt_TypeDef USBH_MSC_CBWData;
	int index;
	udiskinst_t pdiskinst=(udiskinst_t)(pifinst->user_data);
	uint8_t buf[USBH_MSC_CSW_MAX_LENGTH];

	 USBH_MSC_CBWData.field.CBWSignature = USBH_MSC_BOT_CBW_SIGNATURE;
	 USBH_MSC_CBWData.field.CBWTag = USBH_MSC_BOT_CBW_TAG;
	 USBH_MSC_CBWData.field.CBWLUN = 0;  /*Only one LUN is supported*/


	 USBH_MSC_CBWData.field.CBWTransferLength = \
											   ALLOCATION_LENGTH_REQUEST_SENSE;
	 USBH_MSC_CBWData.field.CBWFlags = USB_EP_DIR_IN;
	 USBH_MSC_CBWData.field.CBWLength = CBW_LENGTH;
	 

	 for(index = CBW_CB_LENGTH; index != 0; index--)
	 {
	   USBH_MSC_CBWData.field.CBWCB[index] = 0x00;
	 }	  
	 
	 USBH_MSC_CBWData.field.CBWCB[0]  = OPCODE_REQUEST_SENSE; 
	 USBH_MSC_CBWData.field.CBWCB[1]  = DESC_REQUEST_SENSE;
	 USBH_MSC_CBWData.field.CBWCB[4]  = ALLOCATION_LENGTH_REQUEST_SENSE;

	susb_bulk_xfer(pdiskinst->pipe_in,(void *)&USBH_MSC_CBWData,USBH_MSC_BOT_CBW_PACKET_LENGTH,100);
	/*receive CSW */
	susb_bulk_xfer(pdiskinst->pipe_in,(void *)buf,USBH_MSC_CSW_MAX_LENGTH,100);
	/*decode CSW*/
	for(index=0;index<USBH_MSC_CSW_MAX_LENGTH;index++)
	{
		rt_kprintf("0x%x ",USBH_MSC_CBWData.CBWArray[index]);
	}
	rt_kprintf("\r\n");

}


void usbh_msc_write10(uifinst_t pifinst,USB_OTG_CORE_HANDLE *pdev,USBH_HOST *phost,uint8_t *dataBuffer,
                         uint32_t address,
                         uint32_t nbOfbytes)
{
	HostCBWPkt_TypeDef USBH_MSC_CBWData;
	int index;
	udiskinst_t pdiskinst=(udiskinst_t)(pifinst->user_data);
	uint8_t buf[USBH_MSC_CSW_MAX_LENGTH];
	uint16_t nbOfPages;

	  USBH_MSC_CBWData.field.CBWSignature = USBH_MSC_BOT_CBW_SIGNATURE;
	  USBH_MSC_CBWData.field.CBWTag = USBH_MSC_BOT_CBW_TAG;
	  USBH_MSC_CBWData.field.CBWLUN = 0;  /*Only one LUN is supported*/
	
 
      USBH_MSC_CBWData.field.CBWTransferLength = nbOfbytes;
      USBH_MSC_CBWData.field.CBWFlags = USB_EP_DIR_OUT;
      USBH_MSC_CBWData.field.CBWLength = CBW_LENGTH;
    
      
      for(index = CBW_CB_LENGTH; index != 0; index--)  
      {
        USBH_MSC_CBWData.field.CBWCB[index] = 0x00;
      }
      
      USBH_MSC_CBWData.field.CBWCB[0]  = OPCODE_WRITE10; 
      
      /*logical block address*/
      USBH_MSC_CBWData.field.CBWCB[2]  = (((uint8_t*)&address)[3]) ;
      USBH_MSC_CBWData.field.CBWCB[3]  = (((uint8_t*)&address)[2]);
      USBH_MSC_CBWData.field.CBWCB[4]  = (((uint8_t*)&address)[1]);
      USBH_MSC_CBWData.field.CBWCB[5]  = (((uint8_t*)&address)[0]);
      
      /*USBH_MSC_PAGE_LENGTH = 512*/
      nbOfPages = nbOfbytes/ USBH_MSC_PAGE_LENGTH; 
      
      /*Tranfer length */
      USBH_MSC_CBWData.field.CBWCB[7]  = (((uint8_t *)&nbOfPages)[1]) ; 
      USBH_MSC_CBWData.field.CBWCB[8]  = (((uint8_t *)&nbOfPages)[0]) ; 
      

	susb_bulk_xfer(pdiskinst->pipe_out,(void *)&USBH_MSC_CBWData,USBH_MSC_BOT_CBW_PACKET_LENGTH,100);
	/*receive CSW */
	susb_bulk_xfer(pdiskinst->pipe_in,(void *)buf,USBH_MSC_CSW_MAX_LENGTH,100);
	/*decode CSW*/
	for(index=0;index<USBH_MSC_CSW_MAX_LENGTH;index++)
	{
		rt_kprintf("0x%x ",USBH_MSC_CBWData.CBWArray[index]);
	}
	rt_kprintf("\r\n");

}

void usbh_msc_read10(uifinst_t pifinst,USB_OTG_CORE_HANDLE *pdev,USBH_HOST *phost,uint8_t *dataBuffer,
                         uint32_t address,
                         uint32_t nbOfbytes)
{
	HostCBWPkt_TypeDef USBH_MSC_CBWData;
	int index;
	udiskinst_t pdiskinst=(udiskinst_t)(pifinst->user_data);
	uint8_t buf[USBH_MSC_CSW_MAX_LENGTH];
	uint16_t nbOfPages;

	  USBH_MSC_CBWData.field.CBWSignature = USBH_MSC_BOT_CBW_SIGNATURE;
	  USBH_MSC_CBWData.field.CBWTag = USBH_MSC_BOT_CBW_TAG;
	  USBH_MSC_CBWData.field.CBWLUN = 0;  /*Only one LUN is supported*/
	
 
 	USBH_MSC_CBWData.field.CBWTransferLength = nbOfbytes;
	 USBH_MSC_CBWData.field.CBWFlags = USB_EP_DIR_IN;
	 USBH_MSC_CBWData.field.CBWLength = CBW_LENGTH;

    
      
      for(index = CBW_CB_LENGTH; index != 0; index--)  
      {
        USBH_MSC_CBWData.field.CBWCB[index] = 0x00;
      }
      
      USBH_MSC_CBWData.field.CBWCB[0]  = OPCODE_READ10; 
      
      /*logical block address*/
      USBH_MSC_CBWData.field.CBWCB[2]  = (((uint8_t*)&address)[3]) ;
      USBH_MSC_CBWData.field.CBWCB[3]  = (((uint8_t*)&address)[2]);
      USBH_MSC_CBWData.field.CBWCB[4]  = (((uint8_t*)&address)[1]);
      USBH_MSC_CBWData.field.CBWCB[5]  = (((uint8_t*)&address)[0]);
      
      /*USBH_MSC_PAGE_LENGTH = 512*/
      nbOfPages = nbOfbytes/ USBH_MSC_PAGE_LENGTH; 
      
      /*Tranfer length */
      USBH_MSC_CBWData.field.CBWCB[7]  = (((uint8_t *)&nbOfPages)[1]) ; 
      USBH_MSC_CBWData.field.CBWCB[8]  = (((uint8_t *)&nbOfPages)[0]) ; 
      

	susb_bulk_xfer(pdiskinst->pipe_out,(void *)&USBH_MSC_CBWData,USBH_MSC_BOT_CBW_PACKET_LENGTH,100);
	/*receive CSW */
	susb_bulk_xfer(pdiskinst->pipe_in,(void *)buf,USBH_MSC_CSW_MAX_LENGTH,100);
	/*decode CSW*/
	for(index=0;index<USBH_MSC_CSW_MAX_LENGTH;index++)
	{
		rt_kprintf("0x%x ",USBH_MSC_CBWData.CBWArray[index]);
	}
	rt_kprintf("\r\n");
	/*read data */
	susb_bulk_xfer(pdiskinst->pipe_in,(void *)dataBuffer,nbOfbytes,100);
	/*decode CSW*/
	for(index=0;index<nbOfbytes;index++)
	{
		rt_kprintf("0x%x ",dataBuffer[index]);
	}
	rt_kprintf("\r\n");

}




static rt_err_t msc_init(rt_device_t dev)
{
	rt_kprintf("%s\r\n",__func__);
    return RT_EOK;
}

static rt_size_t msc_read(rt_device_t dev, rt_off_t sector, void* buff, rt_size_t count)
{
	__IO uint8_t status = USBH_MSC_OK;
if(HCD_IsDeviceConnected(&USB_OTG_Core))
{  
  
  do
  {
	status = USBH_MSC_Read10(&USB_OTG_Core, buff,sector,512 * count);
	USBH_MSC_HandleBOTXfer(&USB_OTG_Core ,&USB_Host);
	
	if(!HCD_IsDeviceConnected(&USB_OTG_Core))
	{ 
	  //return RES_ERROR;
	  rt_kprintf("%s error\r\n",__func__);
	  return USBH_MSC_FAIL;
	}	   
  }
  while(status == USBH_MSC_BUSY );
}
if(status==USBH_MSC_OK) return count;
return 0xff;

}


static rt_size_t msc_write(rt_device_t dev, rt_off_t sector, const void* buff, rt_size_t count)
{
	BYTE status = USBH_MSC_OK;
	rt_kprintf("%s\r\n",__func__);
	if(HCD_IsDeviceConnected(&USB_OTG_Core))
	{  
	  do
	  {
		status = USBH_MSC_Write10(&USB_OTG_Core,(BYTE*)buff,sector,512 * count);
		USBH_MSC_HandleBOTXfer(&USB_OTG_Core, &USB_Host);
		
		if(!HCD_IsDeviceConnected(&USB_OTG_Core))
		{ 
		  //return RES_ERROR;
		  rt_kprintf("%s error\r\n",__func__);
		  return USBH_MSC_FAIL;
		}
	  }
	  
	  while(status == USBH_MSC_BUSY );
	}
if(status==USBH_MSC_OK) return count;
return 0xff;



}

rt_err_t msc_control(rt_device_t dev, rt_uint8_t cmd, void *arg)
{
	struct rt_device_blk_geometry *pgeometry = (struct rt_device_blk_geometry *)arg;
	switch(cmd)
	{
		case RT_DEVICE_CTRL_BLK_GETGEOME:
			pgeometry->sector_count=USBH_MSC_Param.MSCapacity;
			pgeometry->bytes_per_sector=USBH_MSC_Param.MSPageLength;
			pgeometry->block_size=0;
			break;
		case RT_DEVICE_CTRL_BLK_SYNC:

			break;

	}



}


static rt_err_t msc_close(rt_device_t dev)
{
	rt_kprintf("%s\r\n",__func__);

    return RT_EOK;
}

static rt_err_t msc_open(rt_device_t dev, rt_uint16_t oflag)
{
	rt_kprintf("%s\r\n",__func__);

    return RT_EOK;
}



void usbh_msc_start(uifinst_t pifinst)
{
	udiskinst_t pdiskinst=(udiskinst_t)(pifinst->user_data);
	int ret;

	usbh_msc_botreset(pifinst,&USB_OTG_Core,&USB_Host);
	usbh_msc_getmaxlun(pifinst,&USB_OTG_Core,&USB_Host);
	usbh_msc_testunitready(pifinst,&USB_OTG_Core,&USB_Host);
	usbh_msc_readcapacity10(pifinst,&USB_OTG_Core,&USB_Host);
	
	//usbh_msc_modesense6(pifinst,&USB_OTG_Core,&USB_Host);
	/*mount driver*/
	    /* register adk device */
    pdiskinst->device.type  = RT_Device_Class_Block;                         
    pdiskinst->device.init = msc_init;         
    pdiskinst->device.open = msc_open;         
    pdiskinst->device.close = msc_close;                 
    pdiskinst->device.read = msc_read;
    pdiskinst->device.write = msc_write;
    pdiskinst->device.control = msc_control;
    pdiskinst->device.user_data = (void*)pifinst;

    rt_device_register(&pdiskinst->device, "udisk", RT_DEVICE_FLAG_RDWR);
	ret=dfs_mount("udisk","/","elm",0,0);
	rt_kprintf("dfs_mount ret=%x\r\n",ret);
}



