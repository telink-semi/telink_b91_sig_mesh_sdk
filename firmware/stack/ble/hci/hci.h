/********************************************************************************************************
 * @file	hci.h
 *
 * @brief	for TLSR chips
 *
 * @author	BLE GROUP
 * @date	2020.06
 *
 * @par		Copyright (c) 2020, Telink Semiconductor (Shanghai) Co., Ltd.
 *			All rights reserved.
 *
 *			The information contained herein is confidential property of Telink
 *          Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *          of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *          Co., Ltd. and the licensee or the terms described here-in. This heading
 *          MUST NOT be removed from this file.
 *
 *          Licensee shall not delete, modify or alter (or permit any third party to delete, modify, or  
 *          alter) any information contained herein in whole or in part except as expressly authorized  
 *          by Telink semiconductor (shanghai) Co., Ltd. Otherwise, licensee shall be solely responsible  
 *          for any claim to the extent arising out of or relating to such deletion(s), modification(s)  
 *          or alteration(s).
 *
 *          Licensees are granted free, non-transferable use of the information in this
 *          file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 *
 *******************************************************************************************************/
#pragma  once



#include "stack/ble/ble_common.h"

typedef int (*blc_hci_rx_handler_t) (void);
typedef int (*blc_hci_tx_handler_t) (void);
typedef int (*blc_hci_handler_t) (unsigned char *p, int n);
typedef int (*blc_hci_app_handler_t) (unsigned char *p);



#define			HCI_FLAG_EVENT_PHYTEST_2_WIRE_UART			(1<<23)
#define			HCI_FLAG_EVENT_TLK_MODULE					(1<<24)
#define			HCI_FLAG_EVENT_BT_STD						(1<<25)
#define			HCI_FLAG_EVENT_STACK						(1<<26)
#define			HCI_FLAG_ACL_BT_STD							(1<<27)

#define			TLK_MODULE_EVENT_STATE_CHANGE				0x0730
#define			TLK_MODULE_EVENT_DATA_RECEIVED				0x0731
#define			TLK_MODULE_EVENT_DATA_SEND					0x0732
#define			TLK_MODULE_EVENT_BUFF_AVAILABLE				0x0733




#define			HCI_MAX_ACL_DATA_LEN              			27

#define 		HCI_MAX_DATA_BUFFERS_SALVE              	8
#define 		HCI_MAX_DATA_BUFFERS_MASTER              	8




/**
 *  @brief  Definition for HCI packet type & HCI packet indicator
 */
typedef enum{
	HCI_TYPE_CMD 		= 0x01,
	HCI_TYPE_ACL_DATA	= 0x02,
	HCI_TYPE_SCO_DATA	= 0x03,
	HCI_TYPE_EVENT  	= 0x04,
	HCI_TYPE_ISO_DATA 	= 0x05,  //core_5.2
	//-------- mesh
	HCI_RSP_USER_START 			= 0x10,
	HCI_RSP_USER 				= HCI_RSP_USER_START,	// line feeds
	HCI_RSP_USER_END 			= 0x2F,
	TSCRIPT_MESH_TX				= 0x30,
	TSCRIPT_PROVISION_SERVICE	= 0x31,
	TSCRIPT_PROXY_SERVICE		= 0x32,
	TSCRIPT_END 				= 0x36,
	HCI_LOG 					= 0x3A,	// ":"
	DONGLE_REPORT_SPP_DATA 		= 0x55,
	DONGLE_REPORT_PROVISION_UUID= 0x56,
	DONGLE_REPORT_PROXY_UUID	= 0x57,
	DONGLE_REPORT_ATT_MTU		= 0x58,
    DONGLE_REPORT_ONLINE_ST_UUID= 0x59,
    DONGLE_REPORT_ONLINE_ST_DATA= 0x5a,
    MESH_CMD_RSP 				= 0x70,
    MESH_ADV_PAYLOAD 			= 0x71,
    MESH_PROV 					= 0x72,	// provision parmeters
	MESH_ADV_BEAR_GATT 			= 0x73,
	MESH_ADV_BLE_ST 			= 0x74,
	MESH_MONITOR_DATA 			= 0x75,
	MESH_ADV_ONE_PKT_COMPLETED 	= 0x76,
	MESH_CONNECTION_STS_REPROT 	= 0x77,
	MESH_TX_CMD_RUN_STATUS		= 0x78,
	MESH_GATT_OTA_STATUS 	    = 0x79,
	// can't use 0x7f,  because of HCI_TYPE_TLK_MODULE

	//-------- mesh cmd receive
	TSCRIPT_MESH_RX 			= 0x80,
	TSCRIPT_MESH_RX_NW 			= 0x90,
	
	TSCRIPT_GATEWAY_DIR_RSP 	= 0x91,
	HCI_GATEWAY_CMD_SAR_MSG		= 0x92, 
	TSCRIPT_CMD_VC_DEBUG 		= 0xfa,
	// can't use 0xff,	because of HCI_TYPE_TLK_MODULE
} hci_type_t;


/**
 *  @brief  Definition for HCI ACL Data packets Packet_Boundary_Flag
 */
typedef enum{
	HCI_FIRST_NAF_PACKET          =		0x00,	//LE Host to Controller
	HCI_CONTINUING_PACKET         =		0x01,	//LE Host to Controller / Controller to Host
	HCI_FIRST_AF_PACKET           =    	0x02,	//LE 					  Controller to Host
} acl_pb_flag_t;




/**
 *  @brief  Definition for HCI ISO Data packets PB_Flag
 */
typedef enum{
	HCI_ISO_SDU_FIRST_FRAG		=	0x00,	//The ISO_Data_Load field contains the first fragment of a fragmented SDU
	HCI_ISO_SDU_CONTINUE_FRAG	=	0x01,	//The ISO_Data_Load field contains a continuation fragment of an SDU
	HCI_ISO_SDU_COMPLETE		=	0x02,	//The ISO_Data_Load field contains a complete SDU
	HCI_ISO_SDU_LAST_FRAG		=	0x03,	//The ISO_Data_Load field contains the last fragment of an SDU.
} iso_pb_flag_t;



/**
 *  @brief  Definition for HCI ISO Data packets Packet_Status_Flag
 */
typedef enum{
	HCI_ISO_VALID_DATA				=	0x00, //Valid data. The complete ISO_SDU was received correctly
	HCI_ISO_POSSIBLE_VALID_DATA		=	0x01, //Possibly invalid data
	HCI_ISO_LOST_DATA				=	0x02, //Part(s) of the ISO_SDU were not received correctly. This is reported as "lost data"
} iso_ps_flag_t;








// hci event
extern u32		hci_eventMask;
extern u32		hci_le_eventMask;
extern u32		hci_le_eventMask_2;



// Controller event handler
/**
 * @brief	this function is used to register HCI Event handler Callback function
 */
typedef int (*hci_event_handler_t) (u32 h, u8 *para, int n);
extern hci_event_handler_t		blc_hci_event_handler;












/******************************* Stack Interface Begin, user can not use!!! ********************************************/

/**
 * @brief      this function is used to get data by USB in RX mode for HCI Layer
 * @param[in]  none.
 * @return     0
 */
int blc_hci_rx_from_usb (void);

/**
 * @brief      this function is used to send data by USB in TX mode for HCI Layer
 * @param[in]  none.
 * @return     0
 */
int blc_hci_tx_to_usb (void);


/**
 * @brief      this function is used to process HCI data
 * @param[in]  *p - the pointer of HCI data
 * @param[in]  n - the length of HCI data
 * @return     0
 */
int blc_hci_handler (u8 *p, int n);

/**
 * @brief      this function is used to report HCI events
 * @param[in]  h - HCI Event type
 * @param[in]  *para - data pointer of event
 * @param[in]  n - data length of event
 * @return     none
 */
int blc_hci_send_event (u32 h, u8 *para, int n);

/**
 * @brief      this function is used to process HCI events
 * @param[in]  none.
 * @return     0
 */
int blc_hci_proc (void);

/******************************* Stack Interface End *******************************************************************/







/******************************* User Interface  Begin *****************************************************************/
/**
 * @brief      this function is used to set HCI EVENT mask
 * @param[in]  evtMask  -  HCI��EVENT��mask
 * @return     0
 */
ble_sts_t	blc_hci_setEventMask_cmd(u32 evtMask);      //eventMask: BT/EDR

/**
 * @brief      this function is used to set HCI LE EVENT mask
 * @param[in]  evtMask  -  HCI��LE EVENT��mask(BIT<0-31>)
 * @return     0
 */
ble_sts_t	blc_hci_le_setEventMask_cmd(u32 evtMask);   //eventMask: LE event  0~31

/**
 * @brief      this function is used to set HCI LE EVENT mask
 * @param[in]  evtMask  -  HCI��LE EVENT��mask(BIT<32-63>)
 * @return     0
 */
ble_sts_t 	blc_hci_le_setEventMask_2_cmd(u32 evtMask_2);   //eventMask: LE event 32~63

/**
 * @brief      this function is used to register HCI event handler callback function
 * @param[in]  handler - hci_event_handler_t
 * @return     none.
 */
void 		blc_hci_registerControllerEventHandler (hci_event_handler_t  handler);

/**
 * @brief      this function is used to register HCI TX or RX handler callback function
 * @param[in]  *prx - blc_hci_rx_handler
 * @param[in]  *ptx - blc_hci_tx_handler
 * @return     none.
 */
void 		blc_register_hci_handler (void *prx, void *ptx);

/**
 * @brief      this function is used to send ACL data to HOST
 * @param[in]  handle - connect handle
 * @param[in]  *p - the pointer of l2cap data
 * @return     0
 */
int 		blc_hci_sendACLData2Host (u16 handle, u8 *p);

/**
 * @brief      this function is used to send data
 * @param[in]  h - HCI Event type
 * @param[in]  *para - data pointer of event
 * @param[in]  n - data length of event
 * @return     0,-1
 */
int 		blc_hci_send_data (u32 h, u8 *para, int n);
/******************************* User Interface  End  ******************************************************************/
