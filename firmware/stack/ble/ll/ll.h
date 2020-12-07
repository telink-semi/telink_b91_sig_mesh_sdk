/********************************************************************************************************
 * @file	ll.h
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
#ifndef LL__H_
#define LL__H_


#include "tl_common.h"
#include "stack/ble/ble_common.h"



/**
 * @brief	BLE link layer state
 */
#define			BLS_LINK_STATE_IDLE								0
#define			BLS_LINK_STATE_ADV								BIT(0)
#define			BLS_LINK_STATE_SCAN								BIT(1)
#define			BLS_LINK_STATE_INIT								BIT(2)
#define 		BLS_LINK_STATE_CONN								BIT(3)

extern 	u8					ble_state;
extern  u8					blt_state;

/**
 * @brief	Telink defined LinkLayer Event Callback Declaration
 */
typedef void (*blt_event_callback_t)(u8 e, u8 *p, int n);


/**
 * @brief	Telink defined LinkLayer Event Type
 */
#define 		BLT_EV_MAX_NUM									20

#define			BLT_EV_FLAG_ADV									0
#define			BLT_EV_FLAG_ADV_DURATION_TIMEOUT				1
#define			BLT_EV_FLAG_SCAN_RSP							2
#define			BLT_EV_FLAG_CONNECT								3
#define			BLT_EV_FLAG_TERMINATE							4
#define			BLT_EV_FLAG_LL_REJECT_IND		    			5
#define			BLT_EV_FLAG_RX_DATA_ABANDOM						6
#define 		BLT_EV_FLAG_PHY_UPDATE							7
#define			BLT_EV_FLAG_DATA_LENGTH_EXCHANGE				8
#define			BLT_EV_FLAG_GPIO_EARLY_WAKEUP					9
#define			BLT_EV_FLAG_CHN_MAP_REQ							10
#define			BLT_EV_FLAG_CONN_PARA_REQ						11
#define			BLT_EV_FLAG_CHN_MAP_UPDATE						12
#define			BLT_EV_FLAG_CONN_PARA_UPDATE					13
#define			BLT_EV_FLAG_SUSPEND_ENTER						14
#define			BLT_EV_FLAG_SUSPEND_EXIT						15


typedef struct {
	u16		connEffectiveMaxRxOctets;
	u16		connEffectiveMaxTxOctets;
	u16 	connMaxRxOctets;
	u16 	connMaxTxOctets;
	u16		connRemoteMaxRxOctets;
	u16 	connRemoteMaxTxOctets;
	u16		supportedMaxRxOctets;
	u16		supportedMaxTxOctets;

	u8	 	connInitialMaxTxOctets;  //u8 is enough
	u8		connMaxTxRxOctets_req;
	u8		connRxDiff100;
	u8		connTxDiff100;
}ll_data_extension_t;

extern _attribute_aligned_(4) ll_data_extension_t  bltData;

/**
 * @brief	This function is used to obtain the effective maximum TX data length
 * @param	none
 * @return	bltData.connEffectiveMaxTxOctets
 */
static inline u8 blc_ll_get_connEffectiveMaxTxOctets(void)
{
	#if (LL_FEATURE_ENABLE_LE_DATA_LENGTH_EXTENSION)
		return bltData.connEffectiveMaxTxOctets;
	#else
		return 27;
	#endif
}

/**
 * @brief	irq_handler for BLE stack, process system tick interrupt and RF interrupt
 * @param	none
 * @return	none
 */
void		irq_blt_sdk_handler (void);


/**
 * @brief   main_loop for BLE stack, process data and event
 * @param	none
 * @return	none
 */
int 		blt_sdk_main_loop(void);




/**
 * @brief   LinkLayer initialization after deepSleep retention wake_up
 * @param	none
 * @return	none
 */
void 		blc_ll_recoverDeepRetention(void);



/**
 * @brief      for user to initialize MCU
 * @param	   none
 * @return     none
 */
void 		blc_ll_initBasicMCU (void);


/**
 * @brief      for user to initialize link layer Standby state
 * @param[in]  *public_adr -  public address pointer
 * @return     none
 */
void 		blc_ll_initStandby_module (u8 *public_adr);




/**
 * @brief      this function is used to set the LE Random Device Address in the Controller
 * @param[in]  *randomAddr -  Random Device Address
 * @return     status, 0x00:  succeed
 * 					   other: failed
 */
ble_sts_t 	blc_ll_setRandomAddr(u8 *randomAddr);

/**
 * @brief      this function is used to exchange data length
 * @param[in]  opcode -  LL_LENGTH_RSP/LL_LENGTH_REQ
 * 			   maxTxOct - Maximum TX packet length
 * @return     status, 0x00:  succeed
 * 					   other: failed
 */
ble_sts_t   blc_ll_exchangeDataLength (u8 opcode, u16 maxTxOct);

/**
 * @brief      this function is used to read MAC address
 * @param[in]  *addr -  The address where the read value(MAC address) prepare to write.
 * @return     status, 0x00:  succeed
 * 					   other: failed
 */
ble_sts_t 	blc_ll_readBDAddr(u8 *addr);

/**
 * @brief      this function is used to get LE stack current state
 * @param[in]  none.
 * @return     blt_state:
 * 					BLS_LINK_STATE_IDLE
 * 					BLS_LINK_STATE_ADV
 * 					BLS_LINK_STATE_SCAN
 * 					BLS_LINK_STATE_INIT
 * 					BLS_LINK_STATE_CONN
 */
u8 			blc_ll_getCurrentState(void);

/**
 * @brief      this function is used to get the most recent average RSSI
 * @param[in]  none.
 * @return     bltParam.ll_recentAvgRSSI
 */
u8 			blc_ll_getLatestAvgRSSI(void);

/**
 * @brief      this function is used to pend Controller event
 * @param[in]  none.
 * @return     blc_tlkEvent_pending
 */
bool		blc_ll_isControllerEventPending(void);

/**
 * @brief      this function is used to get TX FIFO Number of current state
 * @param[in]  none.
 * @return     total_fifo_num
 */
u8  		blc_ll_getTxFifoNumber (void);



/**
 * @brief		this function is used to register LinkLayer Event Callback function
 * @param[in]	e -
 * @param[in]	p -
 * @return		none
 */
void		bls_app_registerEventCallback (u8 e, blt_event_callback_t p);









#endif /* LL__H_ */
