/********************************************************************************************************
 * @file	ll_init.h
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
#ifndef LL_INIT_H_
#define LL_INIT_H_


/**
 * @brief      for user to initialize initiating module
 * 			   notice that only one module can be selected between legacy initiating module and extended initiating module
 * @param	   none
 * @return     none
 */
void	 	blc_ll_initInitiating_module(void);




/**
 * @brief      This function is used to create an ACL connection to a connectable advertiser.
 * @param[in]  scan_interval
 * @param[in]  scan_window
 * @param[in]  initiator_filter_policy
 * @param[in]  adr_type
 * @param[in]  *mac
 * @param[in]  own_adr_type
 * @param[in]  conn_min
 * @param[in]  conn_max
 * @param[in]  conn_latency
 * @param[in]  timeout
 * @param[in]  ce_min
 * @param[in]  ce_max
 * @return     Status - 0x00: command succeeded; 0x01-0xFF: command failed
 */
ble_sts_t 	blc_ll_createConnection ( 	u16 scan_interval, u16 scan_window, init_fp_t initiator_filter_policy,
										u8 adr_type, 	   u8 *mac, 		u8 own_adr_type,
										u16 conn_min, 	   u16 conn_max, 	u16 conn_latency, 					u16 timeout,
										u16 ce_min, 	   u16 ce_max );


/**
 * @brief      This function is is used to cancel the HCI_LE_Create_Connection or HCI_LE_Extended_Create_Connection commands.
 * @param	   none
 * @return     Status - 0x00: command succeeded; 0x01-0xFF: command failed
 */
ble_sts_t 	blc_ll_createConnectionCancel (void);





/**
 * @brief      This function is is used to create the timeout for current connection.
 * @param	   timeout_ms - The length of time, the unit is ms
 * @return     Status - 0x00: succeeded;
 * 						0x01-0xFF: failed
 */
ble_sts_t   blc_ll_setCreateConnectionTimeout (u32 timeout_ms);



#endif /* LL_INIT_H_ */
