/********************************************************************************************************
 * @file	ll_master.h
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
#include "conn_stack.h"

#ifndef LL_MASTER_H_
#define LL_MASTER_H_

/**
 * @brief      This function servers for obtaining l2cap data
 * @param[in]  conn - connect handle
 * @param[in]  *raw_pkt - data pointer of l2cap
 * @return     data pointer of l2cap
 */
rf_packet_l2cap_t * blm_l2cap_packet_pack (u16 conn, u8 * raw_pkt);


/**
 * @brief      for user to initialize ACL connection master role.
 * @param	   none
 * @return     none
 */
void blc_ll_initMasterRoleSingleConn_module(void);


/**
 * @brief      this function is used to terminate an existing connection.
 * @param[in]  handle - indicates which connection is to be disconnected.
 * @param[in]  reason - indicates the reason for ending the connection
 * @return     status, 0x00:  succeed
 * 			           other: failed
 */
ble_sts_t blm_ll_disconnect (u16 handle, u8 reason);


/**
 * @brief      this function is used to change the Link Layer connection parameters of a connection.
 * @param[in]  connHandle - Connection_Handle
 * @param[in]  conn_min - Minimum value for the connection interval.
 * @param[in]  conn_max - Maximum value for the connection interval.
 * @param[in]  conn_latency - Slave latency for the connection in number of connection events
 * @param[in]  timeout - Supervision timeout for the LE Link.
 * @param[in]  ce_min - Information parameter about the minimum length of connection event.
 * @param[in]  ce_max - Information parameter about the maximum length of connection event.
 * @return     status, 0x00:  succeed
 * 			           other: failed
 */
ble_sts_t blm_ll_updateConnection (u16 connHandle,
							  	   u16 conn_min, u16 conn_max, u16 conn_latency, u16 timeout,
							       u16 ce_min, u16 ce_max );


/**
 * @brief      this function is used by the Host to specify a channel classification based on its local information.
 * @param[in]  handle - Connection_Handle
 * @param[in]  *map - channel map
 * @return     status, 0x00:  succeed
 * 			           other: failed
 */
ble_sts_t blm_ll_setHostChannel (u16 handle, u8 * map);



/**
 * @brief      this function is used to detect if RF state machine in PHY layer is working.
 * @param	   none
 * @return     1 : RF state machine is working
 * 			   0 : RF state machine is not working
 */
bool 	blm_ll_isRfStateMachineBusy(void);


#endif /* LL_MASTER_H_ */
