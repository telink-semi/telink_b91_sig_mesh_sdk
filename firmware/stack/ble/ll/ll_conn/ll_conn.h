/********************************************************************************************************
 * @file	ll_conn.h
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
#ifndef LL_CONN_H_
#define LL_CONN_H_


/**
 * @brief	LE connection handle for slave & master role
 */
#define			BLM_CONN_HANDLE									BIT(7)
#define			BLS_CONN_HANDLE									BIT(6)



/**
 * @brief	LE connection role
 */
#define 		LL_ROLE_MASTER              					0
#define 		LL_ROLE_SLAVE               					1


#define 	BLE_INVALID_CONNECTION_HANDLE    					0xffff
#define 	IS_CONNECTION_HANDLE_VALID(handle)  				( handle != BLE_INVALID_CONNECTION_HANDLE )

/**
 * @brief      for user to initialize ACL connection module.
 * 			   notice that: user must sue this API for both connection slave and master role.
 * @param	   none
 * @return     none
 */
void		blc_ll_initConnection_module(void);


/**
 * @brief      for user to initialize LinkLayer TX FIFO.
 * 			   notice that: size*(number - 1)<= 0xFFF
 * @param[in]  pTxbuf - TX FIFO buffer address.
 * @param[in]  size - TX FIFO size,  must 16 Byte aligned
 * @param[in]  number - TX FIFO number, can only be 9, 17 or 33
 * @return     status, 0x00:  succeed
 * 					   other: failed
 */
ble_sts_t 	blc_ll_initTxFifo(u8 *pTxbuf, int size, int number);


/**
 * @brief      for user to initialize LinkLayer RX FIFO.
 * @param[in]  pTxbuf - RX FIFO buffer address.
 * @param[in]  size - RX FIFO size
 * @param[in]  number - RX FIFO number, can only be 4, 8, 16 or 32
 * @return     status, 0x00:  succeed
 * 					   other: failed
 */
ble_sts_t	blc_ll_initRxFifo(u8 *pRxbuf, int size, int number);



#endif /* LL_CONN_H_ */
