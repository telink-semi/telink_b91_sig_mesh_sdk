/********************************************************************************************************
 * @file	smp_peripheral.h
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
#ifndef SMP_PERIPHERAL_H_
#define SMP_PERIPHERAL_H_



// "SecReq" refer to "security request"
typedef enum {
	SecReq_NOT_SEND = 0,   // do not send "security request" after link layer connection established
	SecReq_IMM_SEND = BIT(0),   //"IMM" refer to immediate, send "security request" immediately after link layer connection established
	SecReq_PEND_SEND = BIT(1),  //"PEND" refer to pending,  pending "security request" for some time after link layer connection established, when pending time arrived. send it
}secReq_cfg;



/**
 * @brief      This function is used to initialize each parameter configuration of SMP, including the initialization of the binding area FLASH.
 * @param[in]  none
 * @return     0: Initialization failed;
 *             1: initialization succeeded.
 */
int 		blc_smp_peripheral_init (void);


/**
 * @brief      This function is used to configure whether the slave sends a Security Request to the master immediately after the connection or after the connection is pending_ms milliseconds, or does not send the Security Request.
 * @param[in]  newConn_cfg - refer to "security request"
 * @param[in]  re_conn_cfg - refer to "security request"
 * @param[in]  pending_ms - Send a Security Request to the master after pending_ms milliseconds
 * @return     none.
 */
void 		blc_smp_configSecurityRequestSending( secReq_cfg newConn_cfg,  secReq_cfg re_conn_cfg, u16 pending_ms);


/**
 * @brief      This function is used to send a Security Request to the master.
 * @param[in]  none
 * @return     1: Push Security Request packet succeeded;
 *             0: Push Security Request packet failed.
 */
int 		blc_smp_sendSecurityRequest (void);




#endif /* SMP_PERIPHERAL_H_ */
