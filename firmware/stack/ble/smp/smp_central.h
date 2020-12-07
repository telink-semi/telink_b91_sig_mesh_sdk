/********************************************************************************************************
 * @file	smp_central.h
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
#ifndef SMP_CENTRAL_H_
#define SMP_CENTRAL_H_



#if (LL_MASTER_MULTI_CONNECTION)
	#define	PAIR_SLAVE_MAX_NUM            		8
#else
	#define	PAIR_SLAVE_MAX_NUM            		1
#endif


// 8 byte rand  2 byte ediv		16 byte ltk
#define PAIR_INFO_SECTOR_SIZE	 				64
#define PAIR_OFFSET_RAND		 				8
#define PAIR_OFFSET_EDIV		 				16
#define PAIR_OFFSET_LTK			 				32
#define PAIR_OFFSET_IRK			 				48

#define SLAVE_TRIGGER_SMP_FIRST_PAIRING			0   //first pair, slave send security_request to trigger master's pairing&encryption
#define MASTER_TRIGGER_SMP_FIRST_PAIRING		BIT(0)

#define SLAVE_TRIGGER_SMP_AUTO_CONNECT			0   //auto connect, slave send security_request to trigger master's encryption
#define MASTER_TRIGGER_SMP_AUTO_CONNECT			BIT(1)





/**
 * @brief      This function is used to configure trigger pairing or back-connection encryption.
 * @param[in]  trigger - BIT(0): MASTER_TRIGGER_SMP_FIRST_PAIRING;
 *                       BIT(1): MASTER_TRIGGER_SMP_AUTO_CONNECT.
 * @return     none.
 */
void    blm_host_smp_setSecurityTrigger(u8 trigger);


/**
 * @brief      This function is used to configure the bonding storage address.
 * @param[in]  addr - SMP bonding storage start address.
 * @return     none.
 */
void 	blm_smp_configParingSecurityInfoStorageAddr (int addr);


/**
 * @brief      This function is used to process the SMP protocol interaction process.
 * @param[in]  conn_handle - Connection handle.
 * @param[in]  p - SMP protocol exchange data.
 * @return     none.
 */
void 	blm_host_smp_handler(u16 conn_handle, u8 *p);


/**
 * @brief      This function is used to check whether the input address and address type have been bound.
 * @param[in]  adr_type - Address type.
 * @param[in]  addr - Address.
 * @return     0: not found; others: Index of the paired device.
 */
int 	tbl_bond_slave_search(u8 adr_type, u8 * addr);


/**
 * @brief      This function is used to delete the device binding info corresponding to the specified address and address type from the bonding table.
 * @param[in]  adr_type - Address type.
 * @param[in]  addr - Address.
 * @return     0: Failed to delete; 1: Successfully deleted.
 */
int 	tbl_bond_slave_delete_by_adr(u8 adr_type, u8 *addr);


/**
 * @brief      This function is used to delete the device binding info corresponding to the specified address and address type from the bonding table.
 * @param[in]  adr_type - Address type.
 * @param[in]  addr - Address.
 * @return     none.
 */
void 	tbl_bond_slave_unpair_proc(u8 adr_type, u8 *addr);


/**
 * @brief      This function is used to process SMP encryption change event.
 *             Refer to <<Core spec>> 7.7.8 Encryption Change event for detailed instructions
 * @param[in]  status - 0: An encryption change has occurred;
 *                      1 to 255: An attempt to change encryption failed.
 * @param[in]  conn_handle - Connection handle.
 * @param[in]  enc_enable - Encryption_Enabled.
 * @return     none.
 */
void	blm_smp_encChangeEvt(u8 status, u16 conn_handle, u8 enc_enable);


/**
 * @brief      This function is used to initialize SMP central device.
 * @param[in]  none.
 * @return     0: Initialization failed;
 *             1: initialization succeeded.
 */
int 	blc_smp_central_init (void);



#endif /* SMP_CENTRAL_H_ */
